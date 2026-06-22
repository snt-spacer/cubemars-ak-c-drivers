/*
 * nn_demo.c — ncurses TUI bridging a serial-connected neural network (STM32 Nucleo)
 *             to CAN-bus AK-series actuators via SocketCAN.
 *
 * Layout:
 *   Header bar   : serial port/baud | connection state | operating mode | CAN interface
 *   Left panel   : output→CAN mapping table + MIT global params
 *   Right panel  : last NN outputs with CAN destinations | serial stats
 *   Status bar   : key hints + error/status messages
 *
 * Keys:
 *   Tab      toggle ACTIVE (send inputs + recv outputs) / PASSIVE (recv outputs only)
 *   p        edit serial port and baud rate
 *   n        edit NUM_INPUTS / NUM_OUTPUTS
 *   i        edit input vector (scrollable float editor)
 *   a        add output→CAN mapping
 *   e        edit selected mapping
 *   Del      delete selected mapping
 *   k        edit MIT global params (kp / kd / t_ff)
 *   c        edit CAN interface name
 *   u        bring CAN interface up (prompts for bitrate)
 *   d        bring CAN interface down
 *   ↑/↓      navigate mapping table
 *   q        quit
 */

#include <curses.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <termios.h>
#include <fcntl.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ak_mit.h"
#include "ak_servo.h"

/* ── layout constants ───────────────────────────────────────────────────── */

#define LEFT_W          24   /* width of mapping panel (including right border) */
#define HDR_H            2   /* header rows */
#define STATUS_H         2   /* status bar rows at bottom */
#define SERIAL_PORT_MAX 64
#define MAX_MAPPINGS    32
#define MAX_INPUTS     256
#define MAX_OUTPUTS    256
#define DEFAULT_NUM_INPUTS   66
#define DEFAULT_NUM_OUTPUTS  12

/* ── types ──────────────────────────────────────────────────────────────── */

typedef enum { MOTOR_MIT = 0, MOTOR_SERVO = 1 } NNMotorType;

typedef struct {
    int         output_idx;
    uint8_t     can_id;
    NNMotorType motor_type;
    int         enabled;
} OutputMapping;

typedef struct { float kp; float kd; float t_ff; } MITGlobals;

typedef enum { OP_ACTIVE = 0, OP_PIPE = 1 } OpMode;

typedef struct {
    /* serial */
    char serial_port[SERIAL_PORT_MAX];
    int  serial_baud;
    int  serial_fd;

    /* pipe */
    int  pipe_server_fd; /* Unix socket listener in pipe mode, else -1 */
    int  pipe_client_fd; /* accepted Python client connection, else -1 */

    /* CAN */
    char can_iface[16];
    int  can_sock;
    int  can_bitrate;

    /* NN dimensions */
    int num_inputs;
    int num_outputs;

    /* vectors (allocated to MAX_INPUTS / MAX_OUTPUTS) */
    float *input_vector;
    float *output_vector;

    /* pipe mode: partial input-frame accumulation from Python client */
    int pipe_input_offset;

    /* mapping table */
    OutputMapping mappings[MAX_MAPPINGS];
    int           n_mappings;
    int           selected_mapping;

    /* mode */
    OpMode op_mode;

    /* MIT globals */
    MITGlobals mit;

    /* stats */
    long   frames_sent;
    int    serial_ok;
    int    motors_enabled; /* 1 if MIT enter has been sent */
    double last_infer_ms;  /* last serial round-trip latency (ms) */
    double infer_hz;       /* inferred inference frequency (Hz) */

    /* status bar */
    char status_msg[128];
    int  status_err;
} NNAppState;

/* ── SocketCAN helpers (from demo.c) ────────────────────────────────────── */

static int nn_can_open(const char *iface)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { close(s); return -1; }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(s); return -1;
    }
    return s;
}

static int nn_can_send(int sock, int is_eff, uint32_t id,
                       const uint8_t *data, uint8_t len)
{
    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id  = is_eff ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    f.can_dlc = len;
    memcpy(f.data, data, len);
    return (write(sock, &f, sizeof(f)) == (ssize_t)sizeof(f)) ? 0 : -1;
}

static int nn_iface_is_up(const char *iface)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return 0;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    int r = ioctl(s, SIOCGIFFLAGS, &ifr);
    close(s);
    if (r < 0) return 0;
    return (ifr.ifr_flags & IFF_UP) ? 1 : 0;
}

static int nn_run_cmd(const char *cmd)
{
    char buf[512];
    snprintf(buf, sizeof(buf), "%s >/dev/null 2>&1", cmd);
    return system(buf);
}

/* ── serial helpers ─────────────────────────────────────────────────────── */

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return B115200;
    }
}

static int serial_open(const char *port, int baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return -1; }
    return fd;
}

static int serial_write_exact(int fd, const void *buf, size_t len)
{
    size_t written = 0;
    const uint8_t *p = (const uint8_t *)buf;
    while (written < len) {
        ssize_t n = write(fd, p + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        written += (size_t)n;
    }
    return 0;
}

/* Blocking read of exactly len bytes with timeout_ms deadline. */
static int serial_read_exact(int fd, void *buf, size_t len, int timeout_ms)
{
    size_t got = 0;
    uint8_t *p = (uint8_t *)buf;

    struct timeval deadline;
    gettimeofday(&deadline, NULL);
    deadline.tv_usec += timeout_ms * 1000;
    deadline.tv_sec  += deadline.tv_usec / 1000000;
    deadline.tv_usec %= 1000000;

    while (got < len) {
        struct timeval now, rem;
        gettimeofday(&now, NULL);
        rem.tv_sec  = deadline.tv_sec  - now.tv_sec;
        rem.tv_usec = deadline.tv_usec - now.tv_usec;
        if (rem.tv_usec < 0) { rem.tv_sec--; rem.tv_usec += 1000000; }
        if (rem.tv_sec < 0) return -1;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int r = select(fd + 1, &rfds, NULL, NULL, &rem);
        if (r <= 0) return -1;

        ssize_t n = read(fd, p + got, len - got);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        got += (size_t)n;
    }
    return 0;
}

/* Non-blocking read: returns bytes read (>=0), or -1 on error. */
static ssize_t serial_read_nonblock(int fd, void *buf, size_t len)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    struct timeval tv = {0, 0};
    int r = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (r <= 0) return (r == 0) ? 0 : -1;
    return read(fd, buf, len);
}

/* ── Unix socket pipe server ────────────────────────────────────────────── */

#define PIPE_SOCKET_PATH "/tmp/nn_demo.sock"

/* Create a non-blocking Unix domain socket server. Returns listener fd or -1. */
static int open_pipe_server(const char *path)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
    unlink(path);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(fd); return -1; }
    chmod(path, 0666);
    if (listen(fd, 1) < 0) { close(fd); return -1; }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    return fd;
}

/* ── NN protocol ────────────────────────────────────────────────────────── */

/* ACTIVE: write inputs, blocking-read outputs (40 ms budget). */
static int serial_exchange(NNAppState *s)
{
    size_t in_bytes  = (size_t)s->num_inputs  * sizeof(float);
    size_t out_bytes = (size_t)s->num_outputs * sizeof(float);

    struct timeval t0, t1;
    gettimeofday(&t0, NULL);

    if (serial_write_exact(s->serial_fd, s->input_vector, in_bytes) < 0)
        return -1;
    int r = serial_read_exact(s->serial_fd, s->output_vector, out_bytes, 40);

    if (r == 0) {
        gettimeofday(&t1, NULL);
        double elapsed_ms = (double)(t1.tv_sec  - t0.tv_sec)  * 1000.0
                          + (double)(t1.tv_usec - t0.tv_usec) / 1000.0;
        /* Ignore implausibly small samples (kernel scheduling noise). */
        if (elapsed_ms > 0.5) {
            double raw_hz = 1000.0 / elapsed_ms;
            if (s->infer_hz <= 0.0) {
                s->infer_hz = raw_hz;
            } else {
                /* EMA: weight new sample at 15 %, keep 85 % of history. */
                s->infer_hz = 0.15 * raw_hz + 0.85 * s->infer_hz;
            }
            s->last_infer_ms = 1000.0 / s->infer_hz;
        }
    }
    return r;
}

/* ── CAN dispatch ───────────────────────────────────────────────────────── */

static void dispatch_can(NNAppState *s)
{
    if (s->can_sock < 0) return;
    for (int i = 0; i < s->n_mappings; i++) {
        const OutputMapping *m = &s->mappings[i];
        if (!m->enabled) continue;
        if (m->output_idx < 0 || m->output_idx >= s->num_outputs) continue;
        float val = s->output_vector[m->output_idx];
        if (m->motor_type == MOTOR_MIT) {
            AKMotorMITMessage msg = generate_mit_command_message(
                m->can_id, val, 0.0f, s->mit.kp, s->mit.kd, s->mit.t_ff);
            nn_can_send(s->can_sock, 0, (uint32_t)msg.standard_id, msg.data, msg.len);
        } else {
            /* Servo mode expects degrees; Nucleo outputs radians */
            float deg = val * 57.29577951f;  /* val * (180 / pi) */
            AKMotorServoMessage msg = generate_position_message(m->can_id, deg);
            nn_can_send(s->can_sock, 1, msg.extended_id, msg.data, msg.len);
        }
        s->frames_sent++;
    }
}

/* PIPE: non-blocking accumulate input_vector from Python client.
 * When a full input frame arrives: serial_exchange with STM32, write outputs
 * back to Python, dispatch CAN.
 * Returns 1 on full round-trip, 0 if partial/nothing, -1 on error. */
static int pipe_io(NNAppState *s)
{
    size_t in_bytes  = (size_t)s->num_inputs * sizeof(float);
    uint8_t *dest    = (uint8_t *)s->input_vector + s->pipe_input_offset;
    size_t remaining = in_bytes - (size_t)s->pipe_input_offset;

    ssize_t n = serial_read_nonblock(s->pipe_client_fd, dest, remaining);
    if (n < 0) return -1;
    if (n == 0) return 0;
    s->pipe_input_offset += (int)n;
    if (s->pipe_input_offset < (int)in_bytes) return 0;

    /* Full input frame received — exchange with STM32 */
    s->pipe_input_offset = 0;
    if (serial_exchange(s) < 0) return -1;

    /* Write outputs back to Python */
    size_t out_bytes = (size_t)s->num_outputs * sizeof(float);
    if (serial_write_exact(s->pipe_client_fd, s->output_vector, out_bytes) < 0) return -1;

    dispatch_can(s);
    return 1;
}

/* Send MIT enter or exit to every enabled MIT-mode mapping. */
static void send_mit_motor_mode(NNAppState *s, int enable)
{
    if (s->can_sock < 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "CAN not connected — press [c] or [u] to configure");
        s->status_err = 1;
        return;
    }
    int count = 0;
    for (int i = 0; i < s->n_mappings; i++) {
        const OutputMapping *m = &s->mappings[i];
        if (!m->enabled || m->motor_type != MOTOR_MIT) continue;
        AKMotorMITMessage msg = enable
            ? generate_mit_enter_message(m->can_id)
            : generate_mit_exit_message(m->can_id);
        nn_can_send(s->can_sock, 0, (uint32_t)msg.standard_id, msg.data, msg.len);
        count++;
    }
    s->motors_enabled = enable;
    if (count == 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "No enabled MIT mappings to %s", enable ? "enter" : "exit");
        s->status_err = 1;
    } else {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "MIT motor mode %s (%d motor%s)",
                 enable ? "ENABLED" : "DISABLED", count, count == 1 ? "" : "s");
        s->status_err = 0;
    }
}

/* ── CAN ID scan ────────────────────────────────────────────────────────── */

static void can_scan_popup(NNAppState *s)
{
    if (s->can_sock < 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "CAN not connected — press [c] or [u] to configure");
        s->status_err = 1;
        return;
    }

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int pop_h = 20, pop_w = 50;
    WINDOW *pop = newwin(pop_h, pop_w,
                         (rows - pop_h) / 2, (cols - pop_w) / 2);
    keypad(pop, TRUE);
    nodelay(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " CAN ID Scan ");

    uint32_t found_id[64];
    int      found_eff[64];
    int      n_found = 0;

    struct timeval t_start, t_now;
    gettimeofday(&t_start, NULL);
    const int scan_ms = 3000;
    int bar_w = pop_w - 6;

    /* ── scan loop: exits when time is up or Esc ── */
    for (;;) {
        gettimeofday(&t_now, NULL);
        int elapsed_ms = (int)((t_now.tv_sec  - t_start.tv_sec)  * 1000 +
                               (t_now.tv_usec - t_start.tv_usec) / 1000);
        if (elapsed_ms >= scan_ms) break;
        int remaining_ms = scan_ms - elapsed_ms;

        int filled = (elapsed_ms * bar_w) / scan_ms;
        if (filled > bar_w) filled = bar_w;
        mvwprintw(pop, 2, 2, "Listening... %d.%ds remaining   ",
                  remaining_ms / 1000, (remaining_ms % 1000) / 100);
        wmove(pop, 3, 2);
        waddch(pop, '[');
        for (int i = 0; i < bar_w; i++)
            waddch(pop, i < filled ? ACS_CKBOARD : ' ');
        waddch(pop, ']');

        mvwprintw(pop, 5, 2, "Found %d ID(s):", n_found);
        for (int i = 0; i < n_found && i < pop_h - 9; i++) {
            uint32_t mid = found_eff[i] ? (found_id[i] & 0xFF) : found_id[i];
            mvwprintw(pop, 6 + i, 4, "0x%03X (%3d)  %s",
                      mid, mid,
                      found_eff[i] ? "Servo/EFF" : "MIT/SFF  ");
        }
        mvwprintw(pop, pop_h - 2, 2, "[Esc] stop early");
        wrefresh(pop);

        if (wgetch(pop) == 27) break;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s->can_sock, &rfds);
        struct timeval tv = { 0, 50000 };
        if (select(s->can_sock + 1, &rfds, NULL, NULL, &tv) == 1) {
            struct can_frame f;
            if (read(s->can_sock, &f, sizeof(f)) == (ssize_t)sizeof(f)) {
                int is_eff  = (f.can_id & CAN_EFF_FLAG) ? 1 : 0;
                uint32_t id = f.can_id & (is_eff ? CAN_EFF_MASK : CAN_SFF_MASK);
                int seen = 0;
                for (int i = 0; i < n_found; i++) {
                    if (found_id[i] == id && found_eff[i] == is_eff) { seen = 1; break; }
                }
                if (!seen && n_found < 64) {
                    found_id[n_found]  = id;
                    found_eff[n_found] = is_eff;
                    n_found++;
                }
            }
        }
    }

    /* ── results screen: blocking wait for any key ── */
    nodelay(pop, FALSE);
    werase(pop);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " CAN ID Scan: Results ");
    wmove(pop, 3, 2);
    waddch(pop, '[');
    for (int i = 0; i < bar_w; i++) waddch(pop, ACS_CKBOARD);
    waddch(pop, ']');
    mvwprintw(pop, 2, 2, "Scan complete. Found %d ID(s):", n_found);
    if (n_found == 0) {
        mvwprintw(pop, 5, 4, "(no frames received)");
    } else {
        for (int i = 0; i < n_found && i < pop_h - 9; i++) {
            uint32_t mid = found_eff[i] ? (found_id[i] & 0xFF) : found_id[i];
            mvwprintw(pop, 5 + i, 4, "0x%03X (%3d)  %s",
                      mid, mid,
                      found_eff[i] ? "Servo/EFF" : "MIT/SFF  ");
        }
    }
    mvwprintw(pop, pop_h - 2, 2, "[any key] close");
    wrefresh(pop);
    wgetch(pop);

    snprintf(s->status_msg, sizeof(s->status_msg),
             "Scan complete: %d ID(s) found", n_found);
    s->status_err = 0;

    delwin(pop);
    touchwin(stdscr);
    refresh();
}

/* ── popup editors ──────────────────────────────────────────────────────── */

static void edit_serial_popup(NNAppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(9, 52, (rows - 9) / 2, (cols - 52) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " Serial Connection ");

    char port_buf[SERIAL_PORT_MAX];
    char baud_buf[12];
    strncpy(port_buf, s->serial_port, sizeof(port_buf) - 1);
    port_buf[sizeof(port_buf) - 1] = '\0';
    snprintf(baud_buf, sizeof(baud_buf), "%d", s->serial_baud);

    int field = 0;

    for (;;) {
        mvwprintw(pop, 2, 2, "Port:");
        mvwprintw(pop, 4, 2, "Baud:");
        mvwprintw(pop, 7, 2, "[Tab] switch  [Enter] connect  [Esc] cancel");

        if (field == 0) wattron(pop, A_REVERSE);
        mvwprintw(pop, 3, 2, "  %-46s", port_buf);
        if (field == 0) wattroff(pop, A_REVERSE);

        if (field == 1) wattron(pop, A_REVERSE);
        mvwprintw(pop, 5, 2, "  %-46s", baud_buf);
        if (field == 1) wattroff(pop, A_REVERSE);

        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) { delwin(pop); touchwin(stdscr); refresh(); return; }
        if (ch == '\t' || ch == KEY_DOWN || ch == KEY_UP) field = 1 - field;

        if (ch == '\n' || ch == KEY_ENTER) {
            strncpy(s->serial_port, port_buf, sizeof(s->serial_port) - 1);
            s->serial_port[sizeof(s->serial_port) - 1] = '\0';
            int b = atoi(baud_buf);
            if (b > 0) s->serial_baud = b;
            if (s->serial_fd >= 0) { close(s->serial_fd); s->serial_fd = -1; }
            s->pipe_input_offset = 0;
            /* [p] only reconnects serial; pipe transport is managed by mode (Tab) */
            if (s->op_mode == OP_ACTIVE) {
                s->serial_fd = serial_open(s->serial_port, s->serial_baud);
                s->serial_ok = (s->serial_fd >= 0);
                if (!s->serial_ok) {
                    snprintf(s->status_msg, sizeof(s->status_msg),
                             "Cannot open %s: %s", s->serial_port, strerror(errno));
                    s->status_err = 1;
                } else {
                    snprintf(s->status_msg, sizeof(s->status_msg),
                             "Serial %s @ %d baud", s->serial_port, s->serial_baud);
                    s->status_err = 0;
                }
            } else {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Port saved (takes effect in ACTIVE mode)");
                s->status_err = 0;
            }
            delwin(pop); touchwin(stdscr); refresh();
            return;
        }

        char *buf   = (field == 0) ? port_buf : baud_buf;
        size_t bsz  = (field == 0) ? sizeof(port_buf) : sizeof(baud_buf);
        if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if (field == 0 && ch >= 32 && ch < 127) {
            size_t l = strlen(buf);
            if (l < bsz - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
        } else if (field == 1 && ch >= '0' && ch <= '9') {
            size_t l = strlen(buf);
            if (l < bsz - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
        }
    }
}

static void edit_dims_popup(NNAppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(9, 44, (rows - 9) / 2, (cols - 44) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " NN Dimensions ");

    char in_buf[8], out_buf[8];
    snprintf(in_buf,  sizeof(in_buf),  "%d", s->num_inputs);
    snprintf(out_buf, sizeof(out_buf), "%d", s->num_outputs);

    int field = 0;

    for (;;) {
        mvwprintw(pop, 2, 2, "Number of inputs  (1..%d):", MAX_INPUTS);
        mvwprintw(pop, 4, 2, "Number of outputs (1..%d):", MAX_OUTPUTS);
        mvwprintw(pop, 7, 2, "[Tab] switch  [Enter] set  [Esc] cancel");

        if (field == 0) wattron(pop, A_REVERSE);
        mvwprintw(pop, 3, 2, "  %-38s", in_buf);
        if (field == 0) wattroff(pop, A_REVERSE);

        if (field == 1) wattron(pop, A_REVERSE);
        mvwprintw(pop, 5, 2, "  %-38s", out_buf);
        if (field == 1) wattroff(pop, A_REVERSE);

        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) { delwin(pop); touchwin(stdscr); refresh(); return; }
        if (ch == '\t' || ch == KEY_DOWN || ch == KEY_UP) field = 1 - field;

        if (ch == '\n' || ch == KEY_ENTER) {
            int ni = atoi(in_buf);
            int no = atoi(out_buf);
            if (ni >= 1 && ni <= MAX_INPUTS)  s->num_inputs  = ni;
            if (no >= 1 && no <= MAX_OUTPUTS) s->num_outputs = no;
            s->pipe_input_offset = 0;
            snprintf(s->status_msg, sizeof(s->status_msg),
                     "Dims: %d inputs, %d outputs", s->num_inputs, s->num_outputs);
            s->status_err = 0;
            delwin(pop); touchwin(stdscr); refresh();
            return;
        }

        char *buf  = (field == 0) ? in_buf : out_buf;
        size_t bsz = sizeof(in_buf);
        if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if (ch >= '0' && ch <= '9') {
            size_t l = strlen(buf);
            if (l < bsz - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
        }
    }
}

static void edit_inputs_popup(NNAppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int pop_h = rows - 4;
    int pop_w = 46;
    if (pop_h < 8) pop_h = 8;

    WINDOW *pop = newwin(pop_h, pop_w,
                         (rows - pop_h) / 2, (cols - pop_w) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " Input Vector (%d floats) ", s->num_inputs);

    int visible = pop_h - 4;
    int scroll  = 0;
    int cursor  = 0;
    int editing = 0;
    char edit_buf[32] = "";

    for (;;) {
        mvwprintw(pop, pop_h - 2, 2,
                  "[Arrows] nav  [Enter] edit  [Esc] %s",
                  editing ? "cancel edit" : "close");

        for (int row = 0; row < visible; row++) {
            int idx = scroll + row;
            int y   = 1 + row;
            wmove(pop, y, 1);
            wclrtoeol(pop);
            mvwaddch(pop, y, pop_w - 1, ACS_VLINE);
            if (idx >= s->num_inputs) continue;

            if (idx == cursor && editing) {
                mvwprintw(pop, y, 2, "[%3d] ", idx);
                wattron(pop, A_REVERSE);
                wprintw(pop, "%-34s", edit_buf);
                wattroff(pop, A_REVERSE);
            } else if (idx == cursor) {
                wattron(pop, A_REVERSE | A_BOLD);
                mvwprintw(pop, y, 2, "[%3d] %12.6f", idx,
                          (double)s->input_vector[idx]);
                wattroff(pop, A_REVERSE | A_BOLD);
            } else {
                mvwprintw(pop, y, 2, "[%3d] %12.6f", idx,
                          (double)s->input_vector[idx]);
            }
        }
        wrefresh(pop);

        int ch = wgetch(pop);

        if (editing) {
            if (ch == 27) {
                editing = 0;
                edit_buf[0] = '\0';
            } else if (ch == '\n' || ch == KEY_ENTER) {
                s->input_vector[cursor] = (float)atof(edit_buf);
                editing = 0;
                edit_buf[0] = '\0';
            } else if (ch == KEY_BACKSPACE || ch == 127) {
                size_t l = strlen(edit_buf);
                if (l > 0) edit_buf[l - 1] = '\0';
            } else if ((ch >= '0' && ch <= '9') || ch == '-' ||
                       ch == '.' || ch == 'e' || ch == '+') {
                size_t l = strlen(edit_buf);
                if (l < sizeof(edit_buf) - 1) {
                    edit_buf[l] = (char)ch;
                    edit_buf[l+1] = '\0';
                }
            }
        } else {
            if (ch == 27) { delwin(pop); touchwin(stdscr); refresh(); return; }
            if (ch == '\n' || ch == KEY_ENTER) {
                snprintf(edit_buf, sizeof(edit_buf), "%g",
                         (double)s->input_vector[cursor]);
                editing = 1;
            }
            if (ch == KEY_UP && cursor > 0) {
                cursor--;
                if (cursor < scroll) scroll = cursor;
            }
            if (ch == KEY_DOWN && cursor < s->num_inputs - 1) {
                cursor++;
                if (cursor >= scroll + visible) scroll = cursor - visible + 1;
            }
        }
    }
}

static void edit_mapping_popup(NNAppState *s, int idx)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int output_idx  = (idx >= 0) ? s->mappings[idx].output_idx : s->n_mappings;
    int can_id_val  = (idx >= 0) ? (int)s->mappings[idx].can_id : 1;
    NNMotorType mt  = (idx >= 0) ? s->mappings[idx].motor_type : MOTOR_MIT;
    int enabled_val = (idx >= 0) ? s->mappings[idx].enabled : 1;

    char out_buf[8], cid_buf[8];
    snprintf(out_buf, sizeof(out_buf), "%d", output_idx);
    snprintf(cid_buf, sizeof(cid_buf), "%d", can_id_val);

    WINDOW *pop = newwin(11, 52, (rows - 11) / 2, (cols - 52) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, idx >= 0 ? " Edit Mapping " : " Add Mapping ");

    int field = 0;

    for (;;) {
        mvwprintw(pop, 2, 2, "Output index (0..%d):", s->num_outputs - 1);
        mvwprintw(pop, 4, 2, "CAN ID (1..255):");
        mvwprintw(pop, 9, 2, "[Tab/Arrows] nav  [Enter] ok  [Esc] cancel");

        if (field == 0) wattron(pop, A_REVERSE);
        mvwprintw(pop, 3, 2, "  %-46s", out_buf);
        if (field == 0) wattroff(pop, A_REVERSE);

        if (field == 1) wattron(pop, A_REVERSE);
        mvwprintw(pop, 5, 2, "  %-46s", cid_buf);
        if (field == 1) wattroff(pop, A_REVERSE);

        if (field == 2) wattron(pop, A_REVERSE);
        mvwprintw(pop, 6, 2, "  Motor: %-10s  [Space] toggle",
                  mt == MOTOR_MIT ? "MIT" : "Servo");
        if (field == 2) wattroff(pop, A_REVERSE);
        else mvwprintw(pop, 6, 2, "  Motor: %-10s  [Space] toggle",
                       mt == MOTOR_MIT ? "MIT" : "Servo");

        if (field == 3) wattron(pop, A_REVERSE);
        mvwprintw(pop, 7, 2, "  Enabled: %-6s        [Space] toggle",
                  enabled_val ? "Yes" : "No");
        if (field == 3) wattroff(pop, A_REVERSE);
        else mvwprintw(pop, 7, 2, "  Enabled: %-6s        [Space] toggle",
                       enabled_val ? "Yes" : "No");

        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) { delwin(pop); touchwin(stdscr); refresh(); return; }
        if (ch == '\t' || ch == KEY_DOWN) field = (field + 1) % 4;
        else if (ch == KEY_UP) field = (field - 1 + 4) % 4;

        if (ch == ' ') {
            if (field == 2) mt = (mt == MOTOR_MIT) ? MOTOR_SERVO : MOTOR_MIT;
            if (field == 3) enabled_val = 1 - enabled_val;
        }

        if (ch == '\n' || ch == KEY_ENTER) {
            int oi = atoi(out_buf);
            int ci = atoi(cid_buf);
            if (oi < 0 || oi >= s->num_outputs) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Output index out of range (0..%d)", s->num_outputs - 1);
                s->status_err = 1;
                delwin(pop); touchwin(stdscr); refresh();
                return;
            }
            if (ci < 1 || ci > 255) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "CAN ID must be 1..255");
                s->status_err = 1;
                delwin(pop); touchwin(stdscr); refresh();
                return;
            }
            OutputMapping *m;
            if (idx >= 0) {
                m = &s->mappings[idx];
            } else {
                if (s->n_mappings >= MAX_MAPPINGS) {
                    snprintf(s->status_msg, sizeof(s->status_msg),
                             "Max %d mappings reached", MAX_MAPPINGS);
                    s->status_err = 1;
                    delwin(pop); touchwin(stdscr); refresh();
                    return;
                }
                m = &s->mappings[s->n_mappings++];
            }
            m->output_idx = oi;
            m->can_id     = (uint8_t)ci;
            m->motor_type = mt;
            m->enabled    = enabled_val;
            snprintf(s->status_msg, sizeof(s->status_msg),
                     "Mapping: out[%d] → CAN %d (%s)",
                     oi, ci, mt == MOTOR_MIT ? "MIT" : "Servo");
            s->status_err = 0;
            delwin(pop); touchwin(stdscr); refresh();
            return;
        }

        /* edit integer fields */
        char *buf  = NULL;
        size_t bsz = 0;
        if (field == 0) { buf = out_buf; bsz = sizeof(out_buf); }
        if (field == 1) { buf = cid_buf; bsz = sizeof(cid_buf); }
        if (buf && ch != '\t' && ch != KEY_DOWN && ch != KEY_UP && ch != ' ') {
            if (ch == KEY_BACKSPACE || ch == 127) {
                size_t l = strlen(buf);
                if (l > 0) buf[l - 1] = '\0';
            } else if (ch >= '0' && ch <= '9') {
                size_t l = strlen(buf);
                if (l < bsz - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
            }
        }
    }
}

static void edit_mit_globals_popup(NNAppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(11, 44, (rows - 11) / 2, (cols - 44) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " MIT Global Params ");

    char kp_buf[16], kd_buf[16], tff_buf[16];
    snprintf(kp_buf,  sizeof(kp_buf),  "%g", (double)s->mit.kp);
    snprintf(kd_buf,  sizeof(kd_buf),  "%g", (double)s->mit.kd);
    snprintf(tff_buf, sizeof(tff_buf), "%g", (double)s->mit.t_ff);

    int field = 0;

    for (;;) {
        mvwprintw(pop, 2, 2, "kp   (position gain):");
        mvwprintw(pop, 4, 2, "kd   (velocity gain):");
        mvwprintw(pop, 6, 2, "t_ff (feed-forward torque N*m):");
        mvwprintw(pop, 9, 2, "[Tab/Arrows] field  [Enter] set  [Esc] cancel");

        char *bufs[3]    = { kp_buf, kd_buf, tff_buf };
        const int ys[3]  = { 3, 5, 7 };

        for (int i = 0; i < 3; i++) {
            if (i == field) wattron(pop, A_REVERSE);
            mvwprintw(pop, ys[i], 2, "  %-38s", bufs[i]);
            if (i == field) wattroff(pop, A_REVERSE);
        }
        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) { delwin(pop); touchwin(stdscr); refresh(); return; }
        if (ch == '\t' || ch == KEY_DOWN) field = (field + 1) % 3;
        else if (ch == KEY_UP) field = (field - 1 + 3) % 3;

        if (ch == '\n' || ch == KEY_ENTER) {
            s->mit.kp  = (float)atof(kp_buf);
            s->mit.kd  = (float)atof(kd_buf);
            s->mit.t_ff = (float)atof(tff_buf);
            snprintf(s->status_msg, sizeof(s->status_msg),
                     "MIT: kp=%.2f kd=%.2f t_ff=%.2f",
                     (double)s->mit.kp, (double)s->mit.kd, (double)s->mit.t_ff);
            s->status_err = 0;
            delwin(pop); touchwin(stdscr); refresh();
            return;
        }

        char *buf  = bufs[field];
        size_t bsz = sizeof(kp_buf);
        if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if ((ch >= '0' && ch <= '9') || ch == '-' ||
                   ch == '.' || ch == 'e' || ch == '+') {
            size_t l = strlen(buf);
            if (l < bsz - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
        }
    }
}

static void edit_can_iface_popup(NNAppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(5, 42, (rows - 5) / 2, (cols - 42) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " CAN Interface ");

    char buf[16];
    strncpy(buf, s->can_iface, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    for (;;) {
        wattron(pop, A_REVERSE);
        mvwprintw(pop, 2, 2, "  %-36s", buf);
        wattroff(pop, A_REVERSE);
        mvwprintw(pop, 3, 2, "[Enter] connect  [Esc] cancel");
        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) break;
        if (ch == '\n' || ch == KEY_ENTER) {
            strncpy(s->can_iface, buf, sizeof(s->can_iface) - 1);
            s->can_iface[sizeof(s->can_iface) - 1] = '\0';
            if (s->can_sock >= 0) { close(s->can_sock); s->can_sock = -1; }
            s->can_sock = nn_can_open(s->can_iface);
            if (s->can_sock < 0) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Cannot open CAN %s: %s", s->can_iface, strerror(errno));
                s->status_err = 1;
            } else {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Connected to CAN %s", s->can_iface);
                s->status_err = 0;
            }
            break;
        } else if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if (ch >= 32 && ch < 127) {
            size_t l = strlen(buf);
            if (l < sizeof(buf) - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
        }
    }

    delwin(pop);
    touchwin(stdscr);
    refresh();
}

static void can_iface_up_popup(NNAppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(6, 46, (rows - 6) / 2, (cols - 46) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " Bring up %s ", s->can_iface);

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", s->can_bitrate);

    for (;;) {
        mvwprintw(pop, 2, 2, "Bitrate [bps]:");
        wattron(pop, A_REVERSE);
        mvwprintw(pop, 3, 2, "  %-40s", buf);
        wattroff(pop, A_REVERSE);
        mvwprintw(pop, 4, 2, "[Enter] up  [Esc] cancel");
        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) { delwin(pop); touchwin(stdscr); refresh(); return; }
        if (ch == '\n' || ch == KEY_ENTER) {
            int br = atoi(buf);
            if (br <= 0) br = 1000000;
            s->can_bitrate = br;
            delwin(pop); touchwin(stdscr); refresh();

            char cmd[128];
            snprintf(cmd, sizeof(cmd),
                     "ip link set %s type can bitrate %d && ip link set %s up",
                     s->can_iface, s->can_bitrate, s->can_iface);
            int rc = nn_run_cmd(cmd);
            if (rc != 0) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "ip link up failed (rc=%d) — need root?", rc);
                s->status_err = 1;
                return;
            }
            if (s->can_sock >= 0) { close(s->can_sock); s->can_sock = -1; }
            s->can_sock = nn_can_open(s->can_iface);
            if (s->can_sock < 0) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Up OK but socket failed: %s", strerror(errno));
                s->status_err = 1;
            } else {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "%s up @ %d bps", s->can_iface, s->can_bitrate);
                s->status_err = 0;
            }
            return;
        } else if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if (ch >= '0' && ch <= '9') {
            size_t l = strlen(buf);
            if (l < sizeof(buf) - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
        }
    }
}

/* ── drawing ────────────────────────────────────────────────────────────── */

static void draw_header(const NNAppState *s, int cols)
{
    int can_up = nn_iface_is_up(s->can_iface);

    attron(A_BOLD | A_REVERSE);
    move(0, 0);
    for (int i = 0; i < cols; i++) addch(' ');
    if (s->op_mode == OP_PIPE) {
        mvprintw(0, 1,
                 "NN Bridge  |  PIPE [%s]  Serial: %s [%s]  |  Mode: PIPE  |  CAN: %s [%s]",
                 s->pipe_client_fd >= 0 ? "CONNECTED" : "WAITING",
                 s->serial_port, s->serial_ok ? "OK" : "--",
                 s->can_iface, can_up ? "UP" : "DOWN");
    } else {
        mvprintw(0, 1,
                 "NN Bridge  |  Serial: %s:%d [%s]  |  Mode: ACTIVE  |  CAN: %s [%s]",
                 s->serial_port, s->serial_baud,
                 s->serial_ok ? "OK" : "--",
                 s->can_iface, can_up ? "UP" : "DOWN");
    }
    attroff(A_BOLD | A_REVERSE);

    move(1, 0);
    for (int i = 0; i < cols; i++) addch(ACS_HLINE);
    mvaddch(1, LEFT_W - 1, ACS_TTEE);
}

static void draw_mapping_panel(const NNAppState *s, int rows)
{
    int bot = rows - STATUS_H;

    /* vertical border */
    for (int r = HDR_H; r < bot; r++) mvaddch(r, LEFT_W - 1, ACS_VLINE);
    mvaddch(bot, LEFT_W - 1, ACS_BTEE);

    /* column header */
    int y = HDR_H;
    attron(A_UNDERLINE);
    mvprintw(y, 1, " Out CAN  Type En");
    attroff(A_UNDERLINE);
    mvaddch(y, LEFT_W - 1, ACS_VLINE);
    y++;

    /* leave 4 rows at bottom: separator + 2 MIT lines + 1 padding */
    int map_bot = bot - 4;

    for (int i = 0; i < s->n_mappings && y < map_bot; i++, y++) {
        move(y, 0); clrtoeol();
        if (i == s->selected_mapping) attron(A_REVERSE | A_BOLD);
        mvprintw(y, 0, "%c%3d %3d  %3s   %c",
                 i == s->selected_mapping ? '>' : ' ',
                 s->mappings[i].output_idx,
                 (int)s->mappings[i].can_id,
                 s->mappings[i].motor_type == MOTOR_MIT ? "MIT" : "Srv",
                 s->mappings[i].enabled ? 'Y' : 'N');
        if (i == s->selected_mapping) attroff(A_REVERSE | A_BOLD);
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
    }

    for (; y < map_bot; y++) {
        move(y, 0); clrtoeol();
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
    }

    /* separator above MIT globals */
    move(map_bot, 0);
    for (int i = 0; i < LEFT_W - 1; i++) addch(ACS_HLINE);
    mvaddch(map_bot, LEFT_W - 1, ACS_RTEE);
    y = map_bot + 1;

    /* MIT globals mini-display */
    attron(A_DIM);
    mvprintw(y, 1, "kp=%-5.2f kd=%-5.2f",
             (double)s->mit.kp, (double)s->mit.kd);
    mvaddch(y, LEFT_W - 1, ACS_VLINE);
    y++;
    mvprintw(y, 1, "t_ff=%-10.4f", (double)s->mit.t_ff);
    mvaddch(y, LEFT_W - 1, ACS_VLINE);
    attroff(A_DIM);
    y++;

    for (; y < bot; y++) {
        move(y, 0); clrtoeol();
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
    }
}

static void draw_output_panel(const NNAppState *s, int rows, int cols)
{
    int rx      = LEFT_W;
    int rw      = cols - rx;
    int bot     = rows - STATUS_H;
    int panel_h = bot - HDR_H;
    int out_h   = panel_h * 2 / 3;
    int stats_y = HDR_H + out_h;
    if (out_h < 3) out_h = 3;
    if (stats_y > bot - 3) stats_y = bot - 3;
    if (stats_y < HDR_H + 2) stats_y = HDR_H + 2;

    /* outputs sub-panel */
    int y = HDR_H;
    attron(A_UNDERLINE);
    mvprintw(y, rx + 1, "Last NN Outputs (%d)", s->num_outputs);
    attroff(A_UNDERLINE);
    y++;

    for (int i = 0; i < s->num_outputs && y < stats_y; i++, y++) {
        const OutputMapping *found = NULL;
        for (int j = 0; j < s->n_mappings; j++) {
            if (s->mappings[j].enabled && s->mappings[j].output_idx == i) {
                found = &s->mappings[j];
                break;
            }
        }
        move(y, rx + 1);
        if (found) {
            printw("[%2d] %10.4f  -> CAN %3d  %s",
                   i, (double)s->output_vector[i],
                   (int)found->can_id,
                   found->motor_type == MOTOR_MIT ? "MIT" : "Srv");
        } else {
            attron(A_DIM);
            printw("[%2d] %10.4f  (unmapped)", i, (double)s->output_vector[i]);
            attroff(A_DIM);
        }
        clrtoeol();
    }
    for (; y < stats_y; y++) { move(y, rx); clrtoeol(); }

    /* horizontal separator */
    move(stats_y, rx);
    for (int i = 0; i < rw; i++) addch(ACS_HLINE);
    mvaddch(stats_y, rx - 1, ACS_LTEE);

    /* stats sub-panel */
    y = stats_y + 1;
    move(y, rx + 1);
    if (s->serial_ok) attron(COLOR_PAIR(2));
    else              attron(COLOR_PAIR(1));
    printw("Serial: %-12s", s->serial_ok ? "CONNECTED" : "DISCONNECTED");
    if (s->serial_ok) attroff(COLOR_PAIR(2));
    else              attroff(COLOR_PAIR(1));
    printw("   Frames sent: %ld", s->frames_sent);
    if (s->infer_hz > 0.0)
        printw("   %.1f Hz (%.2f ms)", s->infer_hz, s->last_infer_ms);
    clrtoeol();
    y++;

    if (y < bot) {
        move(y, rx + 1);
        printw("Mode: %-20s  %d in / %d out",
               s->op_mode == OP_ACTIVE ? "ACTIVE (send+recv)" : "PASSIVE (recv only)",
               s->num_inputs, s->num_outputs);
        clrtoeol();
        y++;
    }

    if (y < bot) {
        move(y, rx + 1);
        printw("MIT motors: ");
        if (s->motors_enabled) attron(COLOR_PAIR(2) | A_BOLD);
        else                   attron(COLOR_PAIR(1));
        printw("%s", s->motors_enabled ? "ENABLED " : "DISABLED");
        if (s->motors_enabled) attroff(COLOR_PAIR(2) | A_BOLD);
        else                   attroff(COLOR_PAIR(1));
        clrtoeol();
        y++;
    }

    for (; y < bot; y++) { move(y, rx); clrtoeol(); }
}

static void draw_status(const NNAppState *s, int rows, int cols)
{
    move(rows - STATUS_H, 0);
    for (int i = 0; i < cols; i++) addch(ACS_HLINE);

    move(rows - 1, 0);
    clrtoeol();
    attron(A_DIM);
    printw("  Tab:mode  Spc:send(active)  p:port  n:dims  i:inputs  "
           "a:add  e:edit  Del:del  k:MIT  m:motor  s:scan  c:CAN  u:up  d:down  q:quit");
    attroff(A_DIM);

    if (s->status_msg[0]) {
        int len = (int)strlen(s->status_msg);
        int x   = cols - len - 2;
        if (x < 0) x = 0;
        if (s->status_err) attron(COLOR_PAIR(1) | A_BOLD);
        else               attron(COLOR_PAIR(2));
        mvprintw(rows - 1, x, "%s", s->status_msg);
        if (s->status_err) attroff(COLOR_PAIR(1) | A_BOLD);
        else               attroff(COLOR_PAIR(2));
    }
}

/* ── main ───────────────────────────────────────────────────────────────── */

int main(void)
{
    NNAppState s;
    memset(&s, 0, sizeof(s));
    strncpy(s.serial_port, "/dev/ttyACM1", sizeof(s.serial_port) - 1);
    s.serial_baud = 115200;
    s.serial_fd   = -1;
    strncpy(s.can_iface, "can0", sizeof(s.can_iface) - 1);
    s.can_sock    = -1;
    s.can_bitrate = 1000000;
    s.num_inputs  = DEFAULT_NUM_INPUTS;
    s.num_outputs = DEFAULT_NUM_OUTPUTS;
    s.mit.kp      = 1.00f;
    s.mit.kd      = 0.90f;
    s.mit.t_ff    = 0.0f;
    s.op_mode     = OP_PIPE;

    s.input_vector  = (float *)calloc((size_t)MAX_INPUTS,  sizeof(float));
    s.output_vector = (float *)calloc((size_t)MAX_OUTPUTS, sizeof(float));
    if (!s.input_vector || !s.output_vector) {
        fprintf(stderr, "Out of memory\n");
        free(s.input_vector);
        free(s.output_vector);
        return 1;
    }

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);

    if (has_colors()) {
        start_color();
        init_pair(1, COLOR_RED,   COLOR_BLACK);
        init_pair(2, COLOR_GREEN, COLOR_BLACK);
    }

    timeout(5);

    s.pipe_server_fd = -1;
    s.pipe_client_fd = -1;

    /* Serial is always needed (both ACTIVE and PIPE modes talk to STM32) */
    s.serial_fd = serial_open(s.serial_port, s.serial_baud);
    s.serial_ok = (s.serial_fd >= 0);

    /* In PIPE mode also open the Unix socket server for Python */
    if (s.op_mode == OP_PIPE)
        s.pipe_server_fd = open_pipe_server(PIPE_SOCKET_PATH);

    s.can_sock = nn_can_open(s.can_iface);

    if (!s.serial_ok) {
        snprintf(s.status_msg, sizeof(s.status_msg),
                 "Serial %s not found — press [p] to configure", s.serial_port);
        s.status_err = 1;
    } else if (s.op_mode == OP_PIPE && s.pipe_server_fd < 0) {
        snprintf(s.status_msg, sizeof(s.status_msg),
                 "Cannot create pipe socket — check /tmp permissions");
        s.status_err = 1;
    } else if (s.op_mode == OP_PIPE) {
        snprintf(s.status_msg, sizeof(s.status_msg),
                 "Waiting for pipe client on %s", PIPE_SOCKET_PATH);
        s.status_err = 0;
    } else if (s.can_sock < 0) {
        snprintf(s.status_msg, sizeof(s.status_msg),
                 "CAN %s not open — press [c] or [u] to configure", s.can_iface);
        s.status_err = 1;
    }

    for (;;) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        erase();
        draw_header(&s, cols);
        draw_mapping_panel(&s, rows);
        draw_output_panel(&s, rows, cols);
        draw_status(&s, rows, cols);
        refresh();

        /* Accept incoming pipe client (non-blocking) */
        if (s.pipe_server_fd >= 0 && s.pipe_client_fd < 0) {
            int client = accept(s.pipe_server_fd, NULL, NULL);
            if (client >= 0) {
                s.pipe_client_fd = client;
                s.pipe_input_offset = 0;
                snprintf(s.status_msg, sizeof(s.status_msg), "Pipe client connected");
                s.status_err = 0;
            }
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': goto quit;

        case '\t':
            s.op_mode = (s.op_mode == OP_ACTIVE) ? OP_PIPE : OP_ACTIVE;
            s.pipe_input_offset = 0;
            if (s.op_mode == OP_PIPE) {
                /* Open pipe server; serial stays open for STM32 */
                if (s.pipe_client_fd >= 0) { close(s.pipe_client_fd); s.pipe_client_fd = -1; }
                if (s.pipe_server_fd >= 0) { close(s.pipe_server_fd); unlink(PIPE_SOCKET_PATH); }
                s.pipe_server_fd = open_pipe_server(PIPE_SOCKET_PATH);
                snprintf(s.status_msg, sizeof(s.status_msg),
                         "PIPE — waiting for client on %s", PIPE_SOCKET_PATH);
            } else {
                /* Close pipe; serial stays open for STM32 */
                if (s.pipe_client_fd >= 0) { close(s.pipe_client_fd); s.pipe_client_fd = -1; }
                if (s.pipe_server_fd >= 0) { close(s.pipe_server_fd); unlink(PIPE_SOCKET_PATH); s.pipe_server_fd = -1; }
                snprintf(s.status_msg, sizeof(s.status_msg), "ACTIVE");
            }
            s.status_err = 0;
            break;

        case ' ':
            if (s.op_mode == OP_ACTIVE) {
                if (s.serial_fd < 0) {
                    snprintf(s.status_msg, sizeof(s.status_msg),
                             "Serial not connected — press [p] to configure");
                    s.status_err = 1;
                } else {
                    int r = serial_exchange(&s);
                    if (r == 0) {
                        s.serial_ok = 1;
                        dispatch_can(&s);
                        snprintf(s.status_msg, sizeof(s.status_msg),
                                 "Sent input vector, got outputs");
                        s.status_err = 0;
                    } else {
                        s.serial_ok = 0;
                        snprintf(s.status_msg, sizeof(s.status_msg),
                                 "Serial timeout — check Nucleo connection");
                        s.status_err = 1;
                    }
                }
            }
            break;

        case 'p': case 'P':
            edit_serial_popup(&s);
            break;

        case 'n': case 'N':
            edit_dims_popup(&s);
            break;

        case 'i': case 'I':
            edit_inputs_popup(&s);
            break;

        case 'a': case 'A':
            edit_mapping_popup(&s, -1);
            break;

        case 'e': case 'E':
            if (s.n_mappings > 0)
                edit_mapping_popup(&s, s.selected_mapping);
            break;

        case KEY_DC:
            if (s.n_mappings > 0) {
                if (s.selected_mapping < s.n_mappings - 1) {
                    memmove(&s.mappings[s.selected_mapping],
                            &s.mappings[s.selected_mapping + 1],
                            (size_t)(s.n_mappings - s.selected_mapping - 1)
                                * sizeof(OutputMapping));
                }
                s.n_mappings--;
                if (s.selected_mapping >= s.n_mappings && s.selected_mapping > 0)
                    s.selected_mapping--;
                snprintf(s.status_msg, sizeof(s.status_msg), "Mapping deleted");
                s.status_err = 0;
            }
            break;

        case 'm': case 'M':
            send_mit_motor_mode(&s, s.motors_enabled ? 0 : 1);
            break;

        case 'k': case 'K':
            edit_mit_globals_popup(&s);
            break;

        case 's': case 'S':
            can_scan_popup(&s);
            break;

        case 'c': case 'C':
            edit_can_iface_popup(&s);
            break;

        case 'u': case 'U':
            can_iface_up_popup(&s);
            break;

        case 'd':
            if (s.can_sock >= 0) { close(s.can_sock); s.can_sock = -1; }
            {
                char cmd[64];
                snprintf(cmd, sizeof(cmd), "ip link set %s down", s.can_iface);
                nn_run_cmd(cmd);
                snprintf(s.status_msg, sizeof(s.status_msg),
                         "%s down", s.can_iface);
                s.status_err = 0;
            }
            break;

        case KEY_UP:
            if (s.selected_mapping > 0) s.selected_mapping--;
            break;

        case KEY_DOWN:
            if (s.selected_mapping < s.n_mappings - 1) s.selected_mapping++;
            break;

        default: break;
        }

        /* ── PIPE I/O + CAN dispatch ── */
        if (s.op_mode == OP_PIPE && s.pipe_client_fd >= 0 && s.serial_fd >= 0) {
            int r = pipe_io(&s);
            if (r < 0) {
                close(s.pipe_client_fd);
                s.pipe_client_fd = -1;
                s.pipe_input_offset = 0;
                snprintf(s.status_msg, sizeof(s.status_msg),
                         "Pipe disconnected — waiting for new client");
                s.status_err = 1;
            } else if (r == 1) {
                s.serial_ok = 1;
            }
        }
    }

quit:
    if (s.serial_fd >= 0) close(s.serial_fd);
    if (s.pipe_client_fd >= 0) close(s.pipe_client_fd);
    if (s.pipe_server_fd >= 0) { close(s.pipe_server_fd); unlink(PIPE_SOCKET_PATH); }
    if (s.can_sock  >= 0) close(s.can_sock);
    free(s.input_vector);
    free(s.output_vector);
    endwin();
    return 0;
}
