 /*
 * demo.c — ncurses TUI for CubeMars AK motor drivers over SocketCAN
 *
 * Layout:
 *   Header bar   : interface name | link state | motor ID
 *   Left panel   : command list (arrow-key navigation)
 *   Right-top    : parameter display for the selected command
 *   Right-mid    : last transmitted CAN frame bytes
 *   Right-bottom : last received feedback
 *   Status bar   : key hints + error/status messages
 *
 * Keys:
 *   Up/Down   navigate commands
 *   Enter     open parameter editor, then send
 *   i         edit CAN interface name
 *   u         bring interface up (prompts for bitrate)
 *   d         bring interface down
 *   D         live CAN dump popup
 *   +/-       increment/decrement motor ID
 *   q         quit
 */

#include <curses.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ak_mit.h"
#include "ak_servo.h"

/* ── layout constants ───────────────────────────────────────────────────── */

#define LEFT_W      22   /* width of command panel (including the right border) */
#define HDR_H        2   /* header rows */
#define STATUS_H     2   /* status bar rows at bottom */
#define MAX_PARAMS   5   /* max parameters any command needs */
#define IFACE_MAX   16   /* max CAN interface name length */
#define DUMP_LINES  200  /* ring-buffer depth for can dump */

/* ── command table ──────────────────────────────────────────────────────── */

typedef enum { MODE_SERVO, MODE_MIT } CtrlMode;

typedef struct {
    const char *label;
    CtrlMode    mode;
    int         n_params;
    const char *pnames[MAX_PARAMS];
    const char *phints[MAX_PARAMS];
    double      pdefaults[MAX_PARAMS];
} CmdDef;

static const CmdDef CMDS[] = {
    { "Duty cycle",    MODE_SERVO, 1,
      {"duty"},         {"[-1.0 .. 1.0]"},              {0.0} },
    { "Current",       MODE_SERVO, 1,
      {"current_A"},    {"[-60 .. 60] A"},               {0.0} },
    { "Current brake", MODE_SERVO, 1,
      {"current_A"},    {"[-60 .. 60] A"},               {0.0} },
    { "RPM",           MODE_SERVO, 1,
      {"erpm"},         {"[-320000 .. 320000]"},         {0.0} },
    { "Position",      MODE_SERVO, 1,
      {"deg"},          {"[-3200 .. 3200] deg"},         {0.0} },
    { "Pos + Speed",   MODE_SERVO, 3,
      {"pos_deg",       "spd_erpm",         "acc_erpm_s"},
      {"[-3200..3200]", "[-327680..327680]","[0..327670]"},
      {0.0, 0.0, 0.0} },
    { "Set Origin",    MODE_SERVO, 1,
      {"mode"},         {"0=temp  1=perm"},              {0.0} },
    { "Enter MIT",     MODE_MIT,   0, {}, {}, {} },
    { "Exit MIT",      MODE_MIT,   0, {}, {}, {} },
    { "Set Zero",      MODE_MIT,   0, {}, {}, {} },
    { "MIT Command",   MODE_MIT,   5,
      {"p_des",  "v_des",    "kp",     "kd",   "t_ff"},
      {"+-12.5 rad","+-v_max","0..500","0..5","+-t_max N*m"},
      {0.0, 0.0, 1.0, 0.9, 0.0} },
};
#define N_CMDS (int)(sizeof(CMDS) / sizeof(CMDS[0]))
#define MIT_START_IDX 7

/* ── app state ──────────────────────────────────────────────────────────── */

typedef struct {
    char    iface[IFACE_MAX];
    int     can_sock;
    int     bitrate;        /* last used bitrate for ip link set … up */
    uint8_t motor_id;

    int    selected;
    double params[MAX_PARAMS];

    /* last TX */
    int      tx_valid;
    int      tx_is_eff;
    uint32_t tx_id;
    uint8_t  tx_data[8];
    uint8_t  tx_len;

    /* last servo feedback */
    int              servo_fb_valid;
    ServoCANFeedback servo_fb;

    /* last MIT feedback */
    int   mit_fb_valid;
    float mit_pos;
    float mit_vel;
    float mit_torque;

    char status_msg[128];
    int  status_err;
} AppState;

/* ── SocketCAN helpers ──────────────────────────────────────────────────── */

static int can_open(const char *iface)
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

static int can_send_frame(int sock, int is_eff, uint32_t id,
                          const uint8_t *data, uint8_t len)
{
    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id  = is_eff ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    f.can_dlc = len;
    memcpy(f.data, data, len);
    return (write(sock, &f, sizeof(f)) == (ssize_t)sizeof(f)) ? 0 : -1;
}

/* Returns 1 = frame received, 0 = timeout, -1 = error */
static int can_recv_frame(int sock, struct can_frame *out, int timeout_ms)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int r = select(sock + 1, &rfds, NULL, NULL, &tv);
    if (r <= 0) return r;
    ssize_t n = read(sock, out, sizeof(*out));
    return (n == (ssize_t)sizeof(*out)) ? 1 : -1;
}

/* Returns 1 = UP, 0 = DOWN/unknown */
static int iface_is_up(const char *iface)
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

/* ── interface up/down ──────────────────────────────────────────────────── */

/* Run a shell command, suppressing all output.
 * Returns the exit status (0 = success). */
static int run_cmd(const char *cmd)
{
    char buf[512];
    snprintf(buf, sizeof(buf), "%s >/dev/null 2>&1", cmd);
    return system(buf);
}

static void iface_up_popup(AppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(6, 46, (rows - 6) / 2, (cols - 46) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " Bring up %s ", s->iface);

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", s->bitrate);

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
            s->bitrate = br;
            delwin(pop); touchwin(stdscr); refresh();

            /* bring the interface up */
            char cmd[128];
            snprintf(cmd, sizeof(cmd),
                     "ip link set %s type can bitrate %d && ip link set %s up",
                     s->iface, s->bitrate, s->iface);
            int rc = run_cmd(cmd);
            if (rc != 0) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "ip link up failed (rc=%d) — need root?", rc);
                s->status_err = 1;
                return;
            }

            /* reconnect socket */
            if (s->can_sock >= 0) { close(s->can_sock); s->can_sock = -1; }
            s->can_sock = can_open(s->iface);
            if (s->can_sock < 0) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Up OK but socket open failed: %s", strerror(errno));
                s->status_err = 1;
            } else {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "%s up @ %d bps", s->iface, s->bitrate);
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

static void iface_down(AppState *s)
{
    if (s->can_sock >= 0) { close(s->can_sock); s->can_sock = -1; }

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "ip link set %s down", s->iface);
    int rc = run_cmd(cmd);
    if (rc != 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "ip link down failed (rc=%d) — need root?", rc);
        s->status_err = 1;
    } else {
        snprintf(s->status_msg, sizeof(s->status_msg), "%s down", s->iface);
        s->status_err = 0;
    }
}

/* ── CAN ID scan ────────────────────────────────────────────────────────── */

static void can_scan_popup(AppState *s)
{
    if (s->can_sock < 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "Not connected — bring interface up first");
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

        struct can_frame f;
        int r = can_recv_frame(s->can_sock, &f, 50);
        if (r == 1) {
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

/* ── live CAN dump ──────────────────────────────────────────────────────── */

typedef struct {
    char text[96];
} DumpLine;

static void can_dump_popup(AppState *s)
{
    if (s->can_sock < 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "Not connected — bring interface up first");
        s->status_err = 1;
        return;
    }

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int pop_h = rows - 4;
    int pop_w = cols - 4;
    if (pop_h < 6) pop_h = 6;
    if (pop_w < 40) pop_w = 40;

    WINDOW *pop = newwin(pop_h, pop_w, 2, 2);
    keypad(pop, TRUE);
    nodelay(pop, TRUE);  /* non-blocking wgetch */
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " CAN Dump: %s ", s->iface);
    mvwprintw(pop, pop_h - 2, 2,
              " ID         DLC  Data                         [q/Esc] stop ");

    DumpLine *lines = calloc((size_t)DUMP_LINES, sizeof(DumpLine));
    if (!lines) { delwin(pop); touchwin(stdscr); refresh(); return; }

    int head    = 0;   /* index of next line to write */
    int count   = 0;   /* total frames received */
    int display = pop_h - 4;  /* visible rows */

    struct timeval t0;
    gettimeofday(&t0, NULL);

    for (;;) {
        int ch = wgetch(pop);
        if (ch == 'q' || ch == 'Q' || ch == 27) break;

        /* try to receive a frame (50 ms window) */
        struct can_frame f;
        int r = can_recv_frame(s->can_sock, &f, 50);
        if (r == 1) {
            struct timeval now;
            gettimeofday(&now, NULL);
            double ts = (double)(now.tv_sec  - t0.tv_sec) +
                        (double)(now.tv_usec - t0.tv_usec) * 1e-6;

            int is_eff = (f.can_id & CAN_EFF_FLAG) ? 1 : 0;
            uint32_t id = f.can_id & (is_eff ? CAN_EFF_MASK : CAN_SFF_MASK);

            char *p = lines[head].text;
            int written = snprintf(p, sizeof(lines[0].text),
                         "%8.3f  %s%08X  %d   ",
                         ts, is_eff ? "" : "   ", id, f.can_dlc);
            for (int i = 0; i < f.can_dlc && i < 8; i++)
                written += snprintf(p + written,
                                    sizeof(lines[0].text) - (size_t)written,
                                    "%02X ", f.data[i]);

            head = (head + 1) % DUMP_LINES;
            count++;

            /* redraw visible lines */
            int n_avail = count < DUMP_LINES ? count : DUMP_LINES;
            int n_show  = n_avail < display   ? n_avail : display;
            for (int row = 0; row < display; row++) {
                int row_y = 1 + row;
                if (row_y >= pop_h - 2) break;
                wmove(pop, row_y, 1);
                wclrtoeol(pop);
                mvwaddch(pop, row_y, pop_w - 1, ACS_VLINE);
                if (row >= display - n_show) {
                    int idx = (head - n_show + (display - row - 1)
                               + DUMP_LINES) % DUMP_LINES;
                    mvwprintw(pop, row_y, 2, "%-*s",
                              pop_w - 4, lines[idx].text);
                }
            }

            /* frame count in title */
            mvwprintw(pop, 0, 2, " CAN Dump: %s  [%d frames] ",
                      s->iface, count);
            box(pop, 0, 0);
            wrefresh(pop);
        }
    }

    free(lines);
    delwin(pop);
    touchwin(stdscr);
    refresh();

    snprintf(s->status_msg, sizeof(s->status_msg),
             "Dump stopped (%d frames seen)", count);
    s->status_err = 0;
}

/* ── motor ID input popup ───────────────────────────────────────────────── */

static void edit_motor_id_popup(AppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(5, 30, (rows - 5) / 2, (cols - 30) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " Motor ID ");

    char buf[4];
    snprintf(buf, sizeof(buf), "%d", s->motor_id);

    for (;;) {
        wattron(pop, A_REVERSE);
        mvwprintw(pop, 2, 2, "  %-24s", buf);
        wattroff(pop, A_REVERSE);
        mvwprintw(pop, 3, 2, "[Enter] set  [Esc] cancel");
        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) break;
        if (ch == '\n' || ch == KEY_ENTER) {
            int id = atoi(buf);
            if (id >= 1 && id <= 255)
                s->motor_id = (uint8_t)id;
            else {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Motor ID must be 1–255");
                s->status_err = 1;
            }
            break;
        } else if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if (ch >= '0' && ch <= '9') {
            size_t l = strlen(buf);
            if (l < sizeof(buf) - 1) { buf[l] = (char)ch; buf[l+1] = '\0'; }
        }
    }

    delwin(pop);
    touchwin(stdscr);
    refresh();
}

/* ── send selected command ──────────────────────────────────────────────── */

static void do_send(AppState *s)
{
    if (s->can_sock < 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "Not connected — bring interface up first");
        s->status_err = 1;
        return;
    }

    double *p       = s->params;
    int     is_eff  = 0;
    uint32_t can_id = 0;
    uint8_t  data[8]; memset(data, 0, sizeof(data));
    uint8_t  len    = 0;

    switch (s->selected) {
    case 0: { AKMotorServoMessage m = generate_duty_message(s->motor_id, (float)p[0]);
              is_eff=1; can_id=m.extended_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 1: { AKMotorServoMessage m = generate_current_message(s->motor_id, (float)p[0]);
              is_eff=1; can_id=m.extended_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 2: { AKMotorServoMessage m = generate_current_brake_message(s->motor_id, (float)p[0]);
              is_eff=1; can_id=m.extended_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 3: { AKMotorServoMessage m = generate_rpm_message(s->motor_id, (float)p[0]);
              is_eff=1; can_id=m.extended_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 4: { AKMotorServoMessage m = generate_position_message(s->motor_id, (float)p[0]);
              is_eff=1; can_id=m.extended_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 5: { AKMotorServoMessage m = generate_pos_spd_message(
                  s->motor_id,(float)p[0],(int32_t)p[1],(int32_t)p[2]);
              is_eff=1; can_id=m.extended_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 6: { AKMotorServoMessage m = generate_origin_message(s->motor_id,(uint8_t)(int)p[0]);
              is_eff=1; can_id=m.extended_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 7: { AKMotorMITMessage m = generate_mit_enter_message(s->motor_id);
              is_eff=0; can_id=m.standard_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 8: { AKMotorMITMessage m = generate_mit_exit_message(s->motor_id);
              is_eff=0; can_id=m.standard_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 9: { AKMotorMITMessage m = generate_mit_set_zero_message(s->motor_id);
              is_eff=0; can_id=m.standard_id; memcpy(data,m.data,m.len); len=m.len; break; }
    case 10:{ AKMotorMITMessage m = generate_mit_command_message(
                  s->motor_id,(float)p[0],(float)p[1],(float)p[2],(float)p[3],(float)p[4]);
              is_eff=0; can_id=m.standard_id; memcpy(data,m.data,m.len); len=m.len; break; }
    default: return;
    }

    s->tx_valid  = 1;
    s->tx_is_eff = is_eff;
    s->tx_id     = can_id;
    memcpy(s->tx_data, data, len);
    s->tx_len    = len;

    if (can_send_frame(s->can_sock, is_eff, can_id, data, len) < 0) {
        snprintf(s->status_msg, sizeof(s->status_msg),
                 "Send failed: %s", strerror(errno));
        s->status_err = 1;
        return;
    }

    snprintf(s->status_msg, sizeof(s->status_msg), "Sent '%s'", CMDS[s->selected].label);
    s->status_err = 0;
}

/* ── parameter editor popup ─────────────────────────────────────────────── */

static int edit_params_popup(AppState *s)
{
    const CmdDef *c = &CMDS[s->selected];
    if (c->n_params == 0) return 1;

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int pop_h = c->n_params * 3 + 4;
    int pop_w = 52;
    int pop_y = (rows - pop_h) / 2;
    int pop_x = (cols - pop_w) / 2;
    if (pop_y < 0) pop_y = 0;
    if (pop_x < 0) pop_x = 0;

    WINDOW *pop = newwin(pop_h, pop_w, pop_y, pop_x);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " %s ", c->label);

    char bufs[MAX_PARAMS][32];
    for (int i = 0; i < c->n_params; i++)
        snprintf(bufs[i], sizeof(bufs[i]), "%g", s->params[i]);

    int field = 0;
    int confirmed = 0;

    for (;;) {
        for (int i = 0; i < c->n_params; i++) {
            int fy = 1 + i * 3;
            mvwprintw(pop, fy,     2, "%-14s %s", c->pnames[i], c->phints[i]);
            if (i == field) wattron(pop, A_REVERSE);
            mvwprintw(pop, fy + 1, 2, "  %-46s", bufs[i]);
            if (i == field) wattroff(pop, A_REVERSE);
        }
        mvwprintw(pop, pop_h - 2, 2,
                  "[Tab/Arrows] field  [Enter] send  [Esc] cancel");
        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) { break; }
        if (ch == '\n' || ch == KEY_ENTER) {
            if (field == c->n_params - 1) { confirmed = 1; break; }
            field++;
        } else if (ch == '\t' || ch == KEY_DOWN) {
            field = (field + 1) % c->n_params;
        } else if (ch == KEY_UP) {
            field = (field - 1 + c->n_params) % c->n_params;
        } else if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(bufs[field]);
            if (l > 0) bufs[field][l - 1] = '\0';
        } else if ((ch >= '0' && ch <= '9') || ch == '-' ||
                   ch == '.' || ch == 'e' || ch == '+') {
            size_t l = strlen(bufs[field]);
            if (l < sizeof(bufs[0]) - 1) {
                bufs[field][l]     = (char)ch;
                bufs[field][l + 1] = '\0';
            }
        }
    }

    if (confirmed)
        for (int i = 0; i < c->n_params; i++)
            s->params[i] = atof(bufs[i]);

    delwin(pop);
    touchwin(stdscr);
    refresh();
    return confirmed;
}

/* ── interface name editor popup ────────────────────────────────────────── */

static void edit_iface_popup(AppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(5, 42, (rows - 5) / 2, (cols - 42) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " CAN interface ");

    char buf[IFACE_MAX];
    strncpy(buf, s->iface, sizeof(buf) - 1);
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
            strncpy(s->iface, buf, sizeof(s->iface) - 1);
            s->iface[sizeof(s->iface) - 1] = '\0';
            if (s->can_sock >= 0) { close(s->can_sock); s->can_sock = -1; }
            s->can_sock = can_open(s->iface);
            if (s->can_sock < 0) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Cannot open %s: %s", s->iface, strerror(errno));
                s->status_err = 1;
            } else {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Connected to %s", s->iface);
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

/* ── drawing ────────────────────────────────────────────────────────────── */

static void draw_header(const AppState *s, int cols)
{
    int up = iface_is_up(s->iface);

    attron(A_BOLD | A_REVERSE);
    move(0, 0);
    for (int i = 0; i < cols; i++) addch(' ');
    mvprintw(0, 1, "AK Motor Demo  |  %s [%s]  |  Motor ID: %d",
             s->iface, up ? "UP" : "DOWN", s->motor_id);
    attroff(A_BOLD | A_REVERSE);

    move(1, 0);
    for (int i = 0; i < cols; i++) addch(ACS_HLINE);
    mvaddch(1, LEFT_W - 1, ACS_TTEE);
}

static void draw_cmd_panel(const AppState *s, int rows)
{
    int bot = rows - STATUS_H;
    for (int r = HDR_H; r < bot; r++) mvaddch(r, LEFT_W - 1, ACS_VLINE);
    mvaddch(bot, LEFT_W - 1, ACS_BTEE);

    int y = HDR_H;
    for (int i = 0; i < N_CMDS && y < bot; i++) {
        if (i == MIT_START_IDX) {
            move(y, 0);
            attron(A_DIM);
            printw(" --- MIT Mode ------");
            attroff(A_DIM);
            mvaddch(y, LEFT_W - 1, ACS_VLINE);
            y++;
            if (y >= bot) break;
        }
        move(y, 0);
        clrtoeol();
        if (i == s->selected) {
            attron(A_REVERSE | A_BOLD);
            mvprintw(y, 1, " %-19s", CMDS[i].label);
            attroff(A_REVERSE | A_BOLD);
        } else {
            mvprintw(y, 1, " %-19s", CMDS[i].label);
        }
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
        y++;
    }
    for (; y < bot; y++) {
        move(y, 0); clrtoeol();
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
    }
}

static void draw_right_panel(const AppState *s, int rows, int cols)
{
    int rx      = LEFT_W;
    int rw      = cols - rx;
    int bot     = rows - STATUS_H;
    int panel_h = bot - HDR_H;
    int third   = panel_h / 3;
    if (third < 3) third = 3;

    /* ── Parameters ── */
    {
        int y = HDR_H;
        attron(A_UNDERLINE);
        mvprintw(y, rx + 1, "Parameters");
        attroff(A_UNDERLINE);
        y++;
        const CmdDef *c = &CMDS[s->selected];
        if (c->n_params == 0) {
            mvprintw(y, rx + 1, "(no parameters)");
            move(y, rx + 16); clrtoeol();
            y++;
        } else {
            for (int i = 0; i < c->n_params && y < HDR_H + third - 1; i++, y++) {
                move(y, rx + 1);
                printw("%-14s %-16s = %g", c->pnames[i], c->phints[i], s->params[i]);
                clrtoeol();
            }
        }
        for (; y < HDR_H + third; y++) { move(y, rx); clrtoeol(); }
        move(HDR_H + third, rx);
        for (int i = 0; i < rw; i++) addch(ACS_HLINE);
        mvaddch(HDR_H + third, rx - 1, ACS_LTEE);
    }

    /* ── Last TX Frame ── */
    {
        int y = HDR_H + third + 1;
        attron(A_UNDERLINE);
        mvprintw(y, rx + 1, "Last TX Frame");
        attroff(A_UNDERLINE);
        y++;
        if (s->tx_valid) {
            if (s->tx_is_eff)
                mvprintw(y, rx + 1, "ID: 0x%08X (EFF)  Len: %d", s->tx_id, s->tx_len);
            else
                mvprintw(y, rx + 1, "ID: 0x%03X (STD)  Len: %d", s->tx_id, s->tx_len);
            clrtoeol(); y++;
            move(y, rx + 1);
            for (int i = 0; i < s->tx_len; i++) printw("%02X ", s->tx_data[i]);
            clrtoeol();
        } else {
            mvprintw(y, rx + 1, "(none)"); clrtoeol(); y++;
            move(y, rx + 1); clrtoeol();
        }
        int end2 = HDR_H + third * 2;
        for (int row = y + 1; row < end2; row++) { move(row, rx); clrtoeol(); }
        move(end2, rx);
        for (int i = 0; i < rw; i++) addch(ACS_HLINE);
        mvaddch(end2, rx - 1, ACS_LTEE);
    }

    /* ── Motor Status ── */
    {
        int y = HDR_H + third * 2 + 1;
        attron(A_UNDERLINE);
        mvprintw(y, rx + 1, "Motor Status");
        attroff(A_UNDERLINE);
        y++;
        if (s->servo_fb_valid) {
            mvprintw(y, rx + 1, "Pos:  %8.2f deg    Speed: %.0f ERPM",
                     (double)s->servo_fb.position, (double)s->servo_fb.speed);
            clrtoeol(); y++;
            mvprintw(y, rx + 1, "Curr: %8.2f A      Temp: %d C  Err: %d",
                     (double)s->servo_fb.current,
                     s->servo_fb.temperature, s->servo_fb.error);
            clrtoeol(); y++;
        } else if (s->mit_fb_valid) {
            mvprintw(y, rx + 1, "Pos:    %.4f rad",    (double)s->mit_pos);
            clrtoeol(); y++;
            mvprintw(y, rx + 1, "Vel:    %.4f rad/s    Torque: %.4f N*m",
                     (double)s->mit_vel, (double)s->mit_torque);
            clrtoeol(); y++;
        } else {
            mvprintw(y, rx + 1, "(none)"); clrtoeol(); y++;
            move(y, rx + 1); clrtoeol(); y++;
        }
        for (; y < bot; y++) { move(y, rx); clrtoeol(); }
    }
}

static void draw_status(const AppState *s, int rows, int cols)
{
    move(rows - STATUS_H, 0);
    for (int i = 0; i < cols; i++) addch(ACS_HLINE);

    move(rows - 1, 0);
    clrtoeol();
    attron(A_DIM);
    printw("  [Arrows] Nav  [Enter] Send  [i] Iface  [u] Up  [d] Down  [D] Dump  [s] Scan  [m] ID  [q] Quit");
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
    AppState s;
    memset(&s, 0, sizeof(s));
    strncpy(s.iface, "can0", sizeof(s.iface) - 1);
    s.can_sock = -1;
    s.motor_id = 1;
    s.bitrate  = 1000000;

    for (int i = 0; i < CMDS[0].n_params; i++)
        s.params[i] = CMDS[0].pdefaults[i];

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

    timeout(50);  /* non-blocking getch: refresh feedback at ~20 Hz */

    s.can_sock = can_open(s.iface);
    if (s.can_sock < 0) {
        snprintf(s.status_msg, sizeof(s.status_msg),
                 "Cannot open %s — use [u] to bring it up", s.iface);
        s.status_err = 1;
    } else {
        snprintf(s.status_msg, sizeof(s.status_msg), "Connected to %s", s.iface);
        s.status_err = 0;
    }

    for (;;) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        erase();
        draw_header(&s, cols);
        draw_cmd_panel(&s, rows);
        draw_right_panel(&s, rows, cols);
        draw_status(&s, rows, cols);
        refresh();

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': goto quit;

        case KEY_UP:
            if (s.selected > 0) {
                s.selected--;
                for (int i = 0; i < CMDS[s.selected].n_params; i++)
                    s.params[i] = CMDS[s.selected].pdefaults[i];
            }
            break;

        case KEY_DOWN:
            if (s.selected < N_CMDS - 1) {
                s.selected++;
                for (int i = 0; i < CMDS[s.selected].n_params; i++)
                    s.params[i] = CMDS[s.selected].pdefaults[i];
            }
            break;

        case '\n': case KEY_ENTER:
            if (edit_params_popup(&s)) do_send(&s);
            break;

        case 'i': case 'I':
            edit_iface_popup(&s);
            break;

        case 'u': case 'U':
            iface_up_popup(&s);
            break;

        case 'd':
            iface_down(&s);
            break;

        case 'D':
            can_dump_popup(&s);
            break;

        case 's': case 'S':
            can_scan_popup(&s);
            break;

        case 'm': case 'M':
            edit_motor_id_popup(&s);
            break;

        default: break;
        }

        /* ── continuous feedback polling ── */
        if (s.can_sock >= 0) {
            struct can_frame rx;
            while (can_recv_frame(s.can_sock, &rx, 0) == 1) {
                int is_eff = (rx.can_id & CAN_EFF_FLAG) ? 1 : 0;
                uint32_t id = rx.can_id & (is_eff ? CAN_EFF_MASK : CAN_SFF_MASK);
                if (!is_eff && (uint8_t)id == s.motor_id) {
                    /* MIT feedback: standard frame, ID == motor_id */
                    decode_mit_fb(rx.data, &s.mit_pos, &s.mit_vel, &s.mit_torque);
                    s.mit_fb_valid   = 1;
                    s.servo_fb_valid = 0;
                } else if (is_eff && (uint8_t)(id & 0xFF) == s.motor_id) {
                    /* Servo feedback: extended frame, lower 8 bits == motor_id */
                    s.servo_fb       = decode_servo_can_feedback(rx.data);
                    s.servo_fb_valid = 1;
                    s.mit_fb_valid   = 0;
                }
            }
        }
    }

quit:
    if (s.can_sock >= 0) close(s.can_sock);
    endwin();
    return 0;
}
