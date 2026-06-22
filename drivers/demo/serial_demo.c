/*
 * serial_demo.c — ncurses TUI for CubeMars AK motor via R-Link serial port
 *
 * Wraps the motor drive's built-in text menu in a navigable ncurses interface.
 * Connect the R-Link USB adapter (/dev/ttyACM0), select the port with [p],
 * and navigate the motor's menu with arrow keys.
 *
 * Serial: 921600 baud, 8N1 (motor drive firmware default).
 *
 * Wire protocol (VESC-style binary framing, section 5.3.1 of CubeMars PDF):
 *   TX:  0x02 [length] [0x14=COMM_TERMINAL_CMD] [cmd_string] [crc_hi] [crc_lo] 0x03
 *   RX:  0x02 [length] [0x15=COMM_PRINT]        [text...]    [crc_hi] [crc_lo] 0x03
 *   length = 1 (packet_id) + len(data)
 *   CRC  = CRC16-CCITT (poly=0x1021, init=0) over payload bytes (packet_id + data)
 *
 * Motor menu commands (sent as COMM_TERMINAL_CMD strings):
 *   run        Motor Mode (MIT)     calibrate  Calibrate Encoder
 *   setup      Setup the motor      encoder    Show encoder value now
 *   origin     Set Zero Position    exit       Exit to Menu
 *
 * Setup sub-menu: prefix + value  (e.g. "b1000" sets bandwidth to 1000 Hz)
 *   b  Bandwidth (Hz)   i  CAN ID        m  CAN Master ID
 *   l  Curr Limit (A)   f  FW Limit (A)  t  CAN Timeout
 *
 * Layout:
 *   Header bar  : port | baud | connection status
 *   Left panel  : navigable menu (main or setup)
 *   Right panel : scrolling serial output log
 *   Status bar  : key hints + status messages
 *
 * Keys:
 *   Up/Down   navigate menu items
 *   Enter     select item / edit parameter
 *   p         edit serial port path
 *   Esc       send "exit" to motor / go back to main menu
 *   q         quit
 */

#include <curses.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ── layout constants ───────────────────────────────────────────────────── */

#define LEFT_W    26   /* left panel width including right border */
#define HDR_H      2   /* header rows */
#define STATUS_H   2   /* status bar rows at bottom */
#define LOG_LINES  500 /* ring-buffer line depth */
#define LOG_LINE_W 256 /* max chars per log line */
#define PORT_MAX    64

/* ── VESC framing constants ─────────────────────────────────────────────── */

#define COMM_TERMINAL_CMD 0x14
#define COMM_PRINT        0x15
#define FRAME_START       0x02
#define FRAME_END         0x03

/* ── menu tables ────────────────────────────────────────────────────────── */

typedef enum { SCREEN_MAIN, SCREEN_SETUP, SCREEN_MOTOR } ScreenState;

typedef struct {
    const char *label;
    const char *cmd;   /* string sent via COMM_TERMINAL_CMD frame */
    ScreenState next;  /* screen to switch to */
} MenuItem;

static const MenuItem MAIN_MENU[] = {
    { "Motor Mode (MIT)",  "run",       SCREEN_MOTOR },
    { "Calibrate Encoder", "calibrate", SCREEN_MAIN  },
    { "Setup",             "setup",     SCREEN_SETUP },
    { "Display Encoder",   "encoder",   SCREEN_MAIN  },
    { "Set Zero Position", "origin",    SCREEN_MAIN  },
};
#define N_MAIN (int)(sizeof(MAIN_MENU) / sizeof(MAIN_MENU[0]))

typedef struct {
    const char *label;
    const char *prefix; /* prefix string for the motor command */
    int         min_val;
    int         max_val;
} SetupParam;

static const SetupParam SETUP_PARAMS[] = {
    { "Bandwidth (Hz)",    "set_band_width",        100,    2000 },
    { "CAN ID",            "set_can_id",              0,     127 },
    { "CAN Master ID",     "set_master_id",           0,     127 },
    { "Current Limit (A)", "set_current_limit",       0,      60 },
    { "FW Curr Limit (A)", "set_fw_current_limit",    0,      33 },
    { "CAN Timeout",       "set_can_time_out",        0,  100000 },
};
#define N_SETUP (int)(sizeof(SETUP_PARAMS) / sizeof(SETUP_PARAMS[0]))

/* ── frame receiver state machine ───────────────────────────────────────── */

typedef enum {
    FRME_IDLE,    /* waiting for 0x02 */
    FRME_LEN,     /* next byte is payload length */
    FRME_PAYLOAD, /* reading payload bytes */
    FRME_CRC1,    /* first CRC byte */
    FRME_CRC2,    /* second CRC byte */
    FRME_END,     /* expecting 0x03 */
} FrameRecvState;

/* ── app state ──────────────────────────────────────────────────────────── */

typedef struct {
    char        port[PORT_MAX];
    int         fd;
    ScreenState screen;
    int         selected; /* index in current screen's menu */

    /* serial log ring buffer */
    char log[LOG_LINES][LOG_LINE_W];
    int  log_head;  /* index of oldest line */
    int  log_count; /* number of valid lines */
    int  log_col;   /* current column in newest (current) line */

    /* frame receiver */
    FrameRecvState frm_state;
    uint8_t        frm_len;     /* expected payload length */
    uint8_t        frm_buf[258];/* payload bytes */
    int            frm_pos;     /* bytes collected so far */
    uint8_t        frm_crc1;    /* received CRC high byte */

    char status_msg[128];
    int  status_err;
} AppState;

/* ── CRC16-CCITT (poly=0x1021, init=0) ──────────────────────────────────── */

static uint16_t crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000u)
                crc = (uint16_t)((crc << 1u) ^ 0x1021u);
            else
                crc = (uint16_t)(crc << 1u);
        }
    }
    return crc;
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
    default:     return B921600;
    }
}

static int serial_open(const char *port)
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
    speed_t spd = baud_to_speed(921600);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return -1; }
    return fd;
}

/*
 * Build and send a VESC-framed COMM_TERMINAL_CMD packet.
 * Frame: 0x02 [len] [0x14] [cmd_bytes] [crc_hi] [crc_lo] 0x03
 */
static void send_frame_cmd(int fd, const char *cmd)
{
    if (fd < 0) return;

    int  cmdlen  = (int)strlen(cmd);
    int  paylen  = 1 + cmdlen; /* packet_id + cmd */
    /* max cmd length that fits in a single-byte length field */
    if (paylen > 255) return;

    uint8_t payload[257];
    payload[0] = COMM_TERMINAL_CMD;
    memcpy(payload + 1, cmd, (size_t)cmdlen);

    uint16_t crc  = crc16(payload, paylen);
    uint8_t  crc_hi = (uint8_t)(crc >> 8);
    uint8_t  crc_lo = (uint8_t)(crc & 0xFF);

    /* total frame size: start(1) + len(1) + payload + crc(2) + end(1) */
    uint8_t frame[261];
    int     fi = 0;
    frame[fi++] = FRAME_START;
    frame[fi++] = (uint8_t)paylen;
    for (int i = 0; i < paylen; i++)
        frame[fi++] = payload[i];
    frame[fi++] = crc_hi;
    frame[fi++] = crc_lo;
    frame[fi++] = FRAME_END;

    ssize_t r = write(fd, frame, (size_t)fi);
    (void)r;
}

/* ── log helpers ────────────────────────────────────────────────────────── */

static int log_newest_idx(const AppState *s)
{
    if (s->log_count == 0) return 0;
    return (s->log_head + s->log_count - 1) % LOG_LINES;
}

static void log_append_byte(AppState *s, char c)
{
    /* Ignore bare CR */
    if (c == '\r') return;

    /* Ensure at least one line exists */
    if (s->log_count == 0) {
        s->log_count   = 1;
        s->log[0][0]   = '\0';
        s->log_col     = 0;
        s->log_head    = 0;
    }

    if (c == '\n') {
        if (s->log_count < LOG_LINES) {
            s->log_count++;
        } else {
            s->log_head = (s->log_head + 1) % LOG_LINES;
        }
        int ni = log_newest_idx(s);
        s->log[ni][0] = '\0';
        s->log_col    = 0;
        return;
    }

    int ni = log_newest_idx(s);
    if (s->log_col < LOG_LINE_W - 1) {
        s->log[ni][s->log_col]     = c;
        s->log[ni][s->log_col + 1] = '\0';
        s->log_col++;
    }
}

/* Feed a COMM_PRINT payload (text) into the log. */
static void log_append_print_payload(AppState *s, const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++)
        log_append_byte(s, (char)data[i]);
}

/*
 * Feed one received byte through the frame state machine.
 * Decoded COMM_PRINT frames have their text payload appended to the log.
 * Unknown/raw bytes that arrive outside a frame are also logged directly.
 */
static void feed_rx_byte(AppState *s, uint8_t b)
{
    switch (s->frm_state) {

    case FRME_IDLE:
        if (b == FRAME_START) {
            s->frm_state = FRME_LEN;
        } else {
            /* Raw byte outside framing — log it directly */
            log_append_byte(s, (char)b);
        }
        break;

    case FRME_LEN:
        s->frm_len   = b;
        s->frm_pos   = 0;
        s->frm_state = (b > 0) ? FRME_PAYLOAD : FRME_CRC1;
        break;

    case FRME_PAYLOAD:
        if (s->frm_pos < (int)sizeof(s->frm_buf))
            s->frm_buf[s->frm_pos] = b;
        s->frm_pos++;
        if (s->frm_pos >= s->frm_len)
            s->frm_state = FRME_CRC1;
        break;

    case FRME_CRC1:
        s->frm_crc1  = b;
        s->frm_state = FRME_CRC2;
        break;

    case FRME_CRC2: {
        uint16_t rx_crc = ((uint16_t)s->frm_crc1 << 8) | b;
        int      plen   = (s->frm_pos < (int)sizeof(s->frm_buf))
                          ? s->frm_pos : (int)sizeof(s->frm_buf);
        uint16_t calc   = crc16(s->frm_buf, plen);
        s->frm_state    = FRME_END;
        if (rx_crc != calc) {
            /* CRC mismatch — discard, go back to idle */
            s->frm_state = FRME_IDLE;
        }
        break;
    }

    case FRME_END:
        s->frm_state = FRME_IDLE;
        if (b != FRAME_END) break;
        /* Valid frame — dispatch on packet_id */
        if (s->frm_pos > 0) {
            uint8_t pkt_id = s->frm_buf[0];
            if (pkt_id == COMM_PRINT && s->frm_pos > 1)
                log_append_print_payload(s, s->frm_buf + 1, s->frm_pos - 1);
        }
        break;
    }
}

/* Read whatever is available from the serial port and feed the state machine. */
static void poll_serial(AppState *s)
{
    if (s->fd < 0) return;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s->fd, &rfds);
    struct timeval tv = { 0, 0 };
    if (select(s->fd + 1, &rfds, NULL, NULL, &tv) <= 0) return;

    uint8_t buf[256];
    ssize_t n = read(s->fd, buf, sizeof(buf));
    if (n <= 0) return;
    for (ssize_t i = 0; i < n; i++)
        feed_rx_byte(s, buf[i]);
}

/* ── popups ─────────────────────────────────────────────────────────────── */

static void edit_port_popup(AppState *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int pop_w = 52;
    WINDOW *pop = newwin(5, pop_w, (rows - 5) / 2, (cols - pop_w) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " Serial Port (921600 8N1) ");

    char buf[PORT_MAX];
    strncpy(buf, s->port, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    for (;;) {
        wattron(pop, A_REVERSE);
        mvwprintw(pop, 2, 2, "  %-46s", buf);
        wattroff(pop, A_REVERSE);
        mvwprintw(pop, 3, 2, "[Enter] connect  [Esc] cancel");
        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) break;
        if (ch == '\n' || ch == KEY_ENTER) {
            strncpy(s->port, buf, sizeof(s->port) - 1);
            s->port[sizeof(s->port) - 1] = '\0';
            if (s->fd >= 0) { close(s->fd); s->fd = -1; }
            s->frm_state = FRME_IDLE;
            s->fd = serial_open(s->port);
            if (s->fd < 0) {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Cannot open %s: %s", s->port, strerror(errno));
                s->status_err = 1;
            } else {
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Connected to %s at 921600 baud", s->port);
                s->status_err = 0;
            }
            break;
        } else if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if (ch >= 32 && ch < 127) {
            size_t l = strlen(buf);
            if (l < sizeof(buf) - 1) { buf[l] = (char)ch; buf[l + 1] = '\0'; }
        }
    }

    delwin(pop);
    touchwin(stdscr);
    refresh();
}

/* Opens a value editor for a setup parameter and sends the framed command. */
static void edit_setup_param_popup(AppState *s, int idx)
{
    const SetupParam *p = &SETUP_PARAMS[idx];
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *pop = newwin(7, 48, (rows - 7) / 2, (cols - 48) / 2);
    keypad(pop, TRUE);
    box(pop, 0, 0);
    mvwprintw(pop, 0, 2, " %s ", p->label);

    char buf[24] = "";

    for (;;) {
        mvwprintw(pop, 2, 2, "Range: %d .. %d", p->min_val, p->max_val);
        mvwprintw(pop, 3, 2, "Value:");
        wattron(pop, A_REVERSE);
        mvwprintw(pop, 4, 2, "  %-42s", buf);
        wattroff(pop, A_REVERSE);
        mvwprintw(pop, 5, 2, "[Enter] send  [Esc] cancel");
        wrefresh(pop);

        int ch = wgetch(pop);
        if (ch == 27) break;
        if (ch == '\n' || ch == KEY_ENTER) {
            if (buf[0] != '\0') {
                /* Send: prefix + value as a framed COMM_TERMINAL_CMD */
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "%s %s", p->prefix, buf);
                send_frame_cmd(s->fd, cmd);
                snprintf(s->status_msg, sizeof(s->status_msg),
                         "Sent: %s", cmd);
                s->status_err = 0;
            }
            break;
        } else if (ch == KEY_BACKSPACE || ch == 127) {
            size_t l = strlen(buf);
            if (l > 0) buf[l - 1] = '\0';
        } else if ((ch >= '0' && ch <= '9') || ch == '-') {
            size_t l = strlen(buf);
            if (l < sizeof(buf) - 1) { buf[l] = (char)ch; buf[l + 1] = '\0'; }
        }
    }

    delwin(pop);
    touchwin(stdscr);
    refresh();
}

/* ── drawing ────────────────────────────────────────────────────────────── */

static void draw_header(const AppState *s, int cols)
{
    const char *conn = (s->fd >= 0) ? "CONNECTED" : "DISCONNECTED";

    attron(A_BOLD | A_REVERSE);
    move(0, 0);
    for (int i = 0; i < cols; i++) addch(' ');
    mvprintw(0, 1, "AK Motor Serial Setup  |  %-20s  |  %s  921600 8N1",
             s->port[0] ? s->port : "(no port)", conn);
    attroff(A_BOLD | A_REVERSE);

    move(1, 0);
    for (int i = 0; i < cols; i++) addch(ACS_HLINE);
    mvaddch(1, LEFT_W - 1, ACS_TTEE);
}

static void draw_left_panel(const AppState *s, int rows)
{
    int bot = rows - STATUS_H;

    for (int r = HDR_H; r < bot; r++) mvaddch(r, LEFT_W - 1, ACS_VLINE);
    mvaddch(bot, LEFT_W - 1, ACS_BTEE);

    int y = HDR_H;

    if (s->screen == SCREEN_MAIN) {
        attron(A_UNDERLINE);
        mvprintw(y, 1, "MAIN MENU");
        attroff(A_UNDERLINE);
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
        y++;
        for (int i = 0; i < N_MAIN && y < bot; i++, y++) {
            move(y, 0);
            clrtoeol();
            if (i == s->selected)
                attron(A_REVERSE | A_BOLD);
            mvprintw(y, 1, " %-23s", MAIN_MENU[i].label);
            if (i == s->selected)
                attroff(A_REVERSE | A_BOLD);
            mvaddch(y, LEFT_W - 1, ACS_VLINE);
        }

    } else if (s->screen == SCREEN_SETUP) {
        attron(A_UNDERLINE);
        mvprintw(y, 1, "SETUP");
        attroff(A_UNDERLINE);
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
        y++;
        for (int i = 0; i < N_SETUP && y < bot; i++, y++) {
            move(y, 0);
            clrtoeol();
            if (i == s->selected)
                attron(A_REVERSE | A_BOLD);
            mvprintw(y, 1, " %-23s", SETUP_PARAMS[i].label);
            if (i == s->selected)
                attroff(A_REVERSE | A_BOLD);
            mvaddch(y, LEFT_W - 1, ACS_VLINE);
        }
        if (y < bot) {
            move(y, 0); clrtoeol();
            attron(A_DIM);
            mvprintw(y, 1, " [Esc] Back to main");
            attroff(A_DIM);
            mvaddch(y, LEFT_W - 1, ACS_VLINE);
            y++;
        }

    } else { /* SCREEN_MOTOR */
        attron(A_BOLD);
        mvprintw(y, 1, "MOTOR MODE ACTIVE");
        attroff(A_BOLD);
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
        y++;
        const char *lines[] = {
            "",
            " Motor is listening",
            " for CAN commands.",
            "",
            " [Esc] Exit motor",
            "       mode",
        };
        for (size_t i = 0; i < sizeof(lines)/sizeof(lines[0]) && y < bot; i++, y++) {
            move(y, 0); clrtoeol();
            attron(A_DIM);
            mvprintw(y, 0, "%s", lines[i]);
            attroff(A_DIM);
            mvaddch(y, LEFT_W - 1, ACS_VLINE);
        }
    }

    for (; y < bot; y++) {
        move(y, 0);
        clrtoeol();
        mvaddch(y, LEFT_W - 1, ACS_VLINE);
    }
}

static void draw_right_panel(const AppState *s, int rows, int cols)
{
    int rx  = LEFT_W;
    int bot = rows - STATUS_H;
    int rw  = cols - rx;
    int h   = bot - HDR_H;

    int y = HDR_H;

    attron(A_UNDERLINE);
    mvprintw(y, rx + 1, "Serial Output");
    attroff(A_UNDERLINE);
    move(y, rx + 14);
    clrtoeol();
    y++;

    int avail = h - 1;
    if (avail < 1) avail = 1;

    int nshow = (s->log_count < avail) ? s->log_count : avail;
    int start = (s->log_count <= avail)
                    ? s->log_head
                    : (s->log_head + s->log_count - avail) % LOG_LINES;

    int maxw = rw - 2;
    if (maxw < 0) maxw = 0;
    if (maxw >= LOG_LINE_W) maxw = LOG_LINE_W - 1;

    for (int i = 0; i < nshow && y < bot; i++, y++) {
        int idx = (start + i) % LOG_LINES;
        move(y, rx + 1);
        char clipped[LOG_LINE_W];
        strncpy(clipped, s->log[idx], (size_t)maxw);
        clipped[maxw] = '\0';
        printw("%s", clipped);
        clrtoeol();
    }
    for (; y < bot; y++) { move(y, rx + 1); clrtoeol(); }
}

static void draw_status(const AppState *s, int rows, int cols)
{
    move(rows - STATUS_H, 0);
    for (int i = 0; i < cols; i++) addch(ACS_HLINE);

    move(rows - 1, 0);
    clrtoeol();
    attron(A_DIM);
    if (s->screen == SCREEN_MAIN)
        printw("  [Arrows] Nav  [Enter] Select  [p] Port  [Esc] Send exit  [q] Quit");
    else if (s->screen == SCREEN_SETUP)
        printw("  [Arrows] Nav  [Enter] Edit value  [Esc] Back  [q] Quit");
    else
        printw("  [Esc] Exit motor mode  [q] Quit");
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
    strncpy(s.port, "/dev/ttyACM0", sizeof(s.port) - 1);
    s.fd        = -1;
    s.frm_state = FRME_IDLE;

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

    timeout(50); /* non-blocking getch, ~20 Hz redraw */

    s.fd = serial_open(s.port);
    if (s.fd < 0) {
        snprintf(s.status_msg, sizeof(s.status_msg),
                 "Cannot open %s — press [p] to set port", s.port);
        s.status_err = 1;
    } else {
        snprintf(s.status_msg, sizeof(s.status_msg),
                 "Connected to %s at 921600 baud", s.port);
        s.status_err = 0;
    }

    for (;;) {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);

        erase();
        draw_header(&s, cols);
        draw_left_panel(&s, rows);
        draw_right_panel(&s, rows, cols);
        draw_status(&s, rows, cols);
        refresh();

        int ch = getch();

        switch (ch) {
        case 'q': case 'Q':
            goto quit;

        case 27: /* ESC */
            send_frame_cmd(s.fd, "exit");
            if (s.screen == SCREEN_SETUP || s.screen == SCREEN_MOTOR) {
                s.screen   = SCREEN_MAIN;
                s.selected = 0;
                snprintf(s.status_msg, sizeof(s.status_msg),
                         "Sent \"exit\" — returned to main menu");
            } else {
                snprintf(s.status_msg, sizeof(s.status_msg), "Sent \"exit\"");
            }
            s.status_err = 0;
            break;

        case KEY_UP:
            if (s.screen == SCREEN_MAIN && s.selected > 0)
                s.selected--;
            else if (s.screen == SCREEN_SETUP && s.selected > 0)
                s.selected--;
            break;

        case KEY_DOWN:
            if (s.screen == SCREEN_MAIN && s.selected < N_MAIN - 1)
                s.selected++;
            else if (s.screen == SCREEN_SETUP && s.selected < N_SETUP - 1)
                s.selected++;
            break;

        case '\n': case KEY_ENTER:
            if (s.screen == SCREEN_MAIN) {
                const MenuItem *item = &MAIN_MENU[s.selected];
                send_frame_cmd(s.fd, item->cmd);
                s.screen   = item->next;
                s.selected = 0;
                snprintf(s.status_msg, sizeof(s.status_msg),
                         "Sent \"%s\" (%s)", item->cmd, item->label);
                s.status_err = 0;
            } else if (s.screen == SCREEN_SETUP) {
                edit_setup_param_popup(&s, s.selected);
            }
            break;

        case 'p': case 'P':
            edit_port_popup(&s);
            break;

        default:
            break;
        }

        poll_serial(&s);
    }

quit:
    if (s.fd >= 0) close(s.fd);
    endwin();
    return 0;
}
