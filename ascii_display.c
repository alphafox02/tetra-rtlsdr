/*
 * tetra-rtlsdr - ASCII spectrum display
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Inspired by retrogram-rtlsdr and retrogram-soapysdr by r4d10n,
 * and UHD rx_ascii_art_dft.
 *
 * Renders a live FFT spectrum on stderr using Unicode block characters
 * (U+2581..U+2588) and ANSI escape codes for cursor positioning and
 * color.  No ncurses dependency -- pure POSIX + VT100/xterm escapes.
 *
 * Single accent color (cyan) keeps it clean -- just shows signal
 * shape so you can see the spikes.
 *
 * Keyboard controls (read non-blocking from /dev/tty):
 *   h/?   - toggle help overlay
 *   q     - quit
 *   d/D   - dynamic range -10/+10 dB
 *   r/R   - reference level -5/+5 dB
 *   g/G   - gain down/up (returned to main.c)
 */

/* Needed for clock_gettime(CLOCK_MONOTONIC) with -std=c99 */
#if !defined(_POSIX_C_SOURCE) || _POSIX_C_SOURCE < 199309L
#undef  _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include "ascii_display.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

/* ---- Configuration ---- */

#define DEFAULT_REFRESH_HZ   5       /* target refresh rate */
#define DEFAULT_DYN_RANGE   80.0f    /* initial dynamic range (dB) */
#define DEFAULT_REF_LEVEL    0.0f    /* 0 = auto ref level */
#define GUTTER_COLS           9      /* left gutter: "-100 dB |" = 9 chars */
#define AXIS_ROWS             2      /* frequency axis + tick marks */
#define STATUS_ROWS           3      /* blank + status + help hint */
#define MIN_PLOT_COLS        20      /* minimum usable plot width */
#define MIN_PLOT_ROWS         4      /* minimum usable plot height */

/* Unicode block elements: 8 levels from lowest to full block.
 * Index 0 = empty (space), 1..8 = ▁▂▃▄▅▆▇█ */
static const char *block_chars[9] = {
    " ",
    "\xe2\x96\x81",   /* U+2581 LOWER ONE EIGHTH BLOCK   */
    "\xe2\x96\x82",   /* U+2582 LOWER ONE QUARTER BLOCK   */
    "\xe2\x96\x83",   /* U+2583 LOWER THREE EIGHTHS BLOCK */
    "\xe2\x96\x84",   /* U+2584 LOWER HALF BLOCK          */
    "\xe2\x96\x85",   /* U+2585 LOWER FIVE EIGHTHS BLOCK  */
    "\xe2\x96\x86",   /* U+2586 LOWER THREE QUARTERS BLOCK*/
    "\xe2\x96\x87",   /* U+2587 LOWER SEVEN EIGHTHS BLOCK */
    "\xe2\x96\x88",   /* U+2588 FULL BLOCK                */
};

/* ANSI escape codes */
#define COLOR_RESET   "\033[0m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_DIM     "\033[2m"
#define COLOR_REVERSE "\033[7m"

/* ---- Internal state ---- */

struct ascii_display {
    spectrum_t    *spectrum;
    double         center_freq;     /* Hz */
    double         sample_rate;     /* Hz */

    /* Terminal geometry */
    int            term_cols;
    int            term_rows;
    int            plot_cols;       /* columns available for spectrum bars */
    int            plot_rows;       /* rows available for spectrum bars */

    /* Timing / refresh control */
    struct timespec last_draw;
    long            interval_ns;    /* nanoseconds between redraws */

    /* Spectrum snapshot buffer */
    float           psd[SPECTRUM_NFFT];
    uint64_t        last_seq;

    /* Binned (averaged) PSD for display: one value per column */
    float          *binned;         /* [plot_cols] */

    /* Pre-allocated line buffer for fprintf output */
    char           *linebuf;
    size_t          linebuf_sz;

    /* User-adjustable display params */
    float           dyn_range;      /* dynamic range in dB */
    float           ref_level;      /* reference level (dB); 0 = auto */
    int             show_help;      /* 1 = help overlay visible */

    /* Keyboard input from /dev/tty */
    int             tty_fd;
    struct termios  orig_termios;

    /* Multi-channel status (optional) */
    channel_mgr_t  *chmgr;
};

/* ---- Helpers ---- */

/* Get terminal size via ioctl on stderr fd. Returns 0 on success. */
static int get_term_size(int *cols, int *rows) {
    struct winsize ws;
    if (ioctl(STDERR_FILENO, TIOCGWINSZ, &ws) < 0)
        return -1;
    *cols = ws.ws_col;
    *rows = ws.ws_row;
    return 0;
}

/* Monotonic clock helper (POSIX) */
static void clock_now(struct timespec *ts) {
    clock_gettime(CLOCK_MONOTONIC, ts);
}

static long timespec_diff_ns(const struct timespec *a,
                              const struct timespec *b) {
    return (long)(a->tv_sec - b->tv_sec) * 1000000000L
         + (long)(a->tv_nsec - b->tv_nsec);
}

/* Format frequency with appropriate unit (MHz with 3 decimal places) */
static int fmt_freq_mhz(char *buf, size_t sz, double hz) {
    return snprintf(buf, sz, "%.3f", hz / 1e6);
}

/* ---- Resampling: NFFT bins -> plot_cols bins (averaging) ---- */

static void resample_psd(const float *psd_in, int nfft,
                          float *psd_out, int ncols) {
    /* Map each output column to a range of input FFT bins and average */
    for (int c = 0; c < ncols; c++) {
        float bin_lo = (float)c       * (float)nfft / (float)ncols;
        float bin_hi = (float)(c + 1) * (float)nfft / (float)ncols;

        int i0 = (int)bin_lo;
        int i1 = (int)bin_hi;
        if (i1 >= nfft) i1 = nfft - 1;

        float sum = 0.0f;
        int   cnt = 0;
        for (int i = i0; i <= i1; i++) {
            sum += psd_in[i];
            cnt++;
        }
        psd_out[c] = (cnt > 0) ? sum / (float)cnt : -100.0f;
    }
}

/* ---- Keyboard input ---- */

static int tty_open_raw(struct termios *orig) {
    int fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
    if (fd < 0) return -1;

    if (tcgetattr(fd, orig) < 0) {
        close(fd);
        return -1;
    }

    struct termios raw = *orig;
    raw.c_lflag &= ~(unsigned)(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &raw);

    return fd;
}

static void tty_restore(int fd, const struct termios *orig) {
    if (fd < 0) return;
    tcsetattr(fd, TCSANOW, orig);
    close(fd);
}

/* Read one keypress non-blocking. Returns 0 if no key available. */
static char tty_read_key(int fd) {
    if (fd < 0) return 0;
    char ch = 0;
    if (read(fd, &ch, 1) == 1)
        return ch;
    return 0;
}

/* ---- Help overlay ---- */

static void draw_help_overlay(int term_cols) {
    /* Draw centered help box */
    const char *lines[] = {
        "  KEYBOARD CONTROLS  ",
        "",
        "  h / ?   toggle this help",
        "  q       quit",
        "  d / D   dyn range -/+ 10 dB",
        "  r / R   ref level -/+ 5 dB",
        "  a       auto ref level",
        "  g / G   gain -/+ 2 dB",
        "",
    };
    int nlines = (int)(sizeof(lines) / sizeof(lines[0]));

    /* Find max line width */
    int max_w = 0;
    for (int i = 0; i < nlines; i++) {
        int w = (int)strlen(lines[i]);
        if (w > max_w) max_w = w;
    }
    max_w += 4; /* padding */
    if (max_w > term_cols) max_w = term_cols;

    int left = (term_cols - max_w) / 2;
    if (left < 0) left = 0;

    /* Position overlay starting at row 3 */
    for (int i = 0; i < nlines; i++) {
        fprintf(stderr, "\033[%d;%dH", 3 + i, left + 1);
        fprintf(stderr, COLOR_REVERSE);
        int w = (int)strlen(lines[i]);
        int pad = max_w - w;
        int pad_l = pad / 2;
        int pad_r = pad - pad_l;
        for (int j = 0; j < pad_l; j++) fputc(' ', stderr);
        fputs(lines[i], stderr);
        for (int j = 0; j < pad_r; j++) fputc(' ', stderr);
        fprintf(stderr, COLOR_RESET);
    }
}

/* ---- Public API ---- */

ascii_display_t *ascii_display_create(spectrum_t *spectrum,
                                       double center_freq,
                                       double sample_rate) {
    if (!spectrum)
        return NULL;

    /* Only draw if stderr is a terminal */
    if (!isatty(STDERR_FILENO))
        return NULL;

    ascii_display_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->spectrum    = spectrum;
    d->center_freq = center_freq;
    d->sample_rate = sample_rate;
    d->last_seq    = 0;
    d->dyn_range   = DEFAULT_DYN_RANGE;
    d->ref_level   = DEFAULT_REF_LEVEL;  /* 0 = auto */
    d->show_help   = 0;
    d->tty_fd      = -1;

    /* Refresh interval */
    d->interval_ns = 1000000000L / DEFAULT_REFRESH_HZ;

    /* Initial terminal size */
    if (get_term_size(&d->term_cols, &d->term_rows) < 0) {
        d->term_cols = 80;
        d->term_rows = 24;
    }

    d->plot_cols = d->term_cols - GUTTER_COLS;
    d->plot_rows = d->term_rows - AXIS_ROWS - STATUS_ROWS - 1; /* -1: avoid scroll */

    if (d->plot_cols < MIN_PLOT_COLS)
        d->plot_cols = MIN_PLOT_COLS;
    if (d->plot_rows < MIN_PLOT_ROWS)
        d->plot_rows = MIN_PLOT_ROWS;

    d->binned = calloc((size_t)d->plot_cols, sizeof(float));
    if (!d->binned) {
        free(d);
        return NULL;
    }

    /* Line buffer: generous allocation for ANSI escapes + UTF-8 */
    d->linebuf_sz = (size_t)(d->term_cols * 16 + 256);
    d->linebuf = malloc(d->linebuf_sz);
    if (!d->linebuf) {
        free(d->binned);
        free(d);
        return NULL;
    }

    /* Open /dev/tty for non-blocking keyboard input */
    d->tty_fd = tty_open_raw(&d->orig_termios);

    /* Seed the clock so the first call draws immediately */
    clock_now(&d->last_draw);
    d->last_draw.tv_sec -= 2;   /* force first draw */

    /* Hide cursor, clear screen */
    fprintf(stderr, "\033[?25l\033[2J\033[H");

    return d;
}

ascii_cmd_t ascii_display_update(ascii_display_t *disp, float rssi_db,
                                  float fll_freq_hz, unsigned int sync_count) {
    if (!disp) return ASCII_CMD_NONE;

    ascii_cmd_t cmd = ASCII_CMD_NONE;

    /* --- Process keyboard input (always, even between draws) --- */
    {
        char ch;
        while ((ch = tty_read_key(disp->tty_fd)) != 0) {
            switch (ch) {
            case 'q': case 'Q': cmd = ASCII_CMD_QUIT;      break;
            case 'h': case '?': disp->show_help = !disp->show_help; break;
            case 'd': disp->dyn_range -= 10.0f;
                      if (disp->dyn_range < 20.0f) disp->dyn_range = 20.0f;
                      break;
            case 'D': disp->dyn_range += 10.0f;
                      if (disp->dyn_range > 120.0f) disp->dyn_range = 120.0f;
                      break;
            case 'r': disp->ref_level -= 5.0f; break;
            case 'R': disp->ref_level += 5.0f; break;
            case 'a': disp->ref_level = 0.0f;  break;  /* auto */
            case 'g': case '-': cmd = ASCII_CMD_GAIN_DOWN; break;
            case 'G': case '+': cmd = ASCII_CMD_GAIN_UP;   break;
            default: break;
            }
        }
    }

    /* --- Throttle drawing to configured refresh rate --- */
    struct timespec now;
    clock_now(&now);
    if (timespec_diff_ns(&now, &disp->last_draw) < disp->interval_ns)
        return cmd;
    disp->last_draw = now;

    /* --- Re-detect terminal size (may have been resized) --- */
    {
        int cols, rows;
        if (get_term_size(&cols, &rows) == 0) {
            if (cols != disp->term_cols || rows != disp->term_rows) {
                disp->term_cols = cols;
                disp->term_rows = rows;

                int new_pcols = cols - GUTTER_COLS;
                int new_prows = rows - AXIS_ROWS - STATUS_ROWS - 1;
                if (new_pcols < MIN_PLOT_COLS) new_pcols = MIN_PLOT_COLS;
                if (new_prows < MIN_PLOT_ROWS) new_prows = MIN_PLOT_ROWS;

                if (new_pcols != disp->plot_cols) {
                    disp->plot_cols = new_pcols;
                    free(disp->binned);
                    disp->binned = calloc((size_t)new_pcols, sizeof(float));
                    if (!disp->binned) return cmd;
                }
                disp->plot_rows = new_prows;

                /* Realloc line buffer */
                size_t new_sz = (size_t)(cols * 16 + 256);
                if (new_sz > disp->linebuf_sz) {
                    char *nb = realloc(disp->linebuf, new_sz);
                    if (nb) {
                        disp->linebuf = nb;
                        disp->linebuf_sz = new_sz;
                    }
                }

                /* Clear screen on resize */
                fprintf(stderr, "\033[2J");
            }
        }
    }

    /* --- Get latest PSD snapshot (non-blocking) --- */
    {
        if (pthread_mutex_trylock(&disp->spectrum->mutex) == 0) {
            if (disp->spectrum->seq > disp->last_seq) {
                memcpy(disp->psd,
                       disp->spectrum->psd[disp->spectrum->active],
                       SPECTRUM_NFFT * sizeof(float));
                disp->last_seq = disp->spectrum->seq;
            }
            pthread_mutex_unlock(&disp->spectrum->mutex);
        }
    }

    if (disp->last_seq == 0 || !disp->binned)
        return cmd;   /* no data yet or allocation failed */

    /* --- Bin the 1024-point PSD into plot_cols columns --- */
    resample_psd(disp->psd, SPECTRUM_NFFT,
                 disp->binned, disp->plot_cols);

    /* --- Compute display dB range --- */
    float db_ceil, db_floor;

    if (disp->ref_level != 0.0f) {
        /* Manual reference level */
        db_ceil  = disp->ref_level;
        db_floor = db_ceil - disp->dyn_range;
    } else {
        /* Auto-range: snap ceiling near peak */
        float peak = -999.0f;
        for (int c = 0; c < disp->plot_cols; c++) {
            if (disp->binned[c] > peak)
                peak = disp->binned[c];
        }
        if (peak > -200.0f) {
            db_ceil = ceilf(peak / 10.0f) * 10.0f;
            if (db_ceil < peak + 5.0f)
                db_ceil += 10.0f;
        } else {
            db_ceil = -10.0f;
        }
        db_floor = db_ceil - disp->dyn_range;
    }

    float db_range = db_ceil - db_floor;
    if (db_range < 10.0f) db_range = 10.0f;

    /* Total vertical resolution: plot_rows * 8 sub-levels */
    int total_levels = disp->plot_rows * 8;

    /* --- Move cursor to top-left and draw --- */
    fprintf(stderr, "\033[H");   /* cursor home */

    /* Draw spectrum rows (top = highest dB, bottom = lowest dB).
     * Single color (cyan) -- clean look, just shows signal shape. */
    for (int r = 0; r < disp->plot_rows; r++) {
        /* dB label for this row */
        float label_db = db_ceil - (float)r * db_range / (float)disp->plot_rows;
        fprintf(stderr, COLOR_DIM "%4.0f dB" COLOR_RESET " " COLOR_DIM "|" COLOR_RESET
                COLOR_CYAN, label_db);

        for (int c = 0; c < disp->plot_cols; c++) {
            float norm = (disp->binned[c] - db_floor) / db_range;
            int   fill = (int)(norm * (float)total_levels + 0.5f);
            if (fill < 0) fill = 0;
            if (fill > total_levels) fill = total_levels;

            int levels_below = (disp->plot_rows - 1 - r) * 8;
            int sub = fill - levels_below;
            if (sub < 0) sub = 0;
            if (sub > 8) sub = 8;

            fputs(block_chars[sub], stderr);
        }

        fprintf(stderr, COLOR_RESET "\033[K\n");
    }

    /* --- Frequency axis --- */

    /* Tick mark line */
    fprintf(stderr, COLOR_DIM);
    {
        int i;
        for (i = 0; i < GUTTER_COLS - 1; i++)
            fputc(' ', stderr);
        fputc('+', stderr);
    }

    {
        int tick_spacing = 12;
        if (disp->plot_cols < 60) tick_spacing = 8;
        if (disp->plot_cols < 40) tick_spacing = 6;

        for (int c = 0; c < disp->plot_cols; c++) {
            fputc((c % tick_spacing == 0) ? '+' : '-', stderr);
        }
    }
    fprintf(stderr, COLOR_RESET "\033[K\n");

    /* Frequency labels line */
    {
        double bw = disp->sample_rate;
        double f_start = disp->center_freq - bw / 2.0;

        memset(disp->linebuf, ' ', (size_t)disp->term_cols);

        int tick_spacing = 12;
        if (disp->plot_cols < 60) tick_spacing = 8;
        if (disp->plot_cols < 40) tick_spacing = 6;

        for (int c = 0; c <= disp->plot_cols; c += tick_spacing) {
            double f = f_start + (double)c * bw / (double)disp->plot_cols;
            char lbl[24];
            int  lbl_len = fmt_freq_mhz(lbl, sizeof(lbl), f);

            int pos = GUTTER_COLS + c - lbl_len / 2;
            if (pos < 0) pos = 0;
            if (pos + lbl_len > disp->term_cols)
                pos = disp->term_cols - lbl_len;

            int ok = 1;
            for (int j = pos; j < pos + lbl_len && j < disp->term_cols; j++) {
                if (disp->linebuf[j] != ' ') {
                    ok = 0;
                    break;
                }
            }
            if (ok) {
                for (int j = 0; j < lbl_len && pos + j < disp->term_cols; j++)
                    disp->linebuf[pos + j] = lbl[j];
            }
        }

        if (disp->term_cols > 0)
            disp->linebuf[disp->term_cols - 1] = '\0';
        else
            disp->linebuf[0] = '\0';

        int end = (int)strlen(disp->linebuf);
        while (end > 0 && disp->linebuf[end - 1] == ' ')
            end--;
        disp->linebuf[end] = '\0';

        fprintf(stderr, COLOR_DIM "%s" COLOR_RESET "\033[K\n", disp->linebuf);
    }

    /* --- Status line --- */
    fprintf(stderr, "\033[K\n");   /* blank separator line */

    if (disp->chmgr) {
        /* Multi-channel status: [N/M ch] per-channel freq+RSSI + scan */
        int nch = channel_mgr_n_channels(disp->chmgr);
        fprintf(stderr,
                "  " COLOR_BOLD "[%d/%d ch]" COLOR_RESET,
                nch, CHMGR_MAX_CHANNELS);

        for (int i = 0; i < nch && i < 4; i++) {
            uint32_t chf = channel_mgr_get_freq(disp->chmgr, i);
            float chr = channel_mgr_get_rssi(disp->chmgr, i);
            fprintf(stderr,
                    COLOR_DIM " | " COLOR_RESET
                    COLOR_CYAN "ch%d" COLOR_RESET " %.3f %+.0f",
                    i, chf / 1e6, chr);
        }

        int step, total;
        if (channel_mgr_scan_progress(disp->chmgr, &step, &total)) {
            fprintf(stderr,
                    COLOR_DIM " | " COLOR_RESET
                    COLOR_DIM "scan %d/%d" COLOR_RESET,
                    step, total);
        }

        fprintf(stderr, "\033[K\n");
    } else {
        fprintf(stderr,
                "  " COLOR_BOLD "FREQ" COLOR_RESET " %.3f MHz"
                COLOR_DIM "  |  " COLOR_RESET
                COLOR_BOLD "RSSI" COLOR_RESET " %.1f dB"
                COLOR_DIM "  |  " COLOR_RESET
                COLOR_BOLD "FLL" COLOR_RESET " %+.0f Hz"
                COLOR_DIM "  |  " COLOR_RESET
                COLOR_BOLD "SYNC" COLOR_RESET " %u"
                COLOR_DIM "  |  " COLOR_RESET
                COLOR_DIM "range" COLOR_RESET " %.0f dB"
                "\033[K\n",
                disp->center_freq / 1e6,
                rssi_db,
                fll_freq_hz,
                sync_count,
                disp->dyn_range);
    }

    /* Help hint or controls line */
    if (disp->show_help) {
        fprintf(stderr, COLOR_DIM
                "  q:quit  d/D:range  r/R:ref  a:auto  g/G:gain  h:hide"
                COLOR_RESET "\033[K");
    } else {
        fprintf(stderr, COLOR_DIM "  press h for help" COLOR_RESET "\033[K");
    }

    /* Clear any remaining lines below */
    fprintf(stderr, "\033[J");

    /* Draw help overlay on top if enabled */
    if (disp->show_help)
        draw_help_overlay(disp->term_cols);

    fflush(stderr);
    return cmd;
}

void ascii_display_set_channel_mgr(ascii_display_t *disp, channel_mgr_t *chmgr) {
    if (disp) disp->chmgr = chmgr;
}

void ascii_display_destroy(ascii_display_t *disp) {
    if (!disp) return;

    /* Restore terminal */
    tty_restore(disp->tty_fd, &disp->orig_termios);

    /* Show cursor again, reset colors, clear screen */
    fprintf(stderr, "\033[?25h" COLOR_RESET "\033[2J\033[H");
    fflush(stderr);

    free(disp->binned);
    free(disp->linebuf);
    free(disp);
}
