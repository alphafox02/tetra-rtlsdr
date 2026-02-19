/*
 * tetra-rtlsdr - multi-channel demodulation manager
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Manages multiple simultaneous TETRA demod channels from one RTL-SDR capture.
 *
 * The in-band scan is non-blocking: it runs incrementally inside
 * channel_mgr_process(), examining one 25 kHz step at a time while
 * all active channels keep demodulating in real time.
 */

#include "channel_mgr.h"
#include "dqpsk_demod.h"
#include "tetra_decode.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/wait.h>

/* Respawn limits: max attempts within a time window */
#define RESPAWN_MAX_ATTEMPTS  5
#define RESPAWN_WINDOW_SECS  60

/* SYNC training sequence (38 bits) — same as scanner.c */
static const uint8_t y_bits[38] = {
    1,1, 0,0, 0,0, 0,1, 1,0, 0,1, 1,1, 0,0,
    1,1, 1,0, 1,0, 0,1, 1,1, 0,0, 0,0, 0,1,
    1,0, 0,1, 1,1
};

/* Normal Training Sequence 1 — downlink Normal Burst (22 bits) */
static const uint8_t n_bits[22] = {
    1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1,
    1,1, 0,1, 0,0
};

/* In-band scan dwell: bits buffer for ~1.5s at 36 kbps */
#define SCAN_BITBUF_SIZE  (64 * 1024)

/* IQ read block (same as main.c) */
#define IQ_BLOCK_BYTES    51200u

typedef struct {
    uint32_t        freq;           /* target TETRA frequency */
    float           nco_offset;     /* Hz offset from RTL-SDR center */
    dqpsk_demod_t  *demod;
    char            fifo_path[64];  /* /tmp/tetra_ch<pid>_N */
    int             fifo_fd;        /* write end of FIFO (-1 if not open) */
    pid_t           rx_pid;         /* tetra-rx child process */
    int             rxid;           /* TETRA_HACK_RXID (1-based) */
    int             respawn_count;  /* respawns within current window */
    time_t          respawn_window; /* start of current respawn window */
    int             disabled;       /* 1 = respawn limit hit, channel dead */
} channel_t;

struct channel_mgr {
    channel_t       ch[CHMGR_MAX_CHANNELS];
    int             n_channels;
    uint32_t        center_freq;    /* RTL-SDR center frequency */
    uint32_t        sample_rate;
    uint32_t        channel_rate;
    char            tetra_rx_path[256];
    pid_t           our_pid;        /* for unique FIFO names */

    /* Background scan state */
    int             scan_running;       /* 1 = scan in progress */
    uint32_t        scan_freq;          /* current scan frequency */
    uint32_t        scan_lo;            /* scan range lower bound */
    uint32_t        scan_hi;            /* scan range upper bound */
    int             scan_step;          /* current step number */
    int             scan_n_steps;       /* total steps */
    int             scan_blocks;        /* IQ blocks at current freq */
    int             scan_blocks_needed; /* blocks per dwell period */
    int             scan_found;         /* total channels found */
    dqpsk_demod_t  *scan_demod;         /* temp demod for current step */
    uint8_t        *scan_bitbuf;        /* bit buffer for match detection */
};

/* ---- Helpers ---- */

static int freq_is_duplicate(channel_mgr_t *mgr, uint32_t freq) {
    for (int i = 0; i < mgr->n_channels; i++) {
        uint32_t diff = (mgr->ch[i].freq > freq)
                      ? mgr->ch[i].freq - freq
                      : freq - mgr->ch[i].freq;
        if (diff < 12500)
            return 1;
    }
    return 0;
}

static int freq_in_band(channel_mgr_t *mgr, uint32_t freq) {
    float offset = (float)freq - (float)mgr->center_freq;
    float half_bw = (float)mgr->sample_rate * 0.4f;
    return (offset > -half_bw && offset < half_bw);
}

/* Count TETRA training sequence matches in a bit buffer */
static int count_training_matches(const uint8_t *bits, size_t n) {
    int count = 0;

    /* SYNC (38 bits) */
    if (n >= 38) {
        size_t end = n - 38;
        for (size_t i = 0; i <= end; i++) {
            if (memcmp(bits + i, y_bits, 38) == 0) {
                count++;
                i += 37;
            }
        }
    }

    /* NTS1 (22 bits) */
    if (n >= 22) {
        size_t end = n - 22;
        for (size_t i = 0; i <= end; i++) {
            if (memcmp(bits + i, n_bits, 22) == 0) {
                count++;
                i += 21;
            }
        }
    }

    return count;
}

/*
 * Fork tetra-rx reading from fifo_path with given RXID.
 * Returns child PID, or -1 on error.
 */
static pid_t spawn_tetra_rx(const char *tetra_rx_path,
                              const char *fifo_path, int rxid) {
    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "[multi] fork() failed: %s\n", strerror(errno));
        return -1;
    }

    if (pid == 0) {
        /* child */
        char rxid_str[8];
        snprintf(rxid_str, sizeof(rxid_str), "%d", rxid);
        setenv("TETRA_HACK_RXID", rxid_str, 1);

        /* FIFO → stdin */
        int rfd = open(fifo_path, O_RDONLY);
        if (rfd < 0) _exit(1);
        dup2(rfd, STDIN_FILENO);
        close(rfd);

        /* Let tetra-rx stderr pass through to parent for debugging.
         * This may interleave with our own stderr output but is
         * important for diagnosing startup failures. */
        execl(tetra_rx_path, "tetra-rx", "-r", "-s", "/dev/stdin", NULL);
        _exit(1);
    }

    return pid;
}

/* Start a single channel: create FIFO, fork tetra-rx, create demod */
static int start_channel(channel_mgr_t *mgr, int idx) {
    channel_t *ch = &mgr->ch[idx];

    /* Create named FIFO with unique name (PID avoids collisions) */
    snprintf(ch->fifo_path, sizeof(ch->fifo_path),
             "/tmp/tetra_%d_ch%d", (int)mgr->our_pid, idx);
    unlink(ch->fifo_path);
    if (mkfifo(ch->fifo_path, 0600) < 0) {
        fprintf(stderr, "[multi] mkfifo(%s) failed: %s\n",
                ch->fifo_path, strerror(errno));
        return -1;
    }

    /* Fork tetra-rx */
    ch->rx_pid = spawn_tetra_rx(mgr->tetra_rx_path, ch->fifo_path, ch->rxid);
    if (ch->rx_pid < 0) {
        unlink(ch->fifo_path);
        return -1;
    }

    /* Open write end (blocks until tetra-rx opens read end) */
    ch->fifo_fd = open(ch->fifo_path, O_WRONLY);
    if (ch->fifo_fd < 0) {
        fprintf(stderr, "[multi] open(%s) failed: %s\n",
                ch->fifo_path, strerror(errno));
        kill(ch->rx_pid, SIGTERM);
        waitpid(ch->rx_pid, NULL, 0);
        ch->rx_pid = -1;
        unlink(ch->fifo_path);
        return -1;
    }

    /* Create demod */
    ch->demod = dqpsk_demod_create(mgr->sample_rate, mgr->channel_rate,
                                     ch->nco_offset);
    if (!ch->demod) {
        close(ch->fifo_fd); ch->fifo_fd = -1;
        kill(ch->rx_pid, SIGTERM);
        waitpid(ch->rx_pid, NULL, 0);
        ch->rx_pid = -1;
        unlink(ch->fifo_path);
        return -1;
    }

    return 0;
}

/*
 * Respawn tetra-rx for a channel after the child exits.
 * Closes old FIFO, creates new one, forks new tetra-rx.
 * Reuses the existing demod — it doesn't need to be recreated.
 * Returns 0 on success, -1 on failure or respawn limit reached.
 */
static int respawn_channel(channel_mgr_t *mgr, int idx) {
    channel_t *ch = &mgr->ch[idx];
    time_t now = time(NULL);

    /* Rate limit: reset window if it expired */
    if (now - ch->respawn_window > RESPAWN_WINDOW_SECS) {
        ch->respawn_count = 0;
        ch->respawn_window = now;
    }

    ch->respawn_count++;
    if (ch->respawn_count > RESPAWN_MAX_ATTEMPTS) {
        fprintf(stderr, "[multi] ch%d (%.3f MHz): respawn limit hit "
                "(%d in %ds), disabling channel\n",
                idx, ch->freq / 1e6, RESPAWN_MAX_ATTEMPTS,
                RESPAWN_WINDOW_SECS);
        ch->disabled = 1;
        return -1;
    }

    /* Close old FIFO write end */
    if (ch->fifo_fd >= 0) {
        close(ch->fifo_fd);
        ch->fifo_fd = -1;
    }

    /* Reap dead child */
    if (ch->rx_pid > 0) {
        waitpid(ch->rx_pid, NULL, WNOHANG);
        ch->rx_pid = -1;
    }

    /* Remove and recreate FIFO */
    unlink(ch->fifo_path);
    if (mkfifo(ch->fifo_path, 0600) < 0) {
        fprintf(stderr, "[multi] ch%d respawn: mkfifo failed: %s\n",
                idx, strerror(errno));
        return -1;
    }

    /* Fork new tetra-rx */
    ch->rx_pid = spawn_tetra_rx(mgr->tetra_rx_path, ch->fifo_path, ch->rxid);
    if (ch->rx_pid < 0) {
        unlink(ch->fifo_path);
        return -1;
    }

    /* Open write end */
    ch->fifo_fd = open(ch->fifo_path, O_WRONLY);
    if (ch->fifo_fd < 0) {
        fprintf(stderr, "[multi] ch%d respawn: open FIFO failed: %s\n",
                idx, strerror(errno));
        kill(ch->rx_pid, SIGTERM);
        waitpid(ch->rx_pid, NULL, 0);
        ch->rx_pid = -1;
        unlink(ch->fifo_path);
        return -1;
    }

    /* Reset demod state for clean start */
    dqpsk_demod_reset(ch->demod);

    fprintf(stderr, "[multi] ch%d (%.3f MHz, RXID=%d): respawned tetra-rx "
            "(attempt %d/%d)\n",
            idx, ch->freq / 1e6, ch->rxid,
            ch->respawn_count, RESPAWN_MAX_ATTEMPTS);

    return 0;
}

/* ---- Background scan helpers ---- */

/* Advance the scan to the next 25 kHz step.
 * Creates a fresh temp demod at the new frequency.
 * Skips frequencies that are already tracked or at max channels.
 * Returns 1 if a new step was started, 0 if scan is complete. */
static int scan_advance(channel_mgr_t *mgr) {
    /* Clean up previous step's demod */
    if (mgr->scan_demod) {
        dqpsk_demod_destroy(mgr->scan_demod);
        mgr->scan_demod = NULL;
    }

    /* Find next frequency that isn't already tracked */
    while (mgr->scan_freq <= mgr->scan_hi) {
        mgr->scan_freq += 25000;
        mgr->scan_step++;

        if (mgr->scan_freq > mgr->scan_hi)
            break;

        if (mgr->n_channels >= CHMGR_MAX_CHANNELS)
            break;

        if (freq_is_duplicate(mgr, mgr->scan_freq))
            continue;

        /* Create temp demod at this frequency */
        dqpsk_demod_set_quiet(1);
        float nco_offset = (float)mgr->scan_freq - (float)mgr->center_freq;
        mgr->scan_demod = dqpsk_demod_create(
            mgr->sample_rate, mgr->channel_rate, nco_offset);
        dqpsk_demod_set_quiet(0);

        if (!mgr->scan_demod)
            continue;

        dqpsk_demod_set_scan_buf(mgr->scan_demod,
                                   mgr->scan_bitbuf, SCAN_BITBUF_SIZE);
        mgr->scan_blocks = 0;

        /* Progress */
        if (mgr->scan_step % 5 == 0) {
            fprintf(stderr, "  [scan %d/%d] %.3f MHz ...\n",
                    mgr->scan_step, mgr->scan_n_steps,
                    mgr->scan_freq / 1e6);
        }

        return 1;
    }

    /* Scan complete */
    return 0;
}

/* Process one IQ block for the current scan step.
 * Called from channel_mgr_process() on each iteration. */
static void scan_process_block(channel_mgr_t *mgr,
                                 const uint8_t *iq_buf, size_t len) {
    if (!mgr->scan_demod)
        return;

    /* Feed IQ to the scan demod (fd=-1 = discard bit output) */
    dqpsk_demod_process(mgr->scan_demod, iq_buf, len, -1);
    mgr->scan_blocks++;

    /* Check if dwell period is complete */
    if (mgr->scan_blocks < mgr->scan_blocks_needed)
        return;

    /* Evaluate this step */
    size_t n_bits_len = dqpsk_demod_get_scan_len(mgr->scan_demod);
    int matches = count_training_matches(mgr->scan_bitbuf, n_bits_len);
    float rssi = dqpsk_demod_get_rssi_db(mgr->scan_demod);

    if (matches > 0) {
        fprintf(stderr, "  *** %.3f MHz  TETRA  matches=%d  RSSI=%.1f dB\n",
                mgr->scan_freq / 1e6, matches, rssi);
        mgr->scan_found++;

        /* Add channel immediately (starts FIFO + tetra-rx + demod) */
        channel_mgr_add(mgr, mgr->scan_freq);
    }

    /* Advance to next step */
    if (!scan_advance(mgr)) {
        /* Scan finished */
        mgr->scan_running = 0;
        dqpsk_demod_destroy(mgr->scan_demod);
        mgr->scan_demod = NULL;
        free(mgr->scan_bitbuf);
        mgr->scan_bitbuf = NULL;

        fprintf(stderr, "\n[multi] In-band scan complete: "
                "%d new channel(s) found, %d total active\n",
                mgr->scan_found, mgr->n_channels);
    }
}

/* ---- Public API ---- */

channel_mgr_t *channel_mgr_create(uint32_t sample_rate,
                                    uint32_t channel_rate,
                                    const char *tetra_rx_path,
                                    uint32_t center_freq) {
    channel_mgr_t *mgr = calloc(1, sizeof(*mgr));
    if (!mgr) return NULL;

    mgr->sample_rate = sample_rate;
    mgr->channel_rate = channel_rate;
    mgr->center_freq = center_freq;
    mgr->n_channels = 0;
    mgr->our_pid = getpid();

    snprintf(mgr->tetra_rx_path, sizeof(mgr->tetra_rx_path),
             "%s", tetra_rx_path);

    for (int i = 0; i < CHMGR_MAX_CHANNELS; i++) {
        mgr->ch[i].fifo_fd = -1;
        mgr->ch[i].rx_pid = -1;
    }

    return mgr;
}

int channel_mgr_add(channel_mgr_t *mgr, uint32_t freq) {
    if (mgr->n_channels >= CHMGR_MAX_CHANNELS) {
        fprintf(stderr, "[multi] Maximum %d channels reached\n",
                CHMGR_MAX_CHANNELS);
        return -1;
    }

    if (freq_is_duplicate(mgr, freq))
        return -1;

    if (!freq_in_band(mgr, freq)) {
        fprintf(stderr, "[multi] %.3f MHz outside bandwidth, skipping\n",
                freq / 1e6);
        return -1;
    }

    /* Reject channels too close to DC (RTL-SDR DC spike corrupts them) */
    float dc_offset = (float)freq - (float)mgr->center_freq;
    if (fabsf(dc_offset) < 25000.0f) {
        fprintf(stderr, "[multi] %.3f MHz too close to DC (offset %+.0f Hz), "
                "skipping\n", freq / 1e6, dc_offset);
        return -1;
    }

    int idx = mgr->n_channels;
    channel_t *ch = &mgr->ch[idx];

    ch->freq = freq;
    ch->nco_offset = (float)freq - (float)mgr->center_freq;
    ch->rxid = idx + 1;

    /* Start FIFO + tetra-rx + demod */
    if (start_channel(mgr, idx) < 0)
        return -1;

    mgr->n_channels++;

    fprintf(stderr, "[multi] Channel %d: %.3f MHz  RXID=%d  "
            "NCO=%+.0f Hz\n",
            idx, freq / 1e6, ch->rxid, ch->nco_offset);

    return idx;
}

void channel_mgr_set_spectrum(channel_mgr_t *mgr, spectrum_t *spectrum) {
    if (!mgr || !spectrum || mgr->n_channels == 0) return;
    if (mgr->ch[0].demod)
        dqpsk_demod_set_spectrum(mgr->ch[0].demod, spectrum);
}

void channel_mgr_start_scan(channel_mgr_t *mgr, int dwell_ms) {
    if (!mgr || mgr->scan_running) return;

    /* Scan range: center ± (sample_rate/2 * 0.8), in 25 kHz steps */
    float half_bw = (float)mgr->sample_rate * 0.4f;
    mgr->scan_lo = mgr->center_freq - (uint32_t)half_bw;
    mgr->scan_hi = mgr->center_freq + (uint32_t)half_bw;

    /* Round to 25 kHz grid */
    mgr->scan_lo = (mgr->scan_lo / 25000) * 25000;
    mgr->scan_hi = ((mgr->scan_hi + 24999) / 25000) * 25000;

    mgr->scan_n_steps = (int)((mgr->scan_hi - mgr->scan_lo) / 25000) + 1;
    mgr->scan_blocks_needed = (int)((float)dwell_ms / 1000.0f
                                     * (float)mgr->sample_rate * 2.0f
                                     / (float)IQ_BLOCK_BYTES);
    if (mgr->scan_blocks_needed < 3) mgr->scan_blocks_needed = 3;

    mgr->scan_bitbuf = malloc(SCAN_BITBUF_SIZE);
    if (!mgr->scan_bitbuf) return;

    /* Start at one step below scan_lo so scan_advance() moves to scan_lo */
    mgr->scan_freq = mgr->scan_lo - 25000;
    mgr->scan_step = 0;
    mgr->scan_found = 0;
    mgr->scan_demod = NULL;
    mgr->scan_blocks = 0;

    fprintf(stderr, "\n[multi] Background scan: sweeping %.1f MHz bandwidth "
            "around %.3f MHz (%d steps, %dms dwell)\n",
            mgr->sample_rate / 1e6, mgr->center_freq / 1e6,
            mgr->scan_n_steps, dwell_ms);

    /* Advance to first valid step */
    mgr->scan_running = 1;
    if (!scan_advance(mgr)) {
        mgr->scan_running = 0;
        free(mgr->scan_bitbuf);
        mgr->scan_bitbuf = NULL;
        fprintf(stderr, "[multi] No frequencies to scan.\n");
    }
}

int channel_mgr_scan_active(channel_mgr_t *mgr) {
    return mgr ? mgr->scan_running : 0;
}

int channel_mgr_process(channel_mgr_t *mgr,
                          const uint8_t *iq_buf, size_t len) {
    /* Process all active channels */
    for (int i = 0; i < mgr->n_channels; i++) {
        channel_t *ch = &mgr->ch[i];
        if (!ch->demod || ch->disabled) continue;

        /* Check if tetra-rx died (non-blocking) */
        if (ch->rx_pid > 0) {
            int status;
            pid_t p = waitpid(ch->rx_pid, &status, WNOHANG);
            if (p > 0) {
                fprintf(stderr, "[multi] tetra-rx (ch%d, RXID=%d) exited\n",
                        i, ch->rxid);
                ch->rx_pid = -1;
                if (ch->fifo_fd >= 0) {
                    close(ch->fifo_fd);
                    ch->fifo_fd = -1;
                }
                respawn_channel(mgr, i);
                continue;
            }
        }

        if (ch->fifo_fd < 0) continue;

        int ret = dqpsk_demod_process(ch->demod, iq_buf, len, ch->fifo_fd);
        if (ret < 0) {
            /* Write failed — FIFO broken, tetra-rx likely dead */
            close(ch->fifo_fd);
            ch->fifo_fd = -1;
            if (ch->rx_pid > 0) {
                waitpid(ch->rx_pid, NULL, WNOHANG);
                ch->rx_pid = -1;
            }
            respawn_channel(mgr, i);
        }
    }

    /* Background scan: process one block for the current scan step */
    if (mgr->scan_running)
        scan_process_block(mgr, iq_buf, len);

    return 0;
}

int channel_mgr_n_channels(channel_mgr_t *mgr) {
    return mgr ? mgr->n_channels : 0;
}

float channel_mgr_get_rssi(channel_mgr_t *mgr, int idx) {
    if (!mgr || idx < 0 || idx >= mgr->n_channels) return -200.0f;
    if (!mgr->ch[idx].demod) return -200.0f;
    return dqpsk_demod_get_rssi_db(mgr->ch[idx].demod);
}

uint32_t channel_mgr_get_freq(channel_mgr_t *mgr, int idx) {
    if (!mgr || idx < 0 || idx >= mgr->n_channels) return 0;
    return mgr->ch[idx].freq;
}

float channel_mgr_get_fll_hz(channel_mgr_t *mgr, int idx) {
    if (!mgr || idx < 0 || idx >= mgr->n_channels) return 0.0f;
    if (!mgr->ch[idx].demod) return 0.0f;
    return dqpsk_demod_get_fll_freq_hz(mgr->ch[idx].demod);
}

int channel_mgr_get_rxid(channel_mgr_t *mgr, int idx) {
    if (!mgr || idx < 0 || idx >= mgr->n_channels) return 0;
    return mgr->ch[idx].rxid;
}

int channel_mgr_scan_progress(channel_mgr_t *mgr, int *step, int *total) {
    if (!mgr || !mgr->scan_running) return 0;
    if (step)  *step  = mgr->scan_step;
    if (total) *total = mgr->scan_n_steps;
    return 1;
}

void channel_mgr_destroy(channel_mgr_t *mgr) {
    if (!mgr) return;

    /* Clean up scan state */
    if (mgr->scan_demod) {
        dqpsk_demod_destroy(mgr->scan_demod);
        mgr->scan_demod = NULL;
    }
    free(mgr->scan_bitbuf);
    mgr->scan_bitbuf = NULL;

    for (int i = 0; i < mgr->n_channels; i++) {
        channel_t *ch = &mgr->ch[i];

        if (ch->fifo_fd >= 0) {
            close(ch->fifo_fd);
            ch->fifo_fd = -1;
        }

        if (ch->rx_pid > 0) {
            int status;
            pid_t p = waitpid(ch->rx_pid, &status, WNOHANG);
            if (p == 0) {
                kill(ch->rx_pid, SIGTERM);
                usleep(200000);
                p = waitpid(ch->rx_pid, &status, WNOHANG);
                if (p == 0) {
                    kill(ch->rx_pid, SIGKILL);
                    waitpid(ch->rx_pid, &status, 0);
                }
            }
            ch->rx_pid = -1;
        }

        dqpsk_demod_destroy(ch->demod);
        ch->demod = NULL;

        if (ch->fifo_path[0])
            unlink(ch->fifo_path);
    }

    free(mgr);
}
