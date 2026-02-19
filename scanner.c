/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TETRA band scanner.
 *
 * Sweeps a frequency range and detects TETRA channels by looking for
 * the SYNC training sequence (y_bits, 38 bits) and Normal Training
 * Sequence 1 (n_bits, 22 bits) in the demodulated bit stream.
 *
 * Training sequences from ETSI EN 300 392-2, matching osmo-tetra's
 * tetra_burst.c definitions.
 */

#include "scanner.h"
#include "rtlsdr_source.h"
#include "dqpsk_demod.h"
#include "tetra_decode.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* TETRA training sequences (from osmo-tetra-sq5bpf-2/src/phy/tetra_burst.c) */

/* Synchronization Burst training sequence (38 bits) */
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

/* Scan buffer: holds demodulated bits for one dwell period.
 * At 36,000 bps, 5 seconds = 180,000 bits. Round up generously. */
#define SCAN_BUF_SIZE  (256 * 1024)

/* IQ read block size (same as main.c) */
#define SCAN_READ_BYTES  51200u

struct scanner {
    rtlsdr_src_t    *src;
    dqpsk_demod_t   *demod;
    uint32_t         start_hz;
    uint32_t         end_hz;
    uint32_t         step_hz;
    float            freq_offset;
    float            dwell_sec;
    uint8_t         *iq_buf;
    uint8_t         *bit_buf;
    scanner_hit_t    hits[SCANNER_MAX_HITS];
    int              n_hits;
};

/*
 * Search a bit buffer for TETRA training sequences.
 * Returns the number of matches found (SYNC + NTS1).
 *
 * If sysinfo is non-NULL, attempts to decode BSCH from SB1 (the 120 bits
 * immediately preceding each SYNC training sequence). Stores the first
 * successful decode result.
 */
static int tetra_search_training(const uint8_t *bits, size_t n_bits_len,
                                  tetra_sysinfo_t *sysinfo) {
    int count = 0;

    if (sysinfo) sysinfo->valid = 0;

    if (n_bits_len < 38) return 0;

    /* Search for SYNC training sequence (y_bits, 38 bits) */
    size_t end = n_bits_len - 38;
    for (size_t i = 0; i <= end; i++) {
        if (memcmp(bits + i, y_bits, 38) == 0) {
            count++;

            /* Attempt BSCH decode: SB1 is the 120 bits before SYNC */
            if (sysinfo && !sysinfo->valid && i >= 120) {
                tetra_decode_bsch(bits + i - 120, sysinfo);
            }

            i += 37; /* skip past this match */
        }
    }

    /* Also search for NTS1 (n_bits, 22 bits) */
    if (n_bits_len >= 22) {
        end = n_bits_len - 22;
        for (size_t i = 0; i <= end; i++) {
            if (memcmp(bits + i, n_bits, 22) == 0) {
                count++;
                i += 21;
            }
        }
    }

    return count;
}

scanner_t *scanner_create(rtlsdr_src_t *src, dqpsk_demod_t *demod,
                           uint32_t start_hz, uint32_t end_hz,
                           uint32_t step_hz, float freq_offset,
                           float dwell_sec) {
    scanner_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;

    s->src         = src;
    s->demod       = demod;
    s->start_hz    = start_hz;
    s->end_hz      = end_hz;
    s->step_hz     = step_hz > 0 ? step_hz : 25000;
    s->freq_offset = freq_offset;
    s->dwell_sec   = dwell_sec > 0.0f ? dwell_sec : 3.0f;
    s->n_hits      = 0;

    s->iq_buf  = malloc(SCAN_READ_BYTES);
    s->bit_buf = malloc(SCAN_BUF_SIZE);
    if (!s->iq_buf || !s->bit_buf) {
        free(s->iq_buf);
        free(s->bit_buf);
        free(s);
        return NULL;
    }

    return s;
}

int scanner_run(scanner_t *s) {
    uint32_t n_steps = (s->end_hz - s->start_hz) / s->step_hz + 1;
    size_t reads_per_dwell = (size_t)(s->dwell_sec * 1800000.0f * 2.0f
                                       / (float)SCAN_READ_BYTES);
    if (reads_per_dwell < 1) reads_per_dwell = 1;

    fprintf(stderr,
        "\n[scan] Scanning %.3f - %.3f MHz  "
        "(step: %u kHz, dwell: %.1f s, %u channels)\n\n",
        s->start_hz / 1e6, s->end_hz / 1e6,
        s->step_hz / 1000, s->dwell_sec, n_steps);

    s->n_hits = 0;
    struct timespec t_start;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    for (uint32_t freq = s->start_hz; freq <= s->end_hz; freq += s->step_hz) {
        uint32_t rtl_center = freq - (uint32_t)s->freq_offset;

        /* Retune RTL-SDR */
        rtlsdr_src_set_freq(s->src, rtl_center);

        /* Reset demod state for new channel */
        dqpsk_demod_reset(s->demod);

        /* Set scan buffer for collecting bits */
        dqpsk_demod_set_scan_buf(s->demod, s->bit_buf, SCAN_BUF_SIZE);

        /* Flush stale IQ from ring buffer (~50ms worth) by reading and discarding */
        for (int flush = 0; flush < 3; flush++) {
            if (rtlsdr_src_read(s->src, s->iq_buf, SCAN_READ_BYTES) < 0)
                goto done;
        }

        /* Demodulate for dwell_time */
        for (size_t r = 0; r < reads_per_dwell; r++) {
            if (rtlsdr_src_read(s->src, s->iq_buf, SCAN_READ_BYTES) < 0)
                goto done;
            /* fd=-1 since we're using scan buffer */
            dqpsk_demod_process(s->demod, s->iq_buf, SCAN_READ_BYTES, -1);
        }

        /* Search for TETRA training sequences and attempt BSCH decode */
        size_t bit_len = dqpsk_demod_get_scan_len(s->demod);
        tetra_sysinfo_t sysinfo;
        int matches = tetra_search_training(s->bit_buf, bit_len, &sysinfo);

        if (matches > 0 && s->n_hits < SCANNER_MAX_HITS) {
            scanner_hit_t *h = &s->hits[s->n_hits++];
            h->freq_hz      = freq;
            h->sync_count   = matches;
            h->rssi_db      = dqpsk_demod_get_rssi_db(s->demod);
            h->bsch_decoded = sysinfo.valid;
            h->mcc          = sysinfo.mcc;
            h->mnc          = sysinfo.mnc;
            h->colour_code  = sysinfo.colour_code;

            if (sysinfo.valid) {
                fprintf(stderr, "  *** %.3f MHz  TETRA  matches=%d  RSSI=%.1f dB"
                        "  MCC=%u MNC=%u CC=%u\n",
                        freq / 1e6, matches, h->rssi_db,
                        sysinfo.mcc, sysinfo.mnc, sysinfo.colour_code);
            } else {
                fprintf(stderr, "  *** %.3f MHz  TETRA  matches=%d  RSSI=%.1f dB"
                        "  (BSCH decode failed)\n",
                        freq / 1e6, matches, h->rssi_db);
            }
        }

        /* Progress indicator every 10 channels */
        uint32_t step_num = (freq - s->start_hz) / s->step_hz;
        if (step_num % 10 == 0) {
            fprintf(stderr, "  [%u/%u] %.3f MHz ...\r",
                    step_num + 1, n_steps, freq / 1e6);
        }
    }

done:;
    struct timespec t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_end);
    double elapsed = (t_end.tv_sec - t_start.tv_sec)
                   + (t_end.tv_nsec - t_start.tv_nsec) / 1e9;

    fprintf(stderr, "\n[scan] Done: %d TETRA channel(s) found in %u steps (%.0f s)\n",
            s->n_hits, n_steps, elapsed);

    if (s->n_hits > 0) {
        fprintf(stderr, "\n[scan] Results:\n");
        for (int i = 0; i < s->n_hits; i++) {
            scanner_hit_t *h = &s->hits[i];
            if (h->bsch_decoded) {
                fprintf(stderr, "  %2d. %.3f MHz  matches=%-4d RSSI=%5.1f dB"
                        "  MCC=%-4u MNC=%-5u CC=%u\n",
                        i + 1, h->freq_hz / 1e6, h->sync_count,
                        h->rssi_db, h->mcc, h->mnc, h->colour_code);
            } else {
                fprintf(stderr, "  %2d. %.3f MHz  matches=%-4d RSSI=%5.1f dB\n",
                        i + 1, h->freq_hz / 1e6, h->sync_count,
                        h->rssi_db);
            }
        }
        fprintf(stderr, "\n");
    }

    /* Clear scan buffer mode */
    dqpsk_demod_set_scan_buf(s->demod, NULL, 0);

    return s->n_hits;
}

const scanner_hit_t *scanner_get_hits(scanner_t *s, int *count) {
    if (count) *count = s->n_hits;
    return s->hits;
}

void scanner_destroy(scanner_t *s) {
    if (!s) return;
    free(s->iq_buf);
    free(s->bit_buf);
    free(s);
}
