/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TETRA band scanner: sweeps a frequency range, detects TETRA channels
 * by searching for SYNC training sequences in demodulated bits.
 */

#ifndef SCANNER_H
#define SCANNER_H

#include <stdint.h>

/* Forward declarations */
typedef struct rtlsdr_src rtlsdr_src_t;
typedef struct dqpsk_demod dqpsk_demod_t;

/* Maximum channels the scanner can find in one sweep */
#define SCANNER_MAX_HITS 256

typedef struct {
    uint32_t freq_hz;       /* TETRA channel frequency */
    int      sync_count;    /* number of SYNC training sequences found */
    float    rssi_db;       /* signal strength (from AGC) */
    /* BSCH decoded info (valid when bsch_decoded != 0) */
    int      bsch_decoded;  /* 1 if BSCH/SYSINFO was decoded successfully */
    uint16_t mcc;           /* Mobile Country Code (10 bits) */
    uint16_t mnc;           /* Mobile Network Code (14 bits) */
    uint8_t  colour_code;   /* Colour Code (6 bits, 0-63) */
} scanner_hit_t;

typedef struct scanner scanner_t;

/*
 * Create a scanner.
 *   src          - RTL-SDR source (must already be open, capture started)
 *   demod        - demodulator instance (will be reset between channels)
 *   start_hz     - scan start frequency in Hz
 *   end_hz       - scan end frequency in Hz
 *   step_hz      - frequency step in Hz (25000 for TETRA)
 *   freq_offset  - IF offset in Hz (500000 recommended)
 *   dwell_sec    - dwell time per channel in seconds
 */
scanner_t *scanner_create(rtlsdr_src_t *src, dqpsk_demod_t *demod,
                           uint32_t start_hz, uint32_t end_hz,
                           uint32_t step_hz, float freq_offset,
                           float dwell_sec);

/*
 * Run the scan. Prints results to stderr as it goes.
 * Returns number of TETRA channels found.
 */
int scanner_run(scanner_t *s);

/*
 * Get scan results. Returns pointer to array of hits.
 * *count is set to the number of hits.
 */
const scanner_hit_t *scanner_get_hits(scanner_t *s, int *count);

void scanner_destroy(scanner_t *s);

#endif /* SCANNER_H */
