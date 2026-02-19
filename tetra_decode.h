/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Minimal TETRA lower-MAC decoder for BSCH (Broadcast Synchronization Channel).
 * Extracts MCC, MNC, colour code from Synchronization Burst without needing
 * tetra-rx. Used by the scanner to identify discovered TETRA channels.
 *
 * Implements only what's needed for BSCH:
 *   - LFSR descrambling (fixed init=3 for SB1)
 *   - Block de-interleaving (K=120, a=11)
 *   - Rate 2/3 de-puncturing (mother rate 1/4, K=5)
 *   - Hard-decision Viterbi decoder (16 states)
 *   - CRC-16-CCITT verification
 *   - SYSINFO PDU parsing
 *
 * Reference: ETSI EN 300 392-2
 */

#ifndef TETRA_DECODE_H
#define TETRA_DECODE_H

#include <stdint.h>

/* Result of BSCH decoding */
typedef struct {
    int      valid;       /* 1 if CRC passed */
    uint16_t mcc;         /* Mobile Country Code (10 bits) */
    uint16_t mnc;         /* Mobile Network Code (14 bits) */
    uint8_t  colour_code; /* Colour Code (6 bits, 0-63) */
    uint8_t  timeslot;    /* Timeslot Number (1-4) */
    uint8_t  frame_num;   /* Frame Number (0-17) */
    uint8_t  multi_frame; /* Multiframe Number (0-59) */
} tetra_sysinfo_t;

/*
 * Decode BSCH from a Synchronization Burst.
 *
 *   sb1_bits  - 120 raw bits (uint8_t per bit, 0/1) from SB block 1
 *               (the 120 bits immediately preceding the SYNC training sequence)
 *   result    - output: decoded SYSINFO fields
 *
 * Returns 0 on success (CRC OK), -1 on failure (CRC mismatch).
 */
int tetra_decode_bsch(const uint8_t *sb1_bits, tetra_sysinfo_t *result);

#endif /* TETRA_DECODE_H */
