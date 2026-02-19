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
 * Reference: ETSI EN 300 392-2 V3.2.1
 *
 * Channel coding chain verified against osmo-tetra-sq5bpf-2 by
 * Harald Welte and Jacek Lipkowski SQ5BPF.
 */

#include "tetra_decode.h"
#include <string.h>

/* ---- Constants ---- */

/* BSCH block size */
#define BSCH_BITS       120

/* De-interleaving parameters (Section 8.2.4) */
#define BSCH_K          120
#define BSCH_A          11

/* De-puncturing: rate 2/3 from mother rate 1/4 (Section 8.2.3.1.3) */
#define DEPUNCT_OUT     320     /* 120 * 8/3 = 320 mother code bits */
#define ERASURE         0xFF    /* marker for punctured positions */

/* Puncture pattern P_rate2_3 = {0, 1, 2, 5}: keep positions 1,2,5 (1-indexed)
 * in each period of 8 mother code bits.  In 0-indexed terms: 0, 1, 4. */
static const uint8_t punct_keep[8] = {1, 1, 0, 0, 1, 0, 0, 0};

/* Viterbi decoder: rate 1/4, K=5 (Section 8.2.3.1.1) */
#define VIT_STATES      16      /* 2^(K-1) */
#define VIT_RATE        4       /* mother code rate = 1/4 */
#define VIT_INBITS      80      /* DEPUNCT_OUT / VIT_RATE */

/*
 * Generator polynomials for TETRA rate 1/4 convolutional code:
 *   G1(D) = 1 + D + D^4          = 0x13
 *   G2(D) = 1 + D^2 + D^3 + D^4  = 0x1D
 *   G3(D) = 1 + D + D^2 + D^4    = 0x17
 *   G4(D) = 1 + D + D^3 + D^4    = 0x1B
 *
 * Convention: sr = (state << 1) | input_bit
 *   sr bit i corresponds to delay element m[i] (m[0]=input, m[1..4]=state)
 *   state = m1 | (m2<<1) | (m3<<2) | (m4<<3)
 *
 * Verified against osmo-tetra conv_enc: the encoder output ordering
 * (G1,G2,G3,G4) and state transition match.
 */
static const uint8_t gen_poly[4] = {0x13, 0x1D, 0x17, 0x1B};

/* CRC-16-CCITT: polynomial 0x1021, init 0xFFFF (Section 8.2.3.2) */
#define CRC_INIT        0xFFFFu
#define CRC_GOOD        0x1D0Fu  /* residue after processing data+CRC */


/* ======================================================================
 *  LFSR Descrambling (Section 8.2.5)
 *
 *  Fibonacci LFSR, 32 bits, polynomial:
 *    p(D) = 1+D+D^2+D^4+D^5+D^7+D^8+D^10+D^11+D^12+D^16+D^22+D^23+D^26+D^32
 *
 *  Taps at e(1),e(2),e(4),e(5),e(7),e(8),e(10),e(11),e(12),e(16),
 *          e(22),e(23),e(26),e(32)
 *
 *  Convention: e(1) = bit 31 (MSB), e(32) = bit 0 (LSB).
 *  Output = XOR of all tapped positions, then shift right, new bit to MSB.
 *
 *  For BSCH (SB1), init = 3 (e(31)=e(32)=1, all others 0).
 *
 *  Implementation matches osmo-tetra lower_mac/tetra_scramb.c exactly.
 * ====================================================================== */

/* ST(x,y) extracts e(y) from a 32-bit register where e(1)=bit31, e(32)=bit0 */
#define ST(x, y)  ((x) >> (32-(y)))

static void descramble(uint8_t *bits, int n, uint32_t init) {
    uint32_t lfsr = init;
    for (int i = 0; i < n; i++) {
        /* feedback = XOR of taps: 32,26,23,22,16,12,11,10,8,7,5,4,2,1 */
        uint32_t bit = (ST(lfsr,32) ^ ST(lfsr,26) ^ ST(lfsr,23) ^ ST(lfsr,22) ^
                        ST(lfsr,16) ^ ST(lfsr,12) ^ ST(lfsr,11) ^ ST(lfsr,10) ^
                        ST(lfsr, 8) ^ ST(lfsr, 7) ^ ST(lfsr, 5) ^ ST(lfsr, 4) ^
                        ST(lfsr, 2) ^ ST(lfsr, 1)) & 1;
        /* output bit = feedback */
        bits[i] ^= (uint8_t)bit;
        /* shift right, new bit to MSB */
        lfsr = (lfsr >> 1) | (bit << 31);
    }
}


/* ======================================================================
 *  Block De-interleaving (Section 8.2.4)
 *
 *  Permutation: j(k) = 1 + (a * k) mod K,  k = 1..K
 *  Interleaver writes input[k] to output[j(k)].
 *  De-interleaver reverses: output[k-1] = input[j(k)-1].
 *
 *  Uses osmo-tetra convention (a*k, not a*(k-1)) matching
 *  lower_mac/tetra_interleave.c block_deinterleave().
 * ====================================================================== */

static void deinterleave(const uint8_t *in, uint8_t *out) {
    for (int k = 1; k <= BSCH_K; k++) {
        int j = 1 + (BSCH_A * k) % BSCH_K;
        out[k - 1] = in[j - 1];
    }
}


/* ======================================================================
 *  Rate 2/3 De-puncturing (Section 8.2.3.1.3)
 *
 *  Period = 8, t = 3, P = {0, 1, 2, 5} (osmo-tetra P_rate2_3).
 *  Keep positions {0, 1, 4} (0-indexed) in each group of 8 mother bits.
 *  Insert ERASURE at punctured positions.
 *
 *  120 coded bits → 320 de-punctured bits (40 groups × 8).
 * ====================================================================== */

static void depuncture(const uint8_t *in, uint8_t *out) {
    int in_idx = 0;
    for (int i = 0; i < DEPUNCT_OUT; i++) {
        if (punct_keep[i & 7])
            out[i] = in[in_idx++];
        else
            out[i] = ERASURE;
    }
}


/* ======================================================================
 *  Hard-decision Viterbi Decoder (rate 1/4, K=5, 16 states)
 *
 *  State = m1|(m2<<1)|(m3<<2)|(m4<<3) where m1=most recent memory.
 *  Transition: next_state = ((state << 1) | input_bit) & 0x0F.
 *  Encoder output: for each generator, parity of (sr & gen_poly[g]).
 * ====================================================================== */

/* Compute 4-bit encoder output for a given state and input bit */
static uint8_t encoder_output(uint8_t state, uint8_t input_bit) {
    uint8_t sr = (uint8_t)((state << 1) | input_bit);
    uint8_t out = 0;
    for (int g = 0; g < VIT_RATE; g++) {
        uint8_t v = sr & gen_poly[g];
        v ^= v >> 4;
        v ^= v >> 2;
        v ^= v >> 1;
        out |= (uint8_t)((v & 1) << g);
    }
    return out;
}

/*
 * Hamming distance between expected 4-bit output and received 4 symbols.
 * Erased (punctured) positions contribute 0 to the metric.
 */
static int branch_metric(uint8_t expected, const uint8_t *received) {
    int d = 0;
    for (int i = 0; i < VIT_RATE; i++) {
        if (received[i] != ERASURE && ((expected >> i) & 1) != received[i])
            d++;
    }
    return d;
}

/*
 * Decode 320 de-punctured bits → 80 decoded bits.
 */
static void viterbi_decode(const uint8_t *coded, uint8_t *decoded) {
    int pm[VIT_STATES];                     /* path metrics (current) */
    int pm_new[VIT_STATES];                 /* path metrics (next) */
    uint8_t tb[VIT_INBITS][VIT_STATES];     /* traceback: previous state */

    /* Init: state 0 has metric 0, all others are unreachable */
    for (int s = 0; s < VIT_STATES; s++)
        pm[s] = 999;
    pm[0] = 0;

    /* Forward pass: process one 4-bit symbol group per input bit */
    for (int t = 0; t < VIT_INBITS; t++) {
        const uint8_t *sym = &coded[t * VIT_RATE];

        for (int s = 0; s < VIT_STATES; s++)
            pm_new[s] = 999;

        for (int prev = 0; prev < VIT_STATES; prev++) {
            if (pm[prev] >= 999) continue;

            for (uint8_t bit = 0; bit < 2; bit++) {
                int next = ((prev << 1) | bit) & 0x0F;
                uint8_t exp = encoder_output((uint8_t)prev, bit);
                int m = pm[prev] + branch_metric(exp, sym);

                if (m < pm_new[next]) {
                    pm_new[next] = m;
                    tb[t][next] = (uint8_t)prev;
                }
            }
        }

        memcpy(pm, pm_new, sizeof(pm));
    }

    /* Find best final state (should be 0 due to tail bits, but be robust) */
    int best = 0;
    for (int s = 1; s < VIT_STATES; s++) {
        if (pm[s] < pm[best])
            best = s;
    }

    /* Traceback: recover input bits.
     * Since next = ((prev << 1) | bit) & 0xF, the input bit = next & 1. */
    int state = best;
    for (int t = VIT_INBITS - 1; t >= 0; t--) {
        decoded[t] = (uint8_t)(state & 1);
        state = tb[t][state];
    }
}


/* ======================================================================
 *  CRC-16-CCITT (Section 8.2.3.2)
 *
 *  Polynomial x^16 + x^12 + x^5 + 1 (= 0x1021), init 0xFFFF.
 *  Process all bits (data + CRC); valid residue = 0x1D0F.
 *
 *  Bit-by-bit implementation, matches osmo-tetra crc16_ccitt_bits().
 * ====================================================================== */

static uint16_t crc16(const uint8_t *bits, int n) {
    uint16_t crc = CRC_INIT;
    for (int i = 0; i < n; i++) {
        uint16_t b = bits[i] ^ ((crc >> 15) & 1);
        crc = (uint16_t)(crc << 1);
        if (b) crc ^= 0x1021;
    }
    return crc;
}


/* ======================================================================
 *  SYSINFO PDU Parsing (Section 18.4.38)
 *
 *  60 data bits (type-1), bit layout:
 *    [0..3]   System Code (4 bits)
 *    [4..9]   Colour Code (6 bits, 0-63)
 *    [10..11] Timeslot Number (2 bits, 0=TS1 .. 3=TS4)
 *    [12..16] Frame Number (5 bits, 1-18)
 *    [17..22] Multiframe Number (6 bits, 1-60)
 *    [23..24] Sharing Mode (2 bits)
 *    [25..26] TS Reserved Frames (2 bits)
 *    [27..28] U-plane DTX (2 bits)
 *    [29..30] Frame 18 Extension (2 bits)
 *    [31..40] MCC - Mobile Country Code (10 bits)
 *    [41..54] MNC - Mobile Network Code (14 bits)
 *    [55..57] Neighbour Cell Broadcast (3 bits)
 *    [58..59] Cell Re-select Parameters (2 bits)
 * ====================================================================== */

static uint16_t bits_to_uint(const uint8_t *bits, int start, int len) {
    uint16_t val = 0;
    for (int i = 0; i < len; i++)
        val = (uint16_t)((val << 1) | bits[start + i]);
    return val;
}


/* ======================================================================
 *  Public API
 * ====================================================================== */

int tetra_decode_bsch(const uint8_t *sb1_bits, tetra_sysinfo_t *result) {
    uint8_t work[BSCH_BITS];
    uint8_t deint[BSCH_BITS];
    uint8_t depunct_buf[DEPUNCT_OUT];
    uint8_t decoded[VIT_INBITS];

    memset(result, 0, sizeof(*result));

    /* Step 1: Copy and descramble (init=3 for SB1, Section 8.2.5.2) */
    memcpy(work, sb1_bits, BSCH_BITS);
    descramble(work, BSCH_BITS, 3);

    /* Step 2: Block de-interleave (K=120, a=11) */
    deinterleave(work, deint);

    /* Step 3: De-puncture rate 2/3 → rate 1/4 (120 → 320 bits) */
    depuncture(deint, depunct_buf);

    /* Step 4: Viterbi decode (rate 1/4, K=5 → 320 → 80 bits) */
    viterbi_decode(depunct_buf, decoded);

    /* Step 5: CRC-16-CCITT check over 76 bits (60 data + 16 CRC) */
    uint16_t crc = crc16(decoded, 76);
    if (crc != CRC_GOOD)
        return -1;

    /* Step 6: Parse SYSINFO PDU (60 data bits) */
    result->valid       = 1;
    result->colour_code = (uint8_t)bits_to_uint(decoded, 4, 6);
    result->timeslot    = (uint8_t)(bits_to_uint(decoded, 10, 2) + 1);
    result->frame_num   = (uint8_t)bits_to_uint(decoded, 12, 5);
    result->multi_frame = (uint8_t)bits_to_uint(decoded, 17, 6);
    result->mcc         = bits_to_uint(decoded, 31, 10);
    result->mnc         = bits_to_uint(decoded, 41, 14);

    return 0;
}
