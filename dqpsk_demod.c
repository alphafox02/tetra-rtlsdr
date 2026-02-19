/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * π/4-DQPSK demodulator for TETRA with band-edge FLL carrier recovery.
 *
 * DSP chain:
 *
 *   raw uint8 IQ (1.8 MSps)
 *     → float complex
 *     → nco_crcf           (fixed IF offset to avoid RTL-SDR DC spike)
 *     → firdecim_crcf      (50× → 36 kSps Kaiser LPF + decimation)
 *     → feedforward AGC     (amplitude normalization — MUST precede FLL)
 *     → band-edge FLL      (carrier recovery, 2nd-order PLL at 36 kSps)
 *     → symsync_crcf       (RRC timing recovery, sps=2)
 *     → π/4-DQPSK differential decode (ETSI EN 300 392-2, Table 9.35)
 *     → 2 bits/symbol written as 0x00/0x01 bytes to stdout
 *
 * The band-edge FLL (based on GnuRadio's fll_band_edge_cc) works by
 * comparing power at the upper and lower spectral edges of the RRC-shaped
 * signal. This provides a carrier frequency error estimate that works
 * regardless of data content, before symbol timing recovery.
 *
 * Output format is identical to simdemod3_telive.py and is consumed by:
 *   tetra-rx -r -s /dev/stdin   (from osmo-tetra-sq5bpf-2 by SQ5BPF)
 */

#include "dqpsk_demod.h"

#include <liquid/liquid.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <complex.h>

/* TETRA parameters */
#define TETRA_CHANNEL_RATE  36000u      /* 36 kSps after decimation */
#define TETRA_SYMBOL_RATE   18000u      /* 18 kSym/s */
#define TETRA_SPS           2           /* samples per symbol */
#define TETRA_ROLLOFF       0.35f       /* RRC roll-off factor */

/* Timing recovery filter bank */
#define SYMSYNC_NFILTS      32          /* polyphase branches */
#define SYMSYNC_M           5           /* RRC filter delay [symbols] */

/* Output bit buffer */
#define BITBUF_SIZE         4096

/* Band-edge FLL parameters (match GnuRadio fll_band_edge_cc exactly) */
#define FLL_FILTER_SIZE 45              /* filter length (same as simdemod3) */
#define FLL_BW (M_PI / 100.0)

/* Feedforward AGC: matches GnuRadio's analog.feedforward_agc_cc(8, 1.0).
 * Computes running |x|² over FF_AGC_WINDOW samples, scales to unity RMS.
 * Must precede the band-edge FLL so the power-based error signal stays
 * properly scaled regardless of input signal level. */
#define FF_AGC_WINDOW 8

/* sinc(x) = sin(π·x) / (π·x), matching GnuRadio's definition */
static float sincf_gr(float x) {
    float arg = (float)M_PI * x;
    return (x == 0.0f) ? 1.0f : sinf(arg) / arg;
}

struct dqpsk_demod {
    /* Decimating FIR filter (input_rate → channel_rate) */
    firdecim_crcf   decim;
    unsigned int    decim_factor;
    uint32_t        sample_rate;

    /* NCO for IF shift (1.8 MSps, FIXED — not adjusted by FLL) */
    nco_crcf        nco;

    /* Band-edge FLL carrier recovery (operates at 36 kSps) */
    firfilt_cccf    fll_upper;          /* upper band-edge filter */
    firfilt_cccf    fll_lower;          /* lower band-edge filter */
    float           fll_phase;          /* NCO phase [rad] */
    float           fll_freq;           /* NCO frequency [rad/sample at 36k] */
    float           fll_alpha;          /* proportional loop gain */
    float           fll_beta;           /* integral loop gain */

    /* Feedforward AGC (replaces liquid-dsp agc_crcf) */
    float           ff_agc_pwr[FF_AGC_WINDOW]; /* ring buffer of |x|² */
    unsigned int    ff_agc_idx;                /* ring buffer write index */
    float           ff_agc_sum;                /* running sum of |x|² */

    /* Symbol synchronizer (polyphase RRC matched filter + Gardner TED) */
    symsync_crcf    symsync;

    /* π/4-DQPSK differential decoder state */
    float complex   dqpsk_prev;

    /* Working buffers */
    float complex   *conv_buf;          /* uint8 IQ → float complex */
    float complex   *decim_buf;         /* firdecim output */
    float complex   *fll_buf;           /* band-edge FLL output */
    float complex   *agc_buf;           /* AGC output */
    float complex   *sync_buf;          /* symsync output */
    size_t          decim_block;        /* decimated samples per processing block */

    /* Bit output buffer */
    uint8_t         bitbuf[BITBUF_SIZE];
    size_t          bitbuf_pos;

    /* Stats */
    float           rssi_db;

    /* Optional spectrum */
    spectrum_t     *spectrum;

    /* Scan buffer (when non-NULL, bits go here instead of fd) */
    uint8_t        *scan_buf;
    size_t          scan_buf_cap;
    size_t          scan_buf_len;
};

/* When non-zero, dqpsk_demod_create() suppresses verbose init messages */
static int g_demod_quiet = 0;

void dqpsk_demod_set_quiet(int quiet) {
    g_demod_quiet = quiet;
}

dqpsk_demod_t *dqpsk_demod_create(uint32_t sample_rate,
                                   uint32_t channel_rate,
                                   float    freq_offset) {
    if (sample_rate % channel_rate != 0) {
        fprintf(stderr, "[demod] sample_rate %u must be divisible by "
                "channel_rate %u\n", sample_rate, channel_rate);
        return NULL;
    }

    dqpsk_demod_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->decim_factor = sample_rate / channel_rate;
    d->sample_rate  = sample_rate;
    d->dqpsk_prev   = 1.0f + 0.0f * I;

    if (!g_demod_quiet) {
        fprintf(stderr, "[demod] %u S/s ÷ %u = %u S/s (%.1f kSps)\n",
                sample_rate, d->decim_factor, channel_rate,
                channel_rate / 1000.0f);
        fprintf(stderr, "[demod] Symbol rate: %u Sym/s, sps=%d\n",
                TETRA_SYMBOL_RATE, TETRA_SPS);
    }

    /* --- Decimating FIR (Kaiser LPF) --- */
    d->decim = firdecim_crcf_create_kaiser(d->decim_factor, 12u, 60.0f);
    if (!d->decim) {
        fprintf(stderr, "[demod] Failed to create firdecim\n");
        free(d);
        return NULL;
    }

    /* --- NCO for IF shift (FIXED, not adjusted by FLL) --- */
    d->nco = nco_crcf_create(LIQUID_NCO);
    {
        float omega = 2.0f * (float)M_PI * freq_offset / (float)sample_rate;
        nco_crcf_set_frequency(d->nco, omega);
        if (!g_demod_quiet && fabsf(freq_offset) > 1.0f)
            fprintf(stderr, "[demod] IF offset: %.0f Hz\n", freq_offset);
    }

    /* --- Band-edge FLL (carrier recovery at channel rate) ---
     *
     * Exact replica of GnuRadio's fll_band_edge_cc::design_filter().
     *
     * The band-edge filter is the FREQUENCY DERIVATIVE of the matched
     * filter.  In time domain this equals the sum of two sincs:
     *   h_bb[i] = sinc(α·k − 0.5) + sinc(α·k + 0.5)
     * where α = rolloff and k is a normalised time index.
     *
     * The baseband prototype is then modulated to the upper/lower band
     * edges at ±(1+α)/(2·sps) and power-normalised.
     */
    {
        const unsigned int fll_len = FLL_FILTER_SIZE;   /* 45 */
        const int M = (int)rintf((float)fll_len / (float)TETRA_SPS); /* 22 */
        const float half_sps_inv = 2.0f / (float)TETRA_SPS;         /* 1.0 */

        /* Step 1: baseband prototype (sum of two sincs) */
        float bb[FLL_FILTER_SIZE];
        float power = 0.0f;
        for (unsigned int i = 0; i < fll_len; i++) {
            float k = (float)(-M) + (float)i * half_sps_inv;
            float pos = TETRA_ROLLOFF * k;
            float tap = sincf_gr(pos - 0.5f) + sincf_gr(pos + 0.5f);
            power += tap * tap;
            bb[i] = tap;
        }

        /* Step 2: modulate to band edges and normalise.
         * NO explicit time-reversal: liquid-dsp's firfilt_cccf does
         * convolution internally (which already time-reverses the kernel).
         * GnuRadio's FIR filters also do standard convolution. */
        float complex *h_lower = malloc(fll_len * sizeof(float complex));
        float complex *h_upper = malloc(fll_len * sizeof(float complex));

        const int N = ((int)fll_len - 1) / 2;               /* 22 */
        const float invpower = 1.0f / power;
        const float inv_twice_sps = 0.5f / (float)TETRA_SPS; /* 0.25 */

        for (unsigned int i = 0; i < fll_len; i++) {
            float tap = bb[i] * invpower;

            /* GnuRadio: lower = proto * exp(-j·2π·shift·k)
             *           upper = proto * exp(+j·2π·shift·k)
             * shift = (1+rolloff) / (2·sps) */
            float ph = 2.0f * (float)M_PI
                        * (1.0f + TETRA_ROLLOFF) * inv_twice_sps
                        * (float)((int)i - N);
            h_lower[i] = tap * cexpf(-I * ph);
            h_upper[i] = tap * cexpf( I * ph);
        }

        d->fll_lower = firfilt_cccf_create(h_lower, fll_len);
        d->fll_upper = firfilt_cccf_create(h_upper, fll_len);
        free(h_lower);
        free(h_upper);

        /* Loop filter: 2nd-order PLL matching GnuRadio's control_loop.
         *
         * GnuRadio control_loop::update_gains():
         *   damping = sqrt(2)/2
         *   denom = 1 + 2*damping*bw + bw^2
         *   alpha = 4*damping*bw / denom    (proportional → phase)
         *   beta  = 4*bw^2 / denom          (integral → frequency)
         *
         * For FLL_BW = π/100 ≈ 0.0314:
         *   alpha ≈ 0.086,  beta ≈ 0.00382
         *
         * Previous bug: used ad-hoc formula giving beta=0.395 (27x too large),
         * alpha=0, and wrong error sign.  This caused the FLL to diverge
         * immediately, spinning the NCO at ±2π and destroying the signal. */
        {
            float bw = (float)FLL_BW;
            float damping = sqrtf(2.0f) / 2.0f;
            float denom = 1.0f + 2.0f * damping * bw + bw * bw;
            d->fll_alpha = 4.0f * damping * bw / denom;
            d->fll_beta  = 4.0f * bw * bw / denom;
        }
        d->fll_phase = 0.0f;
        d->fll_freq  = 0.0f;

        if (!g_demod_quiet)
            fprintf(stderr, "[demod] Band-edge FLL: %u taps, bw=%.4f, "
                    "alpha=%.4f, beta=%.6f\n",
                    fll_len, (float)FLL_BW, d->fll_alpha, d->fll_beta);
    }

    /* --- Feedforward AGC (matching GnuRadio's feedforward_agc_cc(8,1)) --- */
    memset(d->ff_agc_pwr, 0, sizeof(d->ff_agc_pwr));
    d->ff_agc_idx = 0;
    d->ff_agc_sum = 0.0f;

    /* --- Symbol synchronizer (PFB, RRC matched filter) --- */
    d->symsync = symsync_crcf_create_rnyquist(
        LIQUID_FIRFILT_RRC,
        TETRA_SPS,
        SYMSYNC_M,
        TETRA_ROLLOFF,
        SYMSYNC_NFILTS);
    if (!d->symsync) {
        fprintf(stderr, "[demod] Failed to create symsync\n");
        dqpsk_demod_destroy(d);
        return NULL;
    }
    symsync_crcf_set_lf_bw(d->symsync, 0.01f);

    /* Allocate buffers */
    d->decim_block = TETRA_SPS * SYMSYNC_NFILTS * 4;   /* 256 */

    size_t raw_block = d->decim_block * d->decim_factor;
    d->conv_buf  = malloc(raw_block        * sizeof(float complex));
    d->decim_buf = malloc(d->decim_block   * sizeof(float complex));
    d->fll_buf   = malloc(d->decim_block   * sizeof(float complex));
    d->agc_buf   = malloc(d->decim_block   * sizeof(float complex));
    d->sync_buf  = malloc(d->decim_block   * sizeof(float complex));

    if (!d->conv_buf || !d->decim_buf || !d->fll_buf ||
        !d->agc_buf  || !d->sync_buf) {
        fprintf(stderr, "[demod] Out of memory\n");
        dqpsk_demod_destroy(d);
        return NULL;
    }

    if (!g_demod_quiet)
        fprintf(stderr,
                "[demod] π/4-DQPSK demodulator ready.\n"
                "[demod] Output → stdout.\n");
    return d;
}

void dqpsk_demod_destroy(dqpsk_demod_t *d) {
    if (!d) return;
    if (d->decim)     firdecim_crcf_destroy(d->decim);
    if (d->nco)       nco_crcf_destroy(d->nco);
    if (d->fll_upper) firfilt_cccf_destroy(d->fll_upper);
    if (d->fll_lower) firfilt_cccf_destroy(d->fll_lower);
    /* feedforward AGC has no heap allocation to free */
    if (d->symsync)   symsync_crcf_destroy(d->symsync);
    free(d->conv_buf);
    free(d->decim_buf);
    free(d->fll_buf);
    free(d->agc_buf);
    free(d->sync_buf);
    free(d);
}

static int flush_bits(dqpsk_demod_t *d, int fd) {
    if (d->bitbuf_pos == 0) return 0;
    if (d->scan_buf) {
        size_t avail = d->scan_buf_cap - d->scan_buf_len;
        size_t n = d->bitbuf_pos < avail ? d->bitbuf_pos : avail;
        memcpy(d->scan_buf + d->scan_buf_len, d->bitbuf, n);
        d->scan_buf_len += n;
        d->bitbuf_pos = 0;
        return 0;
    }
    size_t remaining = d->bitbuf_pos;
    size_t offset = 0;
    while (remaining > 0) {
        ssize_t w = write(fd, d->bitbuf + offset, remaining);
        if (w < 0) return -1;
        offset    += (size_t)w;
        remaining -= (size_t)w;
    }
    d->bitbuf_pos = 0;
    return 0;
}

static inline int emit_bit(dqpsk_demod_t *d, uint8_t bit, int fd) {
    d->bitbuf[d->bitbuf_pos++] = bit & 1u;
    if (d->bitbuf_pos >= BITBUF_SIZE)
        return flush_bits(d, fd);
    return 0;
}

int dqpsk_demod_process(dqpsk_demod_t *d,
                        const uint8_t *buf, size_t len,
                        int fd) {
    size_t bytes_per_block = d->decim_block * d->decim_factor * 2;

    size_t pos = 0;
    while (pos + bytes_per_block <= len) {
        const uint8_t *p = buf + pos;
        size_t n_raw = d->decim_block * d->decim_factor;

        /* Step 1: uint8 IQ → float complex */
        for (size_t i = 0; i < n_raw; i++) {
            float re = ((float)p[2*i]     - 127.5f) * (1.0f / 127.5f);
            float im = ((float)p[2*i + 1] - 127.5f) * (1.0f / 127.5f);
            d->conv_buf[i] = re + im * I;
        }

        /* Step 1b: spectrum waterfall */
        if (d->spectrum)
            spectrum_push(d->spectrum, d->conv_buf, n_raw, d->rssi_db, 8);

        /* Step 2: NCO IF shift (fixed) */
        nco_crcf_mix_block_down(d->nco, d->conv_buf, d->conv_buf, n_raw);

        /* Step 3: Decimating FIR */
        firdecim_crcf_execute_block(d->decim,
                                    d->conv_buf,
                                    d->decim_block,
                                    d->decim_buf);

        /* Step 4: Feedforward AGC (amplitude normalization)
         *
         * MUST precede the band-edge FLL.  The FLL error signal is
         * 0.5·(|lower|² − |upper|²), which scales with signal power².
         * Without normalisation the effective loop gain is far too high
         * and the FLL diverges.  GnuRadio's simdemod3 places
         * analog.feedforward_agc_cc(8,1) before digital.fll_band_edge_cc.
         *
         * Implementation: sliding window of FF_AGC_WINDOW=8 |x|² values,
         * output = input / sqrt(avg_power).  Maintains state across blocks. */
        {
            float pwr_sum = d->ff_agc_sum;
            for (size_t i = 0; i < d->decim_block; i++) {
                float complex s = d->decim_buf[i];
                float pwr = crealf(s)*crealf(s) + cimagf(s)*cimagf(s);

                /* Update running sum: subtract oldest, add newest */
                pwr_sum -= d->ff_agc_pwr[d->ff_agc_idx];
                d->ff_agc_pwr[d->ff_agc_idx] = pwr;
                pwr_sum += pwr;
                d->ff_agc_idx = (d->ff_agc_idx + 1) % FF_AGC_WINDOW;

                float avg = pwr_sum / (float)FF_AGC_WINDOW;
                float gain = (avg > 1e-20f) ? 1.0f / sqrtf(avg) : 1.0f;
                d->agc_buf[i] = s * gain;
            }
            d->ff_agc_sum = pwr_sum;
            /* RSSI from average power across the block */
            d->rssi_db = (d->ff_agc_sum > 1e-20f)
                       ? 10.0f * log10f(d->ff_agc_sum / (float)FF_AGC_WINDOW)
                       : -200.0f;
        }

        /* Step 5: Band-edge FLL carrier recovery (at 36 kSps)
         *
         * Matches GnuRadio fll_band_edge_cc::work() + control_loop.
         * NCO: exp(+j·phase) (same convention as GnuRadio).
         * Error: 0.5 * (|filter_lower(x)|² − |filter_upper(x)|²)
         * Loop:  freq += beta·error;  phase += freq + alpha·error */
        for (size_t i = 0; i < d->decim_block; i++) {
            /* Mix with carrier recovery NCO (exp(+j·phase)) */
            float complex x = d->agc_buf[i] *
                              cexpf(I * d->fll_phase);

            /* Band-edge filters */
            float complex fl_out, fu_out;
            firfilt_cccf_push(d->fll_lower, x);
            firfilt_cccf_execute(d->fll_lower, &fl_out);
            firfilt_cccf_push(d->fll_upper, x);
            firfilt_cccf_execute(d->fll_upper, &fu_out);

            float fl_pwr = crealf(fl_out) * crealf(fl_out) +
                           cimagf(fl_out) * cimagf(fl_out);
            float fu_pwr = crealf(fu_out) * crealf(fu_out) +
                           cimagf(fu_out) * cimagf(fu_out);

            /* Error: 0.5 * (|lower|² - |upper|²)
             * (matching GnuRadio fll_band_edge_cc exactly) */
            float error = 0.5f * (fl_pwr - fu_pwr);

            /* 2nd-order PLL: GnuRadio control_loop::advance_loop()
             *   freq  += beta * error
             *   phase += freq + alpha * error  */
            d->fll_freq  += d->fll_beta * error;
            d->fll_phase += d->fll_freq + d->fll_alpha * error;

            /* Phase wrap to (−π, π] */
            while (d->fll_phase >  (float)M_PI) d->fll_phase -= 2.0f*(float)M_PI;
            while (d->fll_phase < -(float)M_PI) d->fll_phase += 2.0f*(float)M_PI;

            /* Frequency limit: ±2π (same as GnuRadio max_freq = 2π·2/sps) */
            float fll_max = 2.0f * (float)M_PI;
            if (d->fll_freq >  fll_max) d->fll_freq =  fll_max;
            if (d->fll_freq < -fll_max) d->fll_freq = -fll_max;

            d->fll_buf[i] = x;
        }

        /* Step 6: Symbol synchronizer */
        unsigned int n_syms = 0;
        symsync_crcf_execute(d->symsync,
                             d->fll_buf, d->decim_block,
                             d->sync_buf, &n_syms);

        /* Step 7: π/4-DQPSK differential decode
         *
         * Full chain: NCO → decim → AGC → FLL → symsync → diff decode */
        for (unsigned int i = 0; i < n_syms; i++) {
            float complex z = d->sync_buf[i] * conjf(d->dqpsk_prev);
            d->dqpsk_prev = d->sync_buf[i];

            float dphi = cargf(z);

            uint8_t db1, db2;
            if (dphi >= 0.0f) {
                db1 = 0;
                db2 = (dphi >= (float)M_PI / 2.0f) ? 1u : 0u;
            } else {
                db1 = 1;
                db2 = (dphi < -(float)M_PI / 2.0f) ? 1u : 0u;
            }

            if (emit_bit(d, db1, fd) < 0) return -1;
            if (emit_bit(d, db2, fd) < 0) return -1;
        }

        pos += bytes_per_block;
    }

    return flush_bits(d, fd);
}

float dqpsk_demod_get_rssi_db(dqpsk_demod_t *d) {
    return d->rssi_db;
}

float dqpsk_demod_get_fll_freq_hz(dqpsk_demod_t *d) {
    if (!d) return 0.0f;
    /* fll_freq is in rad/sample at channel_rate (36000 S/s) */
    return d->fll_freq * 36000.0f / (2.0f * (float)M_PI);
}

void dqpsk_demod_set_spectrum(dqpsk_demod_t *d, spectrum_t *spectrum) {
    d->spectrum = spectrum;
}

void dqpsk_demod_reset(dqpsk_demod_t *d) {
    /* Reset FLL */
    d->fll_phase = 0.0f;
    d->fll_freq  = 0.0f;
    firfilt_cccf_reset(d->fll_lower);
    firfilt_cccf_reset(d->fll_upper);

    /* Reset feedforward AGC */
    memset(d->ff_agc_pwr, 0, sizeof(d->ff_agc_pwr));
    d->ff_agc_idx = 0;
    d->ff_agc_sum = 0.0f;

    /* Reset symbol synchronizer */
    symsync_crcf_reset(d->symsync);

    /* Reset differential decoder */
    d->dqpsk_prev = 1.0f + 0.0f * I;

    /* Reset bit buffer */
    d->bitbuf_pos = 0;

    /* Reset scan buffer position */
    d->scan_buf_len = 0;
}

void dqpsk_demod_set_scan_buf(dqpsk_demod_t *d, uint8_t *buf, size_t cap) {
    d->scan_buf     = buf;
    d->scan_buf_cap = cap;
    d->scan_buf_len = 0;
}

size_t dqpsk_demod_get_scan_len(dqpsk_demod_t *d) {
    return d->scan_buf_len;
}
