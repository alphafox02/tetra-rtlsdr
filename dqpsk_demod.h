/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * π/4-DQPSK demodulator for TETRA using liquid-dsp.
 *
 * Input:  raw RTL-SDR uint8 IQ bytes at `sample_rate` S/s
 * Output: bits (1 byte per bit, 0x00 or 0x01) written to stdout
 *
 * DSP chain:
 *   uint8 IQ → float complex → [NCO freq shift] →
 *   firdecim_crcf (decimate to 36 kHz) →
 *   agc_crcf → symsync_crcf (RRC, sps=2) →
 *   modemcf PI4DQPSK → bits → stdout
 *
 * This output is directly compatible with:
 *   tetra-rx -r -s /dev/stdin   (osmo-tetra-sq5bpf-2 by SQ5BPF)
 */

#ifndef DQPSK_DEMOD_H
#define DQPSK_DEMOD_H

#include <stdint.h>
#include <stddef.h>
#include "spectrum.h"

typedef struct dqpsk_demod dqpsk_demod_t;

/*
 * Create a π/4-DQPSK demodulator.
 *
 *   sample_rate   - input sample rate in S/s  (e.g. 1800000)
 *   channel_rate  - desired output symbol rate * sps (36000 for TETRA)
 *   freq_offset   - channel frequency offset in Hz relative to RTL-SDR
 *                   center (0 = RTL-SDR tuned to channel center)
 *
 * Decimation factor = sample_rate / channel_rate must be an integer.
 * Returns NULL on failure.
 */
dqpsk_demod_t *dqpsk_demod_create(uint32_t sample_rate,
                                   uint32_t channel_rate,
                                   float    freq_offset);

void dqpsk_demod_destroy(dqpsk_demod_t *d);

/*
 * Process a block of raw RTL-SDR IQ bytes.
 *
 *   buf  - interleaved uint8 I,Q,I,Q,... (len must be even)
 *   len  - number of bytes
 *   fd   - file descriptor for output (use 1 for stdout)
 *
 * Writes bits (0x00 or 0x01) to fd as they are decoded.
 * Returns 0 on success, -1 on write error.
 */
int dqpsk_demod_process(dqpsk_demod_t *d,
                        const uint8_t *buf, size_t len,
                        int fd);

/*
 * Get current RSSI from AGC (dB).
 */
float dqpsk_demod_get_rssi_db(dqpsk_demod_t *d);

/*
 * Get FLL carrier offset in Hz (converted from rad/sample at channel rate).
 */
float dqpsk_demod_get_fll_freq_hz(dqpsk_demod_t *d);

/*
 * Attach a spectrum object for waterfall display (optional).
 * When set, dqpsk_demod_process() feeds pre-decimation IQ samples
 * to the spectrum periodogram. Pass NULL to disable.
 */
void dqpsk_demod_set_spectrum(dqpsk_demod_t *d, spectrum_t *spectrum);

/*
 * Reset demodulator state (for retuning to a new channel).
 * Clears FLL phase/freq, AGC state, symsync, differential decoder.
 */
void dqpsk_demod_reset(dqpsk_demod_t *d);

/*
 * Set a scan buffer for collecting demodulated bits instead of writing
 * to a file descriptor. When buf is non-NULL, dqpsk_demod_process()
 * writes bits to this buffer (up to cap bytes) instead of fd.
 * Pass NULL to return to normal fd-based output.
 */
void dqpsk_demod_set_scan_buf(dqpsk_demod_t *d, uint8_t *buf, size_t cap);

/*
 * Get the number of bits written to the scan buffer.
 */
size_t dqpsk_demod_get_scan_len(dqpsk_demod_t *d);

/*
 * Suppress verbose init messages on dqpsk_demod_create().
 * Call before create to silence log output (used during in-band scan).
 */
void dqpsk_demod_set_quiet(int quiet);

#endif /* DQPSK_DEMOD_H */
