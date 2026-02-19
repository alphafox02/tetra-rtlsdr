/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RTL-SDR IQ capture with lock-free ring buffer.
 */

#ifndef RTLSDR_SOURCE_H
#define RTLSDR_SOURCE_H

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>

/* Ring buffer: holds raw interleaved uint8 IQ bytes from RTL-SDR.
 * Must be a power of 2 for fast modular indexing.
 * 8 MB = ~2 seconds at 1.8 MSps (2 bytes per sample). */
#define RING_BYTES (1 << 23)   /* 8 MB */
#define RING_MASK  (RING_BYTES - 1)

typedef struct {
    uint8_t         data[RING_BYTES];
    volatile size_t head;       /* write position (callback thread) */
    volatile size_t tail;       /* read  position (consumer thread) */
    pthread_mutex_t mutex;
    pthread_cond_t  not_empty;
    int             done;       /* set to 1 to signal shutdown */
    size_t          overflow;   /* dropped bytes due to buffer full */
} ring_buf_t;

typedef struct rtlsdr_src rtlsdr_src_t;

/*
 * Create RTL-SDR source.
 *   freq        - center frequency in Hz (e.g. 392640000)
 *   sample_rate - capture rate in S/s   (e.g. 1800000)
 *   gain_tenths - gain in 0.1 dB units, 0 = auto-gain
 *   device_idx  - RTL-SDR device index (usually 0)
 *   ppm         - crystal error correction in ppm
 *
 * Returns NULL on failure.
 */
rtlsdr_src_t *rtlsdr_src_create(uint32_t freq, uint32_t sample_rate,
                                 int gain_tenths, int device_idx, int ppm);

void rtlsdr_src_destroy(rtlsdr_src_t *s);

/*
 * Start asynchronous capture. Spawns an internal thread that feeds
 * the ring buffer. Returns 0 on success.
 */
int rtlsdr_src_start(rtlsdr_src_t *s);

/*
 * Stop capture and join the internal thread.
 */
void rtlsdr_src_stop(rtlsdr_src_t *s);

/*
 * Blocking read of exactly `n` raw IQ bytes into `buf`.
 * Returns 0 on success, -1 if capture is done/error.
 * n must be even (each sample = 1 I byte + 1 Q byte).
 */
int rtlsdr_src_read(rtlsdr_src_t *s, uint8_t *buf, size_t n);

/*
 * Set new center frequency while running (for scan/AFC).
 */
int rtlsdr_src_set_freq(rtlsdr_src_t *s, uint32_t freq);

/*
 * Set tuner gain while running.  gain_tenths is in 0.1 dB units.
 * Pass 0 to switch to automatic gain.
 */
int rtlsdr_src_set_gain(rtlsdr_src_t *s, int gain_tenths);

/*
 * Enable/disable the RTL2832U internal digital AGC.
 * This is separate from tuner gain — it adjusts the ADC dynamic range.
 * Returns 0 on success, -1 on failure.
 */
int rtlsdr_src_set_agc(rtlsdr_src_t *s, int enable);

/*
 * Get number of bytes dropped due to ring buffer overflow.
 */
size_t rtlsdr_src_overflow(rtlsdr_src_t *s);

#endif /* RTLSDR_SOURCE_H */
