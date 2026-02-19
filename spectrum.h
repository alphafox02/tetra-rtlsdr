/*
 * tetra-rtlsdr - spectrum periodogram (shared between demod and web server)
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <stdint.h>
#include <stddef.h>
#include <complex.h>
#include <pthread.h>

#define SPECTRUM_NFFT   1024    /* FFT size for waterfall */

/*
 * Spectrum state: fed by the demod thread, read by the web server thread.
 * Uses a double-buffer (ping-pong) so the web server always gets a
 * complete, consistent snapshot.
 */
typedef struct {
    /* Ping-pong PSD buffers (dB, fftshift applied) */
    float           psd[2][SPECTRUM_NFFT];
    int             active;         /* which buffer the demod is writing to */
    pthread_mutex_t mutex;
    pthread_cond_t  updated;        /* signalled each time a new PSD is ready */
    uint64_t        seq;            /* incremented on each update */
    int             shutdown;       /* set to 1 to unblock spectrum_get */

    /* Metadata for the web client */
    double          center_hz;
    double          rate_hz;
    float           rssi_db;

    /* Rate control for spectrum_push (only touched by demod thread) */
    unsigned int    push_count;

    /* liquid-dsp periodogram (internal, only touched by demod thread) */
    void           *sg;             /* spgramcf* — opaque to callers */
} spectrum_t;

/*
 * Create spectrum state.
 *   center_hz - RTL-SDR center frequency
 *   rate_hz   - RTL-SDR sample rate
 * Returns NULL on failure.
 */
spectrum_t *spectrum_create(double center_hz, double rate_hz);

void spectrum_destroy(spectrum_t *s);

/*
 * Feed raw complex float samples (already converted from uint8 IQ).
 * Called from the demod thread. Thread-safe: updates internal spgramcf,
 * and periodically commits a new PSD snapshot to the shared buffer.
 *   samples     - float complex buffer
 *   n           - number of samples
 *   rssi_db     - current AGC RSSI for display
 *   update_every - commit a new snapshot every this many calls (rate control)
 */
void spectrum_push(spectrum_t *s, const float complex *samples, size_t n,
                   float rssi_db, unsigned int update_every);

/*
 * Get a copy of the latest PSD snapshot for the web server.
 * Blocks until a new frame is available (seq > last_seq).
 *   psd_out  - caller-provided buffer of SPECTRUM_NFFT floats
 *   last_seq - pass 0 first call; updated to current seq on return
 * Returns 0 on success.
 */
int spectrum_get(spectrum_t *s, float *psd_out, uint64_t *last_seq);

#endif /* SPECTRUM_H */
