/*
 * tetra-rtlsdr - spectrum periodogram
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "spectrum.h"

#include <liquid/liquid.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

spectrum_t *spectrum_create(double center_hz, double rate_hz) {
    spectrum_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;

    s->center_hz = center_hz;
    s->rate_hz   = rate_hz;
    s->active    = 0;
    s->seq       = 0;
    s->rssi_db   = -60.0f;

    pthread_mutex_init(&s->mutex, NULL);
    pthread_cond_init(&s->updated, NULL);

    /* spgramcf: NFFT=1024, Hann window, window_len=NFFT/2, delay=NFFT/2 */
    s->sg = spgramcf_create(SPECTRUM_NFFT,
                             LIQUID_WINDOW_HANN,
                             SPECTRUM_NFFT / 2,
                             SPECTRUM_NFFT / 2);
    if (!s->sg) {
        pthread_cond_destroy(&s->updated);
        pthread_mutex_destroy(&s->mutex);
        free(s);
        return NULL;
    }

    return s;
}

void spectrum_destroy(spectrum_t *s) {
    if (!s) return;

    /* Signal any thread blocked in spectrum_get() to wake up and exit */
    pthread_mutex_lock(&s->mutex);
    s->shutdown = 1;
    pthread_cond_broadcast(&s->updated);
    pthread_mutex_unlock(&s->mutex);

    if (s->sg) spgramcf_destroy((spgramcf)s->sg);
    pthread_cond_destroy(&s->updated);
    pthread_mutex_destroy(&s->mutex);
    free(s);
}

void spectrum_push(spectrum_t *s, const float complex *samples, size_t n,
                   float rssi_db, unsigned int update_every) {
    /* Feed samples into the periodogram (this is fast, no locking needed
     * because spgramcf is only touched from the demod thread) */
    spgramcf_write((spgramcf)s->sg, (liquid_float_complex *)samples, (unsigned)n);

    s->rssi_db = rssi_db;
    s->push_count++;

    if (s->push_count < update_every) return;
    s->push_count = 0;

    /* Get the averaged PSD into the inactive buffer */
    int next = 1 - s->active;
    spgramcf_get_psd((spgramcf)s->sg, s->psd[next]);
    spgramcf_reset((spgramcf)s->sg);

    /* Commit: swap active buffer and notify web server */
    pthread_mutex_lock(&s->mutex);
    s->active = next;
    s->seq++;
    pthread_cond_signal(&s->updated);
    pthread_mutex_unlock(&s->mutex);
}

int spectrum_get(spectrum_t *s, float *psd_out, uint64_t *last_seq) {
    pthread_mutex_lock(&s->mutex);

    while (s->seq == *last_seq && !s->shutdown)
        pthread_cond_wait(&s->updated, &s->mutex);

    if (s->shutdown) {
        pthread_mutex_unlock(&s->mutex);
        return -1;
    }

    /* Copy the currently-active (just-committed) buffer */
    memcpy(psd_out, s->psd[s->active], SPECTRUM_NFFT * sizeof(float));
    *last_seq = s->seq;

    pthread_mutex_unlock(&s->mutex);
    return 0;
}
