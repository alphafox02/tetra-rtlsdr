/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "rtlsdr_source.h"

#include <rtl-sdr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>

struct rtlsdr_src {
    rtlsdr_dev_t    *dev;
    ring_buf_t       ring;
    pthread_t        thread;
    volatile int     running;
    uint32_t         sample_rate;
    uint32_t         freq;
};

/* ---- Ring buffer helpers ---- */

static void ring_init(ring_buf_t *r) {
    r->head     = 0;
    r->tail     = 0;
    r->done     = 0;
    r->overflow = 0;
    pthread_mutex_init(&r->mutex, NULL);
    pthread_cond_init(&r->not_empty, NULL);
}

static void ring_destroy(ring_buf_t *r) {
    pthread_cond_destroy(&r->not_empty);
    pthread_mutex_destroy(&r->mutex);
}

/* Returns bytes available to read */
static inline size_t ring_avail(ring_buf_t *r) {
    return (r->head - r->tail) & RING_MASK;
}

/* Returns free space */
static inline size_t ring_free(ring_buf_t *r) {
    return RING_BYTES - 1 - ring_avail(r);
}

/* Push up to len bytes; drops if ring full. Returns bytes written. */
static size_t ring_push(ring_buf_t *r, const uint8_t *data, size_t len) {
    pthread_mutex_lock(&r->mutex);

    size_t free = ring_free(r);
    if (len > free) {
        r->overflow += len - free;
        len = free;
    }

    size_t head = r->head;
    size_t first = RING_BYTES - (head & RING_MASK);
    if (first > len) first = len;
    memcpy(r->data + (head & RING_MASK), data, first);
    if (len > first)
        memcpy(r->data, data + first, len - first);

    /* barrier: ensure data is visible before updating head */
    __sync_synchronize();
    r->head = (head + len) & RING_MASK;

    pthread_cond_signal(&r->not_empty);
    pthread_mutex_unlock(&r->mutex);
    return len;
}

/* Pull exactly `n` bytes (blocking). Returns 0 ok, -1 done/error. */
int rtlsdr_src_read(rtlsdr_src_t *s, uint8_t *buf, size_t n) {
    ring_buf_t *r = &s->ring;
    size_t got = 0;

    while (got < n) {
        pthread_mutex_lock(&r->mutex);
        while (ring_avail(r) == 0 && !r->done)
            pthread_cond_wait(&r->not_empty, &r->mutex);

        if (ring_avail(r) == 0 && r->done) {
            pthread_mutex_unlock(&r->mutex);
            return -1;
        }

        size_t want = n - got;
        size_t avail = ring_avail(r);
        if (want > avail) want = avail;

        size_t tail = r->tail;
        size_t first = RING_BYTES - (tail & RING_MASK);
        if (first > want) first = want;
        memcpy(buf + got, r->data + (tail & RING_MASK), first);
        if (want > first)
            memcpy(buf + got + first, r->data, want - first);
        r->tail = (tail + want) & RING_MASK;
        got += want;

        pthread_mutex_unlock(&r->mutex);
    }
    return 0;
}

/* ---- RTL-SDR async callback ---- */

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx) {
    rtlsdr_src_t *s = (rtlsdr_src_t *)ctx;
    if (!s->running) return;
    ring_push(&s->ring, buf, len);
}

static void *capture_thread(void *arg) {
    rtlsdr_src_t *s = (rtlsdr_src_t *)arg;
    /* buf_num=0 → use default (15); buf_len=0 → use default (16384 bytes) */
    rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, 0);

    /* Signal consumers that we're done */
    pthread_mutex_lock(&s->ring.mutex);
    s->ring.done = 1;
    pthread_cond_broadcast(&s->ring.not_empty);
    pthread_mutex_unlock(&s->ring.mutex);
    return NULL;
}

/* ---- Public API ---- */

rtlsdr_src_t *rtlsdr_src_create(uint32_t freq, uint32_t sample_rate,
                                 int gain_tenths, int device_idx, int ppm) {
    rtlsdr_src_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;

    ring_init(&s->ring);
    s->sample_rate = sample_rate;
    s->freq        = freq;

    int n = rtlsdr_get_device_count();
    if (n == 0) {
        fprintf(stderr, "[rtlsdr] No RTL-SDR devices found.\n");
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }
    if (device_idx >= n) {
        fprintf(stderr, "[rtlsdr] Device index %d out of range (have %d).\n",
                device_idx, n);
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }

    const char *name = rtlsdr_get_device_name(device_idx);
    fprintf(stderr, "[rtlsdr] Opening device %d: %s\n", device_idx, name ? name : "?");

    if (rtlsdr_open(&s->dev, device_idx) < 0) {
        fprintf(stderr, "[rtlsdr] Failed to open device.\n");
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }

    /* PPM correction */
    if (ppm != 0) {
        rtlsdr_set_freq_correction(s->dev, ppm);
        fprintf(stderr, "[rtlsdr] PPM correction: %d\n", ppm);
    }

    /* Gain */
    if (gain_tenths == 0) {
        rtlsdr_set_tuner_gain_mode(s->dev, 0);  /* auto */
        fprintf(stderr, "[rtlsdr] Gain: auto\n");
    } else {
        rtlsdr_set_tuner_gain_mode(s->dev, 1);  /* manual */
        rtlsdr_set_tuner_gain(s->dev, gain_tenths);
        fprintf(stderr, "[rtlsdr] Gain: %.1f dB\n", gain_tenths / 10.0);
    }

    /* Sample rate */
    if (rtlsdr_set_sample_rate(s->dev, sample_rate) < 0) {
        fprintf(stderr, "[rtlsdr] Failed to set sample rate %u.\n", sample_rate);
        rtlsdr_close(s->dev);
        free(s);
        return NULL;
    }
    fprintf(stderr, "[rtlsdr] Sample rate: %u S/s\n", sample_rate);

    /* Center frequency */
    if (rtlsdr_set_center_freq(s->dev, freq) < 0) {
        fprintf(stderr, "[rtlsdr] Failed to set frequency %u Hz.\n", freq);
        rtlsdr_close(s->dev);
        free(s);
        return NULL;
    }
    fprintf(stderr, "[rtlsdr] Frequency: %.6f MHz\n", freq / 1e6);

    /* Reset buffer */
    rtlsdr_reset_buffer(s->dev);

    return s;
}

void rtlsdr_src_destroy(rtlsdr_src_t *s) {
    if (!s) return;
    if (s->dev) rtlsdr_close(s->dev);
    ring_destroy(&s->ring);
    free(s);
}

int rtlsdr_src_start(rtlsdr_src_t *s) {
    s->running = 1;
    if (pthread_create(&s->thread, NULL, capture_thread, s) != 0) {
        fprintf(stderr, "[rtlsdr] Failed to create capture thread.\n");
        s->running = 0;
        return -1;
    }
    return 0;
}

void rtlsdr_src_stop(rtlsdr_src_t *s) {
    s->running = 0;
    rtlsdr_cancel_async(s->dev);
    pthread_join(s->thread, NULL);
}

int rtlsdr_src_set_freq(rtlsdr_src_t *s, uint32_t freq) {
    s->freq = freq;
    return rtlsdr_set_center_freq(s->dev, freq);
}

int rtlsdr_src_set_gain(rtlsdr_src_t *s, int gain_tenths) {
    if (gain_tenths == 0) {
        rtlsdr_set_tuner_gain_mode(s->dev, 0);
    } else {
        rtlsdr_set_tuner_gain_mode(s->dev, 1);
        rtlsdr_set_tuner_gain(s->dev, gain_tenths);
    }
    return 0;
}

int rtlsdr_src_set_agc(rtlsdr_src_t *s, int enable) {
    int rc = rtlsdr_set_agc_mode(s->dev, enable ? 1 : 0);
    if (rc == 0)
        fprintf(stderr, "[rtlsdr] RTL2832U AGC: %s\n",
                enable ? "enabled" : "disabled");
    else
        fprintf(stderr, "[rtlsdr] Failed to set RTL2832U AGC mode.\n");
    return rc;
}

size_t rtlsdr_src_overflow(rtlsdr_src_t *s) {
    return s->ring.overflow;
}
