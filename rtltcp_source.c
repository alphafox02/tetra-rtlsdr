/*
 * tetra-rtlsdr - rtl_tcp network IQ source
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Connects to an rtl_tcp server and streams IQ data into a ring buffer.
 * Same pattern as rtlsdr_source.c — reader thread + blocking read().
 */

#include "rtltcp_source.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>

/* rtl_tcp command IDs */
#define RTLTCP_CMD_SET_FREQ      0x01
#define RTLTCP_CMD_SET_RATE      0x02
#define RTLTCP_CMD_SET_GAIN_MODE 0x03
#define RTLTCP_CMD_SET_GAIN      0x04
#define RTLTCP_CMD_SET_PPM       0x05
#define RTLTCP_CMD_SET_AGC       0x08

/* Ring buffer — same as rtlsdr_source.h */
#define RING_BYTES (1 << 23)   /* 8 MB */
#define RING_MASK  (RING_BYTES - 1)

typedef struct {
    uint8_t         data[RING_BYTES];
    volatile size_t head;
    volatile size_t tail;
    pthread_mutex_t mutex;
    pthread_cond_t  not_empty;
    int             done;
    size_t          overflow;
} rtltcp_ring_t;

struct rtltcp_src {
    int              sock_fd;
    rtltcp_ring_t    ring;
    pthread_t        thread;
    volatile int     running;
    uint32_t         sample_rate;
    uint32_t         freq;
};

/* ---- Ring buffer helpers (same logic as rtlsdr_source.c) ---- */

static void ring_init(rtltcp_ring_t *r) {
    r->head     = 0;
    r->tail     = 0;
    r->done     = 0;
    r->overflow = 0;
    pthread_mutex_init(&r->mutex, NULL);
    pthread_cond_init(&r->not_empty, NULL);
}

static void ring_destroy(rtltcp_ring_t *r) {
    pthread_cond_destroy(&r->not_empty);
    pthread_mutex_destroy(&r->mutex);
}

static inline size_t ring_avail(rtltcp_ring_t *r) {
    return (r->head - r->tail) & RING_MASK;
}

static inline size_t ring_free(rtltcp_ring_t *r) {
    return RING_BYTES - 1 - ring_avail(r);
}

static size_t ring_push(rtltcp_ring_t *r, const uint8_t *data, size_t len) {
    pthread_mutex_lock(&r->mutex);

    size_t space = ring_free(r);
    if (len > space) {
        r->overflow += len - space;
        len = space;
    }

    size_t head = r->head;
    size_t first = RING_BYTES - (head & RING_MASK);
    if (first > len) first = len;
    memcpy(r->data + (head & RING_MASK), data, first);
    if (len > first)
        memcpy(r->data, data + first, len - first);

    __sync_synchronize();
    r->head = (head + len) & RING_MASK;

    pthread_cond_signal(&r->not_empty);
    pthread_mutex_unlock(&r->mutex);
    return len;
}

/* ---- TCP helpers ---- */

/* Send a 5-byte rtl_tcp command: 1 byte cmd + 4 bytes big-endian param */
static int send_cmd(int fd, uint8_t cmd, uint32_t param) {
    uint8_t buf[5];
    buf[0] = cmd;
    buf[1] = (uint8_t)(param >> 24);
    buf[2] = (uint8_t)(param >> 16);
    buf[3] = (uint8_t)(param >> 8);
    buf[4] = (uint8_t)(param);

    size_t sent = 0;
    while (sent < 5) {
        ssize_t n = send(fd, buf + sent, 5 - sent, MSG_NOSIGNAL);
        if (n <= 0) return -1;
        sent += (size_t)n;
    }
    return 0;
}

/* Read exactly n bytes from socket (blocking) */
static int recv_exact(int fd, uint8_t *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = recv(fd, buf + got, n - got, 0);
        if (r <= 0) return -1;
        got += (size_t)r;
    }
    return 0;
}

/* ---- Reader thread ---- */

static void *reader_thread(void *arg) {
    rtltcp_src_t *s = (rtltcp_src_t *)arg;
    uint8_t buf[16384];

    while (s->running) {
        ssize_t n = recv(s->sock_fd, buf, sizeof(buf), 0);
        if (n <= 0) {
            if (s->running)
                fprintf(stderr, "[rtltcp] Connection lost.\n");
            break;
        }
        ring_push(&s->ring, buf, (size_t)n);
    }

    /* Signal consumers */
    pthread_mutex_lock(&s->ring.mutex);
    s->ring.done = 1;
    pthread_cond_broadcast(&s->ring.not_empty);
    pthread_mutex_unlock(&s->ring.mutex);
    return NULL;
}

/* ---- Public API ---- */

rtltcp_src_t *rtltcp_src_create(const char *host, int port,
                                 uint32_t freq, uint32_t sample_rate,
                                 int gain_tenths, int ppm) {
    rtltcp_src_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;

    ring_init(&s->ring);
    s->sample_rate = sample_rate;
    s->freq = freq;
    s->sock_fd = -1;

    /* Resolve host */
    struct addrinfo hints = {0}, *res = NULL;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", port);

    int rc = getaddrinfo(host, port_str, &hints, &res);
    if (rc != 0) {
        fprintf(stderr, "[rtltcp] Cannot resolve %s:%d: %s\n",
                host, port, gai_strerror(rc));
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }

    /* Connect */
    fprintf(stderr, "[rtltcp] Connecting to %s:%d ...\n", host, port);
    s->sock_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s->sock_fd < 0) {
        fprintf(stderr, "[rtltcp] socket() failed: %s\n", strerror(errno));
        freeaddrinfo(res);
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }

    if (connect(s->sock_fd, res->ai_addr, res->ai_addrlen) < 0) {
        fprintf(stderr, "[rtltcp] connect() failed: %s\n", strerror(errno));
        close(s->sock_fd);
        freeaddrinfo(res);
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }
    freeaddrinfo(res);

    /* Disable Nagle for low-latency command sends */
    int one = 1;
    setsockopt(s->sock_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    /* Increase TCP receive buffer for high-throughput IQ stream */
    int rcvbuf = 1 << 20;  /* 1 MB */
    setsockopt(s->sock_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    /* Read 12-byte dongle info header */
    uint8_t hdr[12];
    if (recv_exact(s->sock_fd, hdr, 12) < 0) {
        fprintf(stderr, "[rtltcp] Failed to read server header.\n");
        close(s->sock_fd);
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }

    if (memcmp(hdr, "RTL0", 4) != 0) {
        fprintf(stderr, "[rtltcp] Invalid server header (not rtl_tcp?).\n");
        close(s->sock_fd);
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }

    uint32_t tuner_type = ((uint32_t)hdr[4] << 24) | ((uint32_t)hdr[5] << 16) |
                          ((uint32_t)hdr[6] << 8) | hdr[7];
    uint32_t gain_count = ((uint32_t)hdr[8] << 24) | ((uint32_t)hdr[9] << 16) |
                          ((uint32_t)hdr[10] << 8) | hdr[11];

    const char *tuner_names[] = {
        "unknown", "E4000", "FC0012", "FC0013", "FC2580", "R820T", "R828D"
    };
    const char *tname = (tuner_type < 7) ? tuner_names[tuner_type] : "unknown";
    fprintf(stderr, "[rtltcp] Connected. Tuner: %s, gain steps: %u\n",
            tname, gain_count);

    /* Send initial configuration */
    if (send_cmd(s->sock_fd, RTLTCP_CMD_SET_RATE, sample_rate) < 0 ||
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_FREQ, freq) < 0) {
        fprintf(stderr, "[rtltcp] Failed to send initial config.\n");
        close(s->sock_fd);
        ring_destroy(&s->ring);
        free(s);
        return NULL;
    }

    fprintf(stderr, "[rtltcp] Sample rate: %u S/s\n", sample_rate);
    fprintf(stderr, "[rtltcp] Frequency: %.6f MHz\n", freq / 1e6);

    /* Gain */
    if (gain_tenths == 0) {
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_GAIN_MODE, 0);
        fprintf(stderr, "[rtltcp] Gain: auto\n");
    } else {
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_GAIN_MODE, 1);
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_GAIN, (uint32_t)gain_tenths);
        fprintf(stderr, "[rtltcp] Gain: %.1f dB\n", gain_tenths / 10.0);
    }

    /* PPM correction */
    if (ppm != 0) {
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_PPM, (uint32_t)ppm);
        fprintf(stderr, "[rtltcp] PPM correction: %d\n", ppm);
    }

    return s;
}

void rtltcp_src_destroy(rtltcp_src_t *s) {
    if (!s) return;
    if (s->sock_fd >= 0) close(s->sock_fd);
    ring_destroy(&s->ring);
    free(s);
}

int rtltcp_src_start(rtltcp_src_t *s) {
    s->running = 1;
    if (pthread_create(&s->thread, NULL, reader_thread, s) != 0) {
        fprintf(stderr, "[rtltcp] Failed to create reader thread.\n");
        s->running = 0;
        return -1;
    }
    return 0;
}

void rtltcp_src_stop(rtltcp_src_t *s) {
    s->running = 0;
    /* Shutdown socket to unblock recv() in reader thread */
    if (s->sock_fd >= 0)
        shutdown(s->sock_fd, SHUT_RDWR);
    pthread_join(s->thread, NULL);
}

int rtltcp_src_read(rtltcp_src_t *s, uint8_t *buf, size_t n) {
    rtltcp_ring_t *r = &s->ring;
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

int rtltcp_src_set_freq(rtltcp_src_t *s, uint32_t freq) {
    s->freq = freq;
    return send_cmd(s->sock_fd, RTLTCP_CMD_SET_FREQ, freq);
}

int rtltcp_src_set_gain(rtltcp_src_t *s, int gain_tenths) {
    if (gain_tenths == 0) {
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_GAIN_MODE, 0);
    } else {
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_GAIN_MODE, 1);
        send_cmd(s->sock_fd, RTLTCP_CMD_SET_GAIN, (uint32_t)gain_tenths);
    }
    return 0;
}

int rtltcp_src_set_agc(rtltcp_src_t *s, int enable) {
    int rc = send_cmd(s->sock_fd, RTLTCP_CMD_SET_AGC, enable ? 1 : 0);
    if (rc == 0)
        fprintf(stderr, "[rtltcp] RTL2832U AGC: %s\n",
                enable ? "enabled" : "disabled");
    else
        fprintf(stderr, "[rtltcp] Failed to send AGC command.\n");
    return rc;
}

size_t rtltcp_src_overflow(rtltcp_src_t *s) {
    return s->ring.overflow;
}
