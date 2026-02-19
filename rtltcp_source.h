/*
 * tetra-rtlsdr - rtl_tcp network IQ source
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Connects to an rtl_tcp server for remote RTL-SDR streaming.
 * Same ring-buffer interface as rtlsdr_source for drop-in use.
 *
 * rtl_tcp protocol:
 *   Server sends 12-byte header: "RTL0" + tuner_type(4) + gain_count(4)
 *   Client sends 5-byte commands: cmd(1) + param(4, big-endian)
 *   Server streams continuous uint8 IQ (same as local RTL-SDR)
 *
 * Start a server on a remote machine with:
 *   rtl_tcp -a 0.0.0.0 -p 1234 -f 392640000 -s 1800000
 */

#ifndef RTLTCP_SOURCE_H
#define RTLTCP_SOURCE_H

#include <stdint.h>
#include <stddef.h>

typedef struct rtltcp_src rtltcp_src_t;

/*
 * Create rtl_tcp source.
 *   host        - server hostname or IP (e.g. "192.168.1.50")
 *   port        - server TCP port (e.g. 1234)
 *   freq        - center frequency in Hz
 *   sample_rate - capture rate in S/s (e.g. 1800000)
 *   gain_tenths - gain in 0.1 dB units, 0 = auto-gain
 *   ppm         - crystal error correction in ppm
 *
 * Connects to the server and sends initial configuration.
 * Returns NULL on failure.
 */
rtltcp_src_t *rtltcp_src_create(const char *host, int port,
                                 uint32_t freq, uint32_t sample_rate,
                                 int gain_tenths, int ppm);

void rtltcp_src_destroy(rtltcp_src_t *s);

/*
 * Start the reader thread that fills the ring buffer from TCP.
 * Returns 0 on success.
 */
int rtltcp_src_start(rtltcp_src_t *s);

/*
 * Stop the reader thread.
 */
void rtltcp_src_stop(rtltcp_src_t *s);

/*
 * Blocking read of exactly `n` raw IQ bytes into `buf`.
 * Returns 0 on success, -1 if connection lost/stopped.
 */
int rtltcp_src_read(rtltcp_src_t *s, uint8_t *buf, size_t n);

/*
 * Set new center frequency (sends command to server).
 */
int rtltcp_src_set_freq(rtltcp_src_t *s, uint32_t freq);

/*
 * Set tuner gain. gain_tenths in 0.1 dB units, 0 = auto.
 */
int rtltcp_src_set_gain(rtltcp_src_t *s, int gain_tenths);

/*
 * Enable/disable RTL2832U digital AGC on the remote dongle.
 */
int rtltcp_src_set_agc(rtltcp_src_t *s, int enable);

/*
 * Get number of bytes dropped due to ring buffer overflow.
 */
size_t rtltcp_src_overflow(rtltcp_src_t *s);

#endif /* RTLTCP_SOURCE_H */
