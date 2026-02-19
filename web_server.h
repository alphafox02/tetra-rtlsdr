/*
 * tetra-rtlsdr - web server with spectrum waterfall and tuning controls
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "spectrum.h"
#include "rtlsdr_source.h"
#include "rtltcp_source.h"
#include "channel_mgr.h"

typedef struct web_server web_server_t;

/*
 * Create and start the HTTP server.
 *   port     - TCP port to listen on (e.g. 8080)
 *   spectrum - spectrum state shared with the demod thread
 *   src      - RTL-SDR source for re-tuning (NULL for file mode)
 *   target_freq  - initial TETRA target frequency in Hz
 *   freq_offset  - IF offset in Hz (e.g. 500000)
 *
 * Endpoints:
 *   GET  /              - Control panel + waterfall UI
 *   GET  /spectrum      - SSE stream of FFT data
 *   GET  /api/status    - JSON status (freq, gain, rssi, multi)
 *   POST /api/freq      - Set frequency (body: freq_hz=NNN)
 *   POST /api/gain      - Set gain (body: gain_db=NNN)
 *   POST /api/channel   - Add channel in multi-channel mode (body: freq_hz=NNN)
 *
 * Returns NULL on failure.
 */
web_server_t *web_server_create(int port, spectrum_t *spectrum,
                                 rtlsdr_src_t *src,
                                 uint32_t target_freq, float freq_offset);

void web_server_destroy(web_server_t *ws);

/* Update target frequency (called after re-tuning RTL-SDR) */
void web_server_set_freq(web_server_t *ws, uint32_t target_freq);

/* Get current target frequency */
uint32_t web_server_get_freq(web_server_t *ws);

/* Set channel manager for multi-channel mode.
 * When set, clicking the waterfall adds a channel instead of retuning. */
void web_server_set_channel_mgr(web_server_t *ws, channel_mgr_t *chmgr);

/* Set rtl_tcp source for remote retuning/gain control.
 * When set, freq/gain API calls use rtl_tcp instead of local RTL-SDR. */
void web_server_set_tcp_source(web_server_t *ws, rtltcp_src_t *tcp);

#endif /* WEB_SERVER_H */
