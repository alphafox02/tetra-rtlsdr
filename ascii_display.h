/*
 * tetra-rtlsdr - ASCII spectrum display
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Renders a live FFT spectrum on stderr using Unicode block characters
 * and ANSI escape codes.  No ncurses dependency.
 * Inspired by retrogram-rtlsdr / retrogram-soapysdr (r4d10n)
 * and UHD rx_ascii_art_dft.
 *
 * Usage:
 *   ascii_display_t *disp = ascii_display_create(spectrum, center, rate);
 *   // in main loop:
 *   ascii_cmd_t cmd = ascii_display_update(disp, rssi, fll_hz, sync_count);
 *   if (cmd == ASCII_CMD_QUIT) break;
 *   // on exit:
 *   ascii_display_destroy(disp);
 *
 * stdout is never touched (reserved for demodulated bits).
 * If stderr is not a TTY the display is silently disabled.
 */

#ifndef ASCII_DISPLAY_H
#define ASCII_DISPLAY_H

#include "spectrum.h"
#include "channel_mgr.h"

typedef struct ascii_display ascii_display_t;

/* Commands returned by ascii_display_update() for main.c to act on. */
typedef enum {
    ASCII_CMD_NONE = 0,
    ASCII_CMD_QUIT,         /* q pressed */
    ASCII_CMD_GAIN_UP,      /* G / + */
    ASCII_CMD_GAIN_DOWN,    /* g / - */
} ascii_cmd_t;

/*
 * Create the ASCII spectrum display.
 *
 *   spectrum    - shared spectrum_t object (provides FFT PSD data)
 *   center_freq - RTL-SDR center frequency in Hz (for axis labels)
 *   sample_rate - RTL-SDR sample rate in Hz (for axis labels)
 *
 * Returns NULL if stderr is not a TTY or on allocation failure.
 */
ascii_display_t *ascii_display_create(spectrum_t *spectrum,
                                       double center_freq,
                                       double sample_rate);

/*
 * Update the display.  Call this periodically from the main loop.
 * Internally throttles to the configured refresh rate (~5 Hz default).
 * Reads keyboard input non-blocking and returns commands for main.c.
 *
 *   disp       - display context (NULL is safe, returns ASCII_CMD_NONE)
 *   rssi_db    - current signal level from AGC (dB)
 *   fll_freq_hz - FLL frequency offset in Hz (carrier recovery error)
 *   sync_count - cumulative TETRA sync bursts detected
 */
ascii_cmd_t ascii_display_update(ascii_display_t *disp, float rssi_db,
                                  float fll_freq_hz, unsigned int sync_count);

/*
 * Set channel manager for multi-channel status display.
 * When set, the status line shows per-channel freq+RSSI and scan progress.
 */
void ascii_display_set_channel_mgr(ascii_display_t *disp, channel_mgr_t *chmgr);

/*
 * Destroy the display, restore the terminal, and close /dev/tty.
 */
void ascii_display_destroy(ascii_display_t *disp);

#endif /* ASCII_DISPLAY_H */
