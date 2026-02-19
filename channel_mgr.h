/*
 * tetra-rtlsdr - multi-channel demodulation manager
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Manages multiple simultaneous TETRA demod channels from one RTL-SDR capture.
 * Auto-discovers channels by scanning the capture bandwidth for TETRA SYNC
 * training sequences, then spawns a demod chain + tetra-rx for each.
 *
 * The in-band scan is non-blocking: it piggybacks on the main processing
 * loop, examining one frequency step per dwell period while all active
 * channels continue to demodulate in real time.
 */

#ifndef CHANNEL_MGR_H
#define CHANNEL_MGR_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

#include "spectrum.h"

#define CHMGR_MAX_CHANNELS  6

typedef struct channel_mgr channel_mgr_t;

/*
 * Create a channel manager.
 *
 *   sample_rate   - RTL-SDR sample rate (e.g. 1800000)
 *   channel_rate  - per-channel rate after decimation (36000)
 *   tetra_rx_path - absolute path to tetra-rx binary
 *   center_freq   - RTL-SDR center frequency in Hz
 */
channel_mgr_t *channel_mgr_create(uint32_t sample_rate,
                                    uint32_t channel_rate,
                                    const char *tetra_rx_path,
                                    uint32_t center_freq);

/*
 * Add a channel at the given TETRA frequency.
 * Creates FIFO, forks tetra-rx, creates demod instance.
 * Returns channel index (0-based) on success, -1 on failure.
 * Silently rejects duplicates and out-of-band frequencies.
 */
int channel_mgr_add(channel_mgr_t *mgr, uint32_t freq);

/*
 * Attach spectrum for waterfall display.
 * Must be called after at least one channel is added.
 * Attaches the spectrum to channel 0's demod so the waterfall
 * stays live during scanning and normal operation.
 */
void channel_mgr_set_spectrum(channel_mgr_t *mgr, spectrum_t *spectrum);

/*
 * Start a non-blocking in-band scan.  The scan runs incrementally
 * inside channel_mgr_process() — each call processes one IQ block
 * for the current scan step.  When a dwell period completes,
 * the step is evaluated and the scan advances.
 *
 *   dwell_ms  - dwell time per 25 kHz step in milliseconds
 *
 * Active channels continue to demodulate during the scan.
 * Discovered channels are added automatically.
 */
void channel_mgr_start_scan(channel_mgr_t *mgr, int dwell_ms);

/*
 * Check if a background scan is in progress.
 */
int channel_mgr_scan_active(channel_mgr_t *mgr);

/*
 * Process a block of raw IQ bytes through all active demod channels.
 * Also advances the background scan if one is running.
 * Returns 0 on success, -1 on error.
 */
int channel_mgr_process(channel_mgr_t *mgr,
                          const uint8_t *iq_buf, size_t len);

/*
 * Get the number of active channels.
 */
int channel_mgr_n_channels(channel_mgr_t *mgr);

/*
 * Get RSSI for channel idx (dB).
 */
float channel_mgr_get_rssi(channel_mgr_t *mgr, int idx);

/*
 * Get TETRA target frequency for channel idx (Hz).
 */
uint32_t channel_mgr_get_freq(channel_mgr_t *mgr, int idx);

/*
 * Get FLL carrier offset in Hz for channel idx.
 */
float channel_mgr_get_fll_hz(channel_mgr_t *mgr, int idx);

/*
 * Get RXID for channel idx (1-based).
 */
int channel_mgr_get_rxid(channel_mgr_t *mgr, int idx);

/*
 * Get scan progress: current step and total steps.
 * Returns 0 if no scan active.
 */
int channel_mgr_scan_progress(channel_mgr_t *mgr, int *step, int *total);

/*
 * Destroy: kills tetra-rx children, closes FIFOs, frees resources.
 */
void channel_mgr_destroy(channel_mgr_t *mgr);

#endif /* CHANNEL_MGR_H */
