# tetra-rtlsdr Development Notes

## Band-Edge FLL Carrier Recovery — Debug History

The band-edge FLL (Frequency-Locked Loop) was the most challenging part of
the demodulator to get right. This document records the bugs found and
fixes applied, as a reference for anyone working on similar DSP code.

### What the FLL does

TETRA uses pi/4-DQPSK modulation at 18,000 symbols/second with RRC pulse
shaping (alpha=0.35). The FLL recovers the carrier frequency offset before
symbol timing recovery. It works by comparing spectral power at the upper
and lower edges of the RRC-shaped signal — if the carrier is off-frequency,
one edge will have more power than the other.

Reference implementation: GnuRadio's `digital.fll_band_edge_cc`.

### Bug 1: Loop gain formula was completely wrong

**Symptom:** FLL frequency (NCO) immediately diverged to +-2pi (the clamp
limits), destroying the signal. Zero SYNC detections.

**Root cause:** Used an ad-hoc formula for the loop filter gains:
```c
// WRONG:
beta = 2*PI * 4 * BW / sps;   // = 0.395 (27x too large!)
alpha = 0;                      // missing entirely
```

**Fix:** Use GnuRadio's `control_loop::update_gains()` formula:
```c
damping = sqrt(2) / 2;
denom = 1 + 2*damping*bw + bw^2;
alpha = 4*damping*bw / denom;    // proportional (phase)  ≈ 0.086
beta  = 4*bw^2 / denom;          // integral (frequency)  ≈ 0.0038
```

### Bug 2: Error signal sign was reversed

**Symptom:** FLL pushed the frequency in the wrong direction, causing
divergence even with correct loop gains.

**Root cause:** Used `fu_pwr - fl_pwr` (upper minus lower) instead of
GnuRadio's convention `0.5 * (fl_pwr - fu_pwr)` (lower minus upper).

**Fix:**
```c
float error = 0.5f * (fl_pwr - fu_pwr);
```

### Bug 3: Missing alpha*error term in phase update

**Symptom:** Degraded tracking performance. The proportional path (alpha)
provides fast response to frequency changes.

**Root cause:** Phase update was `phase += freq` instead of the full
2nd-order PLL: `phase += freq + alpha*error`.

**Fix:**
```c
d->fll_freq  += d->fll_beta * error;
d->fll_phase += d->fll_freq + d->fll_alpha * error;
```

### Bug 4: Filter taps were time-reversed (double reversal)

**Symptom:** Subtle degradation in FLL error signal quality.

**Root cause:** The band-edge filter taps were explicitly time-reversed
before passing to `firfilt_cccf_create()`. But liquid-dsp's `firfilt`
already performs convolution (which inherently time-reverses the kernel).
Explicit reversal + convolution = correlation (wrong).

**Fix:** Remove the explicit time-reversal. Pass taps directly to
`firfilt_cccf_create()`.

### Bug 5 (Critical): AGC was AFTER the FLL instead of BEFORE

**Symptom:** Even after fixing bugs 1-4, the FLL diverged at the standard
bandwidth (pi/100). A bandwidth sweep showed:

| FLL Bandwidth | SYNC detections |
|---------------|-----------------|
| pi/100        | 0 (diverged)    |
| pi/200        | 0               |
| pi/500        | 1               |
| pi/800        | 40              |
| pi/1000       | 52              |
| FLL disabled  | 229             |

The FLL only stopped hurting performance at extremely narrow bandwidths
where the effective gain was low enough to not diverge.

**Root cause:** The FLL error signal is `0.5 * (|lower|^2 - |upper|^2)`.
This scales with signal power squared. Without amplitude normalization
before the FLL, the effective loop gain depends on the signal level:

- Strong signal: error ∝ A^2, effective gain = loop_gain * A^2 → too high → diverges
- Weak signal: error ∝ A^2, effective gain = loop_gain * A^2 → too low → can't track

GnuRadio's simdemod3_telive.py (the reference chain) places
`analog.feedforward_agc_cc(8, 1)` BEFORE `digital.fll_band_edge_cc`:

```
file_source → feedforward_agc → fll_band_edge → pfb_clock_sync → ...
```

Our chain had AGC after the FLL:
```
decim → FLL → AGC → symsync   (WRONG)
```

**Fix:** Move AGC before FLL:
```
decim → AGC → FLL → symsync   (CORRECT)
```

### Bug 6: Feedback AGC vs Feedforward AGC

After fixing the AGC order, we initially used liquid-dsp's `agc_crcf`
(a feedback/PLL-style AGC with bandwidth 1e-3). This worked but is
different from GnuRadio's `feedforward_agc_cc(8, 1.0)`.

The feedforward AGC computes a running RMS over 8 samples and divides
by it — instant normalization, no adaptation delay. We implemented this
as a simple ring buffer of |x|^2 values with a running sum.

In practice, both AGC types gave nearly identical results on our test
signal (2,207 vs 2,208 SYNC). The key was the ordering, not the AGC type.

---

## Verification Results

Test signal: synthetic pi/4-DQPSK IQ generated from known-good bits
(originally demodulated by GnuRadio's simdemod3 from a live TETRA
network). ~53 seconds of TETRA data at 1.8 MSps with 500 kHz IF offset.

| Configuration | SYNC | CRC OK | Notes |
|--------------|------|--------|-------|
| Known-good bits → tetra-rx (baseline) | 2,224 | 115 | First 2M bits from simdemod3 |
| FLL disabled (AGC + symsync only) | 229 | 117 | No carrier recovery |
| FLL at pi/100, AGC after FLL | 0 | 0 | FLL diverges immediately |
| FLL at pi/800, AGC after FLL | 40 | — | Barely functional |
| **FLL at pi/100, AGC before FLL** | **2,207** | **117** | **99.2% of GnuRadio** |
| FLL at pi/100, feedforward AGC before FLL | 2,207 | 117 | Same performance |

The final configuration matches GnuRadio's simdemod3 performance to within
0.8% on SYNC detection rate and actually has slightly better CRC success.

---

## Signal Chain (Final)

```
raw uint8 IQ (1.8 MSps)
  → float complex conversion
  → NCO IF shift (nco_crcf, fixed 500 kHz, avoids DC spike)
  → Kaiser LPF + 50× decimation (firdecim_crcf) → 36 kSps
  → Feedforward AGC (8-sample window, unity reference)
  → Band-edge FLL (45-tap filters, pi/100 bandwidth, 2nd-order PLL)
  → Polyphase RRC timing recovery (symsync_crcf, sps=2, alpha=0.35)
  → pi/4-DQPSK differential decode
  → 1 byte per bit (0x00/0x01) to stdout
```

## Key Lessons

1. **AGC must precede FLL** — the power-based error signal scales with A^2
2. **Use the exact GnuRadio loop gain formulas** — ad-hoc approximations fail
3. **liquid-dsp firfilt does convolution** — don't manually reverse filter taps
4. **Test with synthetic IQ** — generate known-good modulated signals to isolate
   demod bugs from RF/antenna issues
5. **Compare against a known-good chain** — having simdemod3 bits as ground truth
   made it possible to quantify our demod's accuracy

---

## Multi-Channel Architecture

### Design

The multi-channel system demodulates up to 6 TETRA channels simultaneously
from one RTL-SDR capture. Each channel is fully independent: own NCO offset,
decimation filter, AGC, FLL, symsync, and DQPSK state. The main loop reads
one block of raw IQ and passes the same block to all demod instances — each
extracts its channel via its own NCO frequency.

```
RTL-SDR (1.8 MHz)
  └─ ring buffer → main loop
       ├─ demod[0] (NCO +500 kHz) → FIFO /tmp/tetra_PID_ch0 → tetra-rx (RXID=1)
       ├─ demod[1] (NCO -225 kHz) → FIFO /tmp/tetra_PID_ch1 → tetra-rx (RXID=2)
       └─ demod[2] (NCO +250 kHz) → FIFO /tmp/tetra_PID_ch2 → tetra-rx (RXID=3)
```

### Why FIFOs + fork (not threads)

We chose named FIFOs and forked `tetra-rx` processes instead of linking
`tetra-rx` as a library or using threads:

- **Zero modifications to tetra-rx or telive** — they're SQ5BPF's code
- tetra-rx reads bits from stdin, sends UDP to telive. FIFOs map directly
  to stdin via `dup2()`.
- Each tetra-rx process gets a unique `TETRA_HACK_RXID` env var — telive
  already tracks multiple receivers by RXID
- Process isolation: if one tetra-rx crashes, the others continue.
  The channel manager detects dead children and respawns them (up to 5
  attempts per 60-second window).

### In-Band Scan (Non-Blocking)

When `-T` is specified, a background scan runs inside `channel_mgr_process()`.
It's non-blocking: all active channels keep demodulating while the scan
sweeps remaining frequencies.

How it works:
1. Calculate the scan range: center ± 80% of sample_rate/2
2. Step through in 25 kHz increments (TETRA channel spacing)
3. For each step, create a temporary demod with a scan buffer
4. Dwell for 1500ms, looking for TETRA SYNC training sequences (38-bit
   pattern match) and Normal Training Sequences (22-bit NTS1)
5. If matches found → `channel_mgr_add()` creates a permanent channel
6. Active channels continue demodulating during the entire scan

The scan demod is recycled per step — `dqpsk_demod_reset()` clears FLL,
symsync, and AGC state, then sets a new NCO frequency.

### DC Spike Avoidance

RTL-SDR devices have a DC spike at the center frequency. In single-channel
mode, we avoid this with a 500 kHz IF offset. In multi-channel mode, the
center frequency is chosen to place the user's channel at +500 kHz offset,
but auto-discovered channels could land anywhere — including on DC.

`channel_mgr_add()` rejects any channel within 25 kHz of the center
frequency:

```c
float dc_offset = (float)freq - (float)mgr->center_freq;
if (fabsf(dc_offset) < 25000.0f) {
    fprintf(stderr, "[multi] %.3f MHz too close to DC, skipping\n", ...);
    return -1;
}
```

### AFC Per-Channel

The AFC (Automatic Frequency Control) UDP messages tell telive about
carrier frequency offsets. In single-channel mode, a single AFC sender
was used with a fixed RXID.

In multi-channel mode, this caused a phantom "RX:0" in telive. The fix:
send per-channel AFC with each channel's real FLL frequency offset and
correct RXID. The FLL frequency (in rad/sample at 36 kSps) is converted
to Hz and sent as the AFC value:

```c
for (int i = 0; i < nch; i++) {
    float fll_hz = channel_mgr_get_fll_hz(chmgr, i);
    int rxid = channel_mgr_get_rxid(chmgr, i);
    afc_udp_send_rx(afc, (int)fll_hz, rxid);
}
```

### tetra-rx Child Management

Each channel's tetra-rx is forked with:
- `TETRA_HACK_RXID` set to the channel's 1-based index
- `TETRA_HACK_IP` and `TETRA_HACK_PORT` inherited from parent
- stdin redirected from the channel's FIFO via `dup2()`

If a tetra-rx process exits unexpectedly (e.g. crash, broken pipe),
the channel manager detects it via `waitpid(WNOHANG)` and respawns
a new instance. Respawn is rate-limited: max 5 attempts per 60-second
window to prevent infinite restart loops.

Cleanup on exit (SIGINT/SIGTERM):
1. Send SIGTERM to all tetra-rx children
2. Close all FIFO write ends
3. `unlink()` all FIFO paths in /tmp
4. Free demod instances

---

## Scanner Mode

The scanner sweeps a frequency range in 25 kHz steps, dwelling on each
for a configurable period (default 3 seconds). At each step it:

1. Retunes the RTL-SDR
2. Creates a fresh demod with the scan buffer enabled
3. Demodulates for the dwell period
4. Checks the bit buffer for TETRA training sequences:
   - **SYNC** (y_bits): 38-bit pattern in the Synchronisation Burst
   - **NTS1** (n_bits): 22-bit Normal Training Sequence 1
5. If matches > 0, attempts BSCH decode for MCC/MNC/frequency
6. Reports the result and moves to the next step

The scanner uses the same DSP chain as normal demodulation. The only
difference is `dqpsk_demod_set_scan_buf()` which redirects bit output
to an internal buffer instead of stdout/FIFO, and `dqpsk_demod_reset()`
which reinitializes DSP state between steps.

---

## Sample Rate Constraints

The sample rate must be an integer multiple of 36,000 S/s (the TETRA
channel rate = 2 samples/symbol × 18,000 symbols/sec). The decimation
factor is `sample_rate / 36000`.

Common RTL-SDR rates like 2,400,000 S/s don't divide evenly (remainder
24,000). Valid rates: 1,800,000 (÷50), 2,304,000 (÷64), 2,520,000 (÷70),
2,880,000 (÷80).

The code validates this at startup:
```c
if (sample_rate % channel_rate != 0) {
    fprintf(stderr, "Error: sample_rate must be divisible by channel_rate\n");
    return 1;
}
```

---

## Web Server and Waterfall

The embedded web server (`web_server.c`) is a single-threaded HTTP server
that serves a self-contained HTML/JS/CSS page with no external dependencies.
It runs on a separate thread from the main DSP loop.

Key design decisions:
- **No external dependencies** — the entire web UI is embedded as a C string
  in `web_server.c`. No npm, no bundler, no static files.
- **Spectrum data via polling** — the browser fetches `/api/spectrum` at ~5 Hz.
  Returns the current FFT magnitude array as binary float32.
- **Frequency/gain control** — POST to `/api/freq` and `/api/gain`. In
  multi-channel mode, frequency retuning is rejected server-side to prevent
  breaking active demod channels.
- **Click-to-add-channel** — in multi-channel mode, clicking the waterfall
  posts to `/api/add_channel` with the clicked frequency. The server calls
  `channel_mgr_add()`.
- **Shared spectrum object** — the `spectrum_t` is fed by channel 0's demod
  (pre-decimation IQ) and read by both the web server and ASCII display.

---

## rtl_tcp Support

The `rtltcp_source.c` implements the rtl_tcp client protocol:
- TCP connection to `host:port`
- Reads 12-byte dongle info header (tuner type, gain count)
- Sends 5-byte commands: set frequency, set sample rate, set gain
- Reads raw uint8 IQ stream, same format as local RTL-SDR

The rtl_tcp source exposes the same `read()` / `set_freq()` / `set_gain()`
interface as the local RTL-SDR source, so the main loop doesn't need to
know which one is active. Both are wrapped in the `iq_src_t` dispatch
struct in `main.c`.
