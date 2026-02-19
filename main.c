/*
 * tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Replaces the GnuRadio + simdemod3_telive.py chain with a single C binary.
 * Outputs raw bits (1 byte per bit, 0x00/0x01) to stdout, compatible with:
 *
 *   tetra-rtlsdr -f 392.640e6 -g 32 | tetra-rx -r -s /dev/stdin
 *
 * Credits:
 *   telive / osmo-tetra-sq5bpf-2 by Jacek Lipkowski SQ5BPF
 *   https://github.com/sq5bpf/telive
 *   https://github.com/sq5bpf/osmo-tetra-sq5bpf-2
 *
 * Dependencies:
 *   librtlsdr  - RTL-SDR driver
 *   liquid-dsp - DSP library (AGC, timing sync, π/4-DQPSK demod)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <getopt.h>
#include <time.h>
#include <rtl-sdr.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rtlsdr_source.h"
#include "rtltcp_source.h"
#include "dqpsk_demod.h"
#include "spectrum.h"
#include "web_server.h"
#include "scanner.h"
#include "ascii_display.h"
#include "channel_mgr.h"

/* Default parameters */
#define DEFAULT_SAMPLE_RATE  1800000u   /* 1.8 MSps → decimate by 50 → 36 kSps */
#define DEFAULT_CHANNEL_RATE  36000u    /* TETRA channel sample rate */
#define DEFAULT_OFFSET       500000.0f  /* 500 kHz IF offset (avoids DC spike) */
#define DEFAULT_PPM           0
#define DEFAULT_DEVICE        0

/* Read block size: must be multiple of (decim_factor * 2) bytes = 100 bytes.
 * 512 * 100 = 51200 bytes. */
#define READ_BLOCK_BYTES     51200u

/* AFC UDP send interval in main loop iterations (approx every second) */
#define AFC_SEND_INTERVAL    20

static volatile int g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/* ---- AFC UDP (Automatic Frequency Correction info sent to telive) ----
 *
 * Mimics simdemod3_telive_send_udp_to_telive.py:
 *   Format: "TETMON_begin FUNC:AFCVAL AFC:<val> RX:<rxid> TETMON_end"
 * Reads the same TETRA_HACK_* environment variables as simdemod3.
 */
typedef struct {
    int                  fd;
    struct sockaddr_in   addr;
    int                  rxid;
} afc_udp_t;

static afc_udp_t *afc_udp_create(void) {
    const char *ip   = getenv("TETRA_HACK_IP");
    const char *port = getenv("TETRA_HACK_PORT");
    const char *rxid = getenv("TETRA_HACK_RXID");

    if (!ip)   ip   = "127.0.0.1";
    if (!port) port = "7379";
    if (!rxid) rxid = "0";

    afc_udp_t *a = calloc(1, sizeof(*a));
    if (!a) return NULL;

    a->rxid = atoi(rxid);
    a->fd   = socket(AF_INET, SOCK_DGRAM, 0);
    if (a->fd < 0) {
        fprintf(stderr, "[afc] Warning: cannot create UDP socket: %s\n",
                strerror(errno));
        free(a);
        return NULL;
    }

    memset(&a->addr, 0, sizeof(a->addr));
    a->addr.sin_family      = AF_INET;
    a->addr.sin_port        = htons((uint16_t)atoi(port));
    a->addr.sin_addr.s_addr = inet_addr(ip);

    fprintf(stderr, "[afc] AFC UDP → %s:%s  RX:%s\n", ip, port, rxid);
    return a;
}

static void afc_udp_send(afc_udp_t *a, int afc_val) {
    if (!a || a->fd < 0) return;
    char msg[128];
    int n = snprintf(msg, sizeof(msg),
                     "TETMON_begin FUNC:AFCVAL AFC:%d RX:%d TETMON_end",
                     afc_val, a->rxid);
    sendto(a->fd, msg, n, 0,
           (struct sockaddr *)&a->addr, sizeof(a->addr));
}

/* Send AFC for a specific RXID (used in multi-channel mode) */
static void afc_udp_send_rx(afc_udp_t *a, int afc_val, int rxid) {
    if (!a || a->fd < 0) return;
    char msg[128];
    int n = snprintf(msg, sizeof(msg),
                     "TETMON_begin FUNC:AFCVAL AFC:%d RX:%d TETMON_end",
                     afc_val, rxid);
    sendto(a->fd, msg, n, 0,
           (struct sockaddr *)&a->addr, sizeof(a->addr));
}

static void afc_udp_destroy(afc_udp_t *a) {
    if (!a) return;
    if (a->fd >= 0) close(a->fd);
    free(a);
}

/* ---- Generic IQ source wrapper (local RTL-SDR or rtl_tcp) ----
 *
 * Thin dispatch so the main loop doesn't need if/else everywhere.
 * Only one of .local / .tcp will be non-NULL at a time.
 */
typedef struct {
    rtlsdr_src_t  *local;
    rtltcp_src_t  *tcp;
} iq_source_t;

static int iq_src_start(iq_source_t *s) {
    if (s->tcp)   return rtltcp_src_start(s->tcp);
    if (s->local) return rtlsdr_src_start(s->local);
    return -1;
}
static void iq_src_stop(iq_source_t *s) {
    if (s->tcp)   rtltcp_src_stop(s->tcp);
    if (s->local) rtlsdr_src_stop(s->local);
}
static int iq_src_read(iq_source_t *s, uint8_t *buf, size_t n) {
    if (s->tcp)   return rtltcp_src_read(s->tcp, buf, n);
    if (s->local) return rtlsdr_src_read(s->local, buf, n);
    return -1;
}
static int __attribute__((unused)) iq_src_set_freq(iq_source_t *s, uint32_t freq) {
    if (s->tcp)   return rtltcp_src_set_freq(s->tcp, freq);
    if (s->local) return rtlsdr_src_set_freq(s->local, freq);
    return -1;
}
static int iq_src_set_gain(iq_source_t *s, int gain_tenths) {
    if (s->tcp)   return rtltcp_src_set_gain(s->tcp, gain_tenths);
    if (s->local) return rtlsdr_src_set_gain(s->local, gain_tenths);
    return -1;
}
static int iq_src_set_agc(iq_source_t *s, int enable) {
    if (s->tcp)   return rtltcp_src_set_agc(s->tcp, enable);
    if (s->local) return rtlsdr_src_set_agc(s->local, enable);
    return -1;
}
static size_t iq_src_overflow(iq_source_t *s) {
    if (s->tcp)   return rtltcp_src_overflow(s->tcp);
    if (s->local) return rtlsdr_src_overflow(s->local);
    return 0;
}
static void iq_src_destroy(iq_source_t *s) {
    if (s->tcp)   { rtltcp_src_destroy(s->tcp);   s->tcp = NULL; }
    if (s->local) { rtlsdr_src_destroy(s->local); s->local = NULL; }
}

/* ---- Usage ---- */

static void print_usage(const char *name) {
    fprintf(stderr,
        "tetra-rtlsdr - RTL-SDR π/4-DQPSK demodulator for TETRA\n"
        "\n"
        "Usage: %s -f FREQ [options] | tetra-rx -r -s /dev/stdin\n"
        "\n"
        "Required:\n"
        "  -f FREQ       Center frequency in Hz (e.g. 392.640e6 or 392640000)\n"
        "\n"
        "Optional:\n"
        "  -g GAIN       Tuner gain in dB (default: auto)\n"
        "  -p PPM        Frequency correction in ppm (default: 0)\n"
        "  -d DEVICE     RTL-SDR device index (default: 0)\n"
        "  -r RATE       Sample rate in S/s (default: 1800000)\n"
        "  -o OFFSET     IF offset in Hz, avoids DC spike (default: 500000)\n"
        "  -i FILE       Read raw uint8 IQ from file instead of RTL-SDR\n"
        "  -n HOST:PORT  Connect to rtl_tcp server instead of local RTL-SDR\n"
        "                Example: -n 192.168.1.50:1234\n"
        "  -w PORT       Enable web spectrum waterfall on http://localhost:PORT\n"
        "  -T PATH       Multi-channel: path to tetra-rx binary. Auto-scans\n"
        "                capture bandwidth and demodulates all TETRA found.\n"
        "  -A            Enable RTL2832U internal digital AGC\n"
        "  -a            ASCII spectrum display (retrogram-style, on stderr)\n"
        "  -S START:END  Scan mode: sweep frequency range for TETRA channels\n"
        "                Example: -S 390e6:395e6  (scan 390-395 MHz)\n"
        "  -D SECS       Scan dwell time per channel (default: 3.0)\n"
        "  -s            Show signal level on stderr (periodic)\n"
        "  -l            List available RTL-SDR devices and exit\n"
        "  -h            This help\n"
        "\n"
        "Environment variables (same as simdemod3_telive.py):\n"
        "  TETRA_HACK_IP      telive IP   (default: 127.0.0.1)\n"
        "  TETRA_HACK_PORT    telive port (default: 7379)\n"
        "  TETRA_HACK_RXID    receiver ID (default: 0)\n"
        "\n"
        "Examples:\n"
        "  tetra-rtlsdr -S 380e6:400e6 -g 48          # scan for TETRA channels\n"
        "  tetra-rtlsdr -f 394.811e6 -g 48 -a > /dev/null  # spectrum display\n"
        "  tetra-rtlsdr -f 394.811e6 -g 48 | tetra-rx -r -s /dev/stdin  # decode\n"
        "\n"
        "Credits: telive/osmo-tetra-sq5bpf-2 by SQ5BPF (sq5bpf@lipkowski.org)\n"
        "         https://github.com/sq5bpf/telive\n",
        name);
}

static void list_devices(void) {
    uint32_t n = rtlsdr_get_device_count();
    if (n == 0) {
        fprintf(stderr, "No RTL-SDR devices found.\n");
        return;
    }
    fprintf(stderr, "Found %u RTL-SDR device(s):\n", n);
    for (uint32_t i = 0; i < n; i++) {
        char manufact[256] = {0}, product[256] = {0}, serial[256] = {0};
        rtlsdr_get_device_usb_strings(i, manufact, product, serial);
        fprintf(stderr, "  [%u] %s %s  Serial: %s\n",
                i, manufact, product, serial);
    }
}

/* ---- main ---- */

int main(int argc, char *argv[]) {
    uint32_t freq         = 0;
    uint32_t sample_rate  = DEFAULT_SAMPLE_RATE;
    uint32_t channel_rate = DEFAULT_CHANNEL_RATE;
    double   gain_db      = 0.0;   /* 0 = auto */
    int      ppm          = DEFAULT_PPM;
    int      device_idx   = DEFAULT_DEVICE;
    float    freq_offset  = DEFAULT_OFFSET;
    int      show_signal  = 0;
    int      do_list      = 0;
    int      web_port     = 0;    /* 0 = disabled */
    int      ascii_mode   = 0;    /* -a: ASCII spectrum display */
    const char *iq_file   = NULL; /* file input mode */
    const char *tetra_rx  = NULL; /* -T: path to tetra-rx for multi-channel */
    const char *tcp_addr  = NULL; /* -n: rtl_tcp host:port */
    int      rtl_agc      = 0;    /* -A: enable RTL2832U digital AGC */
    uint32_t scan_start   = 0;   /* scan mode start frequency */
    uint32_t scan_end     = 0;   /* scan mode end frequency */
    float    scan_dwell   = 3.0f; /* scan dwell time per channel */

    int opt;
    while ((opt = getopt(argc, argv, "f:g:p:d:r:o:i:n:w:S:D:T:Aaslh")) != -1) {
        switch (opt) {
        case 'f': freq         = (uint32_t)strtod(optarg, NULL); break;
        case 'g': gain_db      = strtod(optarg, NULL);           break;
        case 'p': ppm          = atoi(optarg);                   break;
        case 'd': device_idx   = atoi(optarg);                   break;
        case 'r': sample_rate  = (uint32_t)strtod(optarg, NULL); break;
        case 'o': freq_offset  = (float)strtod(optarg, NULL);   break;
        case 'i': iq_file      = optarg;                         break;
        case 'n': tcp_addr     = optarg;                          break;
        case 'w': web_port     = atoi(optarg);                   break;
        case 'S': {
            char *colon = strchr(optarg, ':');
            if (!colon) {
                fprintf(stderr, "Error: -S requires START:END format "
                        "(e.g. -S 390e6:395e6)\n");
                return 1;
            }
            scan_start = (uint32_t)strtod(optarg, NULL);
            scan_end   = (uint32_t)strtod(colon + 1, NULL);
            break;
        }
        case 'D': scan_dwell   = (float)strtod(optarg, NULL);     break;
        case 'T': tetra_rx     = optarg;                          break;
        case 'A': rtl_agc      = 1;                              break;
        case 'a': ascii_mode   = 1;                              break;
        case 's': show_signal  = 1;                              break;
        case 'l': do_list      = 1;                              break;
        case 'h': print_usage(argv[0]); return 0;
        default:  print_usage(argv[0]); return 1;
        }
    }

    if (do_list) {
        list_devices();
        return 0;
    }

    if (!iq_file && !scan_start && freq == 0) {
        fprintf(stderr, "Error: frequency not specified (-f FREQ)\n\n");
        print_usage(argv[0]);
        return 1;
    }

    if (sample_rate % channel_rate != 0) {
        fprintf(stderr, "Error: sample_rate %u must be divisible by "
                "channel_rate %u\n", sample_rate, channel_rate);
        return 1;
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);   /* don't die if tetra-rx exits */

    fprintf(stderr,
            "tetra-rtlsdr  (c) 2026 CEMAXECUTER LLC  GPL-3.0\n"
            "Based on telive/osmo-tetra-sq5bpf-2 by SQ5BPF\n\n");

    /* --- AFC UDP (sends frequency correction info to telive) --- */
    afc_udp_t *afc = afc_udp_create();

    /* --- Spectrum (optional, enabled with -w PORT or -a) --- */
    spectrum_t   *spectrum = NULL;
    web_server_t *web      = NULL;

    if (web_port > 0 || ascii_mode) {
        /* Spectrum center = RTL-SDR center freq (target - offset) */
        double spec_center = iq_file ? (double)freq
                                     : (double)(freq - (uint32_t)freq_offset);
        spectrum = spectrum_create(spec_center, (double)sample_rate);
        if (!spectrum)
            fprintf(stderr, "[main] Warning: failed to create spectrum.\n");
    }

    /* --- Create demodulator --- */
    dqpsk_demod_t *demod = dqpsk_demod_create(sample_rate, channel_rate,
                                               freq_offset);
    if (!demod) {
        fprintf(stderr, "Failed to create demodulator.\n");
        web_server_destroy(web);
        spectrum_destroy(spectrum);
        afc_udp_destroy(afc);
        return 1;
    }

    /* Attach spectrum to demodulator if web is enabled */
    if (spectrum) dqpsk_demod_set_spectrum(demod, spectrum);

    /* =================================================================
     *  FILE INPUT MODE (-i FILE): read raw uint8 IQ from file
     * ================================================================= */
    if (iq_file) {
        FILE *fp = fopen(iq_file, "rb");
        if (!fp) {
            fprintf(stderr, "[main] Cannot open '%s': %s\n",
                    iq_file, strerror(errno));
            dqpsk_demod_destroy(demod);
            spectrum_destroy(spectrum);
            afc_udp_destroy(afc);
            return 1;
        }

        /* Get file size */
        fseek(fp, 0, SEEK_END);
        long file_size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        fprintf(stderr, "[main] File input: %s (%.1f MB)\n",
                iq_file, (double)file_size / (1024.0 * 1024.0));
        fprintf(stderr, "[main] Sample rate: %u S/s, IF offset: %.0f Hz\n",
                sample_rate, freq_offset);
        fprintf(stderr, "[main] Demodulating → stdout\n\n");

        uint8_t *read_buf = malloc(READ_BLOCK_BYTES);
        if (!read_buf) {
            fprintf(stderr, "Out of memory.\n");
            fclose(fp);
            dqpsk_demod_destroy(demod);
            spectrum_destroy(spectrum);
            afc_udp_destroy(afc);
            return 1;
        }

        size_t total_read = 0;
        while (g_running) {
            size_t n = fread(read_buf, 1, READ_BLOCK_BYTES, fp);
            if (n == 0) break;
            total_read += n;

            if (dqpsk_demod_process(demod, read_buf, n, 1) < 0)
                break;
        }

        fprintf(stderr, "\n[main] Done. Processed %.1f MB.\n",
                (double)total_read / (1024.0 * 1024.0));

        free(read_buf);
        fclose(fp);
        dqpsk_demod_destroy(demod);
        web_server_destroy(web);
        spectrum_destroy(spectrum);
        afc_udp_destroy(afc);
        return 0;
    }

    /* =================================================================
     *  SCAN MODE (-S START:END): sweep for TETRA channels
     * ================================================================= */
    if (scan_start > 0 && scan_end > scan_start) {
        /* Open RTL-SDR at the scan start frequency */
        uint32_t rtl_center = scan_start - (uint32_t)freq_offset;
        int gain_tenths = (gain_db == 0.0) ? 0 : (int)(gain_db * 10.0 + 0.5);

        rtlsdr_src_t *src = rtlsdr_src_create(rtl_center, sample_rate,
                                               gain_tenths, device_idx, ppm);
        if (!src) {
            dqpsk_demod_destroy(demod);
            spectrum_destroy(spectrum);
            afc_udp_destroy(afc);
            return 1;
        }

        if (rtl_agc)
            rtlsdr_src_set_agc(src, 1);

        if (rtlsdr_src_start(src) < 0) {
            rtlsdr_src_destroy(src);
            dqpsk_demod_destroy(demod);
            spectrum_destroy(spectrum);
            afc_udp_destroy(afc);
            return 1;
        }

        scanner_t *scan = scanner_create(src, demod,
                                          scan_start, scan_end,
                                          25000, freq_offset,
                                          scan_dwell);
        if (!scan) {
            fprintf(stderr, "Failed to create scanner.\n");
            rtlsdr_src_stop(src);
            rtlsdr_src_destroy(src);
            dqpsk_demod_destroy(demod);
            spectrum_destroy(spectrum);
            afc_udp_destroy(afc);
            return 1;
        }

        int n_found = scanner_run(scan);
        (void)n_found;

        scanner_destroy(scan);
        rtlsdr_src_stop(src);
        rtlsdr_src_destroy(src);
        dqpsk_demod_destroy(demod);
        spectrum_destroy(spectrum);
        afc_udp_destroy(afc);
        return 0;
    }

    /* =================================================================
     *  LIVE MODE (local RTL-SDR or rtl_tcp)
     * ================================================================= */

    /* -f is the TETRA target frequency.
     * RTL-SDR tunes below by freq_offset to avoid the DC spike. */
    uint32_t target_freq = freq;
    uint32_t rtl_center  = freq - (uint32_t)freq_offset;

    /* gain_tenths: librtlsdr uses 0.1 dB units; 0 = auto */
    int gain_tenths = (gain_db == 0.0) ? 0 : (int)(gain_db * 10.0 + 0.5);

    /* --- Open IQ source (local RTL-SDR or rtl_tcp) --- */
    iq_source_t src = {0};

    if (tcp_addr) {
        /* Parse host:port from -n argument */
        char tcp_host[256] = "127.0.0.1";
        int  tcp_port = 1234;
        const char *colon = strrchr(tcp_addr, ':');
        if (colon && colon != tcp_addr) {
            size_t hlen = (size_t)(colon - tcp_addr);
            if (hlen >= sizeof(tcp_host)) hlen = sizeof(tcp_host) - 1;
            memcpy(tcp_host, tcp_addr, hlen);
            tcp_host[hlen] = '\0';
            tcp_port = atoi(colon + 1);
        } else {
            snprintf(tcp_host, sizeof(tcp_host), "%s", tcp_addr);
        }

        src.tcp = rtltcp_src_create(tcp_host, tcp_port, rtl_center,
                                      sample_rate, gain_tenths, ppm);
        if (!src.tcp) {
            dqpsk_demod_destroy(demod);
            afc_udp_destroy(afc);
            return 1;
        }
    } else {
        src.local = rtlsdr_src_create(rtl_center, sample_rate,
                                        gain_tenths, device_idx, ppm);
        if (!src.local) {
            dqpsk_demod_destroy(demod);
            afc_udp_destroy(afc);
            return 1;
        }
    }

    /* --- RTL2832U digital AGC (optional, -A flag) --- */
    if (rtl_agc)
        iq_src_set_agc(&src, 1);

    /* --- Start web server (now that IQ source is ready) --- */
    if (web_port > 0 && spectrum) {
        web = web_server_create(web_port, spectrum, src.local,
                                target_freq, freq_offset);
        if (web && src.tcp)
            web_server_set_tcp_source(web, src.tcp);
        if (!web)
            fprintf(stderr, "[main] Warning: failed to start web server.\n");
    }

    /* --- ASCII spectrum display (-a flag) --- */
    ascii_display_t *adisp = NULL;
    if (ascii_mode && spectrum) {
        adisp = ascii_display_create(spectrum, (double)rtl_center,
                                      (double)sample_rate);
        if (!adisp)
            fprintf(stderr, "[main] Warning: ASCII display unavailable "
                    "(stderr not a TTY?).\n");
    }

    /* --- Start capture --- */
    if (iq_src_start(&src) < 0) {
        ascii_display_destroy(adisp);
        iq_src_destroy(&src);
        dqpsk_demod_destroy(demod);
        web_server_destroy(web);
        spectrum_destroy(spectrum);
        afc_udp_destroy(afc);
        return 1;
    }

    /* Allocate read buffer (must be multiple of decim_factor*2 bytes) */
    uint8_t *read_buf = malloc(READ_BLOCK_BYTES);
    if (!read_buf) {
        fprintf(stderr, "Out of memory.\n");
        ascii_display_destroy(adisp);
        iq_src_stop(&src);
        iq_src_destroy(&src);
        dqpsk_demod_destroy(demod);
        web_server_destroy(web);
        spectrum_destroy(spectrum);
        afc_udp_destroy(afc);
        return 1;
    }

    /* =================================================================
     *  MULTI-CHANNEL MODE (-T /path/to/tetra-rx)
     *
     *  Starts the user's frequency immediately, then runs a background
     *  in-band scan that discovers additional channels while all active
     *  channels + waterfall continue in real time.
     * ================================================================= */
    channel_mgr_t *chmgr = NULL;

    if (tetra_rx) {
        /* Verify tetra-rx binary exists */
        if (access(tetra_rx, X_OK) != 0) {
            fprintf(stderr, "[multi] tetra-rx not found or not executable: %s\n",
                    tetra_rx);
            free(read_buf);
            ascii_display_destroy(adisp);
            iq_src_stop(&src);
            iq_src_destroy(&src);
            dqpsk_demod_destroy(demod);
            web_server_destroy(web);
            spectrum_destroy(spectrum);
            afc_udp_destroy(afc);
            return 1;
        }

        chmgr = channel_mgr_create(sample_rate, channel_rate,
                                     tetra_rx, rtl_center);
        if (!chmgr) {
            fprintf(stderr, "[multi] Failed to create channel manager.\n");
            free(read_buf);
            ascii_display_destroy(adisp);
            iq_src_stop(&src);
            iq_src_destroy(&src);
            dqpsk_demod_destroy(demod);
            web_server_destroy(web);
            spectrum_destroy(spectrum);
            afc_udp_destroy(afc);
            return 1;
        }

        /* Start user's frequency immediately — no waiting for scan */
        channel_mgr_add(chmgr, target_freq);

        /* Attach spectrum to ch0 so waterfall is live from the start */
        if (spectrum)
            channel_mgr_set_spectrum(chmgr, spectrum);

        /* Kick off non-blocking background scan.
         * Runs incrementally inside channel_mgr_process(),
         * 1500ms dwell per 25 kHz step. */
        channel_mgr_start_scan(chmgr, 1500);

        /* Tell web server and ASCII display about channel manager */
        if (web)
            web_server_set_channel_mgr(web, chmgr);
        if (adisp)
            ascii_display_set_channel_mgr(adisp, chmgr);

        fprintf(stderr, "\n[main] Multi-channel mode: %.3f MHz active, "
                "scanning in background\n"
                "[main] RTL-SDR center: %.6f MHz\n\n",
                target_freq / 1e6, rtl_center / 1e6);
    } else {
        fprintf(stderr, "[main] Demodulating TETRA at %.6f MHz → stdout\n"
                "[main] RTL-SDR center: %.6f MHz (offset: %.0f Hz)\n\n",
                (double)target_freq / 1e6, (double)rtl_center / 1e6,
                freq_offset);
    }

    /* --- Main processing loop --- */
    time_t   last_status = time(NULL);
    size_t   blocks      = 0;
    unsigned afc_counter = 0;

    while (g_running) {
        if (iq_src_read(&src, read_buf, READ_BLOCK_BYTES) < 0) {
            fprintf(stderr, "[main] Read error or capture done.\n");
            break;
        }

        if (chmgr) {
            /* Multi-channel: distribute IQ to all demod channels */
            channel_mgr_process(chmgr, read_buf, READ_BLOCK_BYTES);

            /* Feed spectrum from raw IQ (first demod handles this) */
            if (spectrum) {
                /* Manually push raw IQ to spectrum for display.
                 * In multi-channel mode, the individual demods don't have
                 * the spectrum attached since we want one shared display. */
                /* Note: spectrum is fed by the demod with set_spectrum,
                 * which we attached to the single-channel demod.
                 * For multi-channel we need to attach it to the first channel's
                 * demod or feed it separately. For now, skip. */
            }
        } else {
            /* Single-channel: direct to stdout */
            if (dqpsk_demod_process(demod, read_buf, READ_BLOCK_BYTES, 1) < 0)
                break;
        }

        blocks++;

        /* Send AFC UDP to telive */
        if (++afc_counter >= AFC_SEND_INTERVAL) {
            afc_counter = 0;
            if (chmgr) {
                /* Multi-channel: send per-channel AFC with correct RXID */
                int nch = channel_mgr_n_channels(chmgr);
                for (int i = 0; i < nch; i++) {
                    float fll_hz = channel_mgr_get_fll_hz(chmgr, i);
                    int rxid = channel_mgr_get_rxid(chmgr, i);
                    afc_udp_send_rx(afc, (int)fll_hz, rxid);
                }
            } else {
                afc_udp_send(afc, 0);
            }
        }

        /* Optional stderr status */
        if (show_signal && !adisp) {
            time_t now = time(NULL);
            if (now - last_status >= 5) {
                if (chmgr) {
                    fprintf(stderr, "[status] %d ch  overflow: %zu  "
                            "blocks: %zu\n",
                            channel_mgr_n_channels(chmgr),
                            iq_src_overflow(&src), blocks);
                } else {
                    fprintf(stderr, "[status] RSSI: %.1f dB  overflow: %zu  "
                            "blocks: %zu\n",
                            dqpsk_demod_get_rssi_db(demod),
                            iq_src_overflow(&src), blocks);
                }
                last_status = now;
            }
        }

        /* ASCII spectrum display */
        float display_rssi = chmgr
            ? channel_mgr_get_rssi(chmgr, 0)
            : dqpsk_demod_get_rssi_db(demod);
        ascii_cmd_t acmd = ascii_display_update(adisp, display_rssi, 0.0f, 0);
        if (acmd == ASCII_CMD_QUIT) {
            g_running = 0;
        } else if (acmd == ASCII_CMD_GAIN_UP) {
            gain_tenths += 20;
            iq_src_set_gain(&src, gain_tenths);
        } else if (acmd == ASCII_CMD_GAIN_DOWN) {
            gain_tenths -= 20;
            if (gain_tenths < 0) gain_tenths = 0;
            iq_src_set_gain(&src, gain_tenths);
        }
    }

    fprintf(stderr, "\n[main] Stopping...\n");
    iq_src_stop(&src);

    if (iq_src_overflow(&src) > 0)
        fprintf(stderr, "[main] Warning: %zu bytes dropped (ring buffer "
                "overflow)\n", iq_src_overflow(&src));

    free(read_buf);
    channel_mgr_destroy(chmgr);
    ascii_display_destroy(adisp);
    iq_src_destroy(&src);
    dqpsk_demod_destroy(demod);
    web_server_destroy(web);
    spectrum_destroy(spectrum);
    afc_udp_destroy(afc);
    return 0;
}
