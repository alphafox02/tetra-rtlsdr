/*
 * tetra-rtlsdr - web server with spectrum waterfall and tuning controls
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Serves a control panel + spectrum waterfall on http://localhost:<port>/
 * Streams FFT data as Server-Sent Events on /spectrum
 * Provides JSON API for frequency and gain control
 * No external HTTP library required.
 */

#include "web_server.h"
#include "spectrum.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ---- Embedded HTML/CSS/JS ------------------------------------------------ */

static const char PANEL_HTML[] =
"<!DOCTYPE html>\n"
"<html><head><meta charset=utf-8><meta name=viewport content='width=device-width'>\n"
"<title>tetra-rtlsdr</title>\n"
"<style>\n"
"*{box-sizing:border-box;margin:0;padding:0}\n"
"body{background:#0a0e14;color:#c8ccd4;font-family:-apple-system,system-ui,sans-serif;\n"
"  font-size:14px;padding:12px;max-width:1100px;margin:0 auto}\n"
"h1{font-size:16px;font-weight:600;color:#61afef;margin-bottom:8px}\n"
"h1 span{color:#5c6370;font-weight:400;font-size:13px}\n"
".panel{background:#1a1f29;border:1px solid #2c313a;border-radius:6px;\n"
"  padding:16px;margin-bottom:10px}\n"
".row{display:flex;align-items:center;gap:12px;margin-bottom:10px;flex-wrap:wrap}\n"
".row:last-child{margin-bottom:0}\n"
"label{color:#7a818e;font-size:12px;text-transform:uppercase;letter-spacing:.5px;\n"
"  min-width:70px;flex-shrink:0}\n"
".freq-group{display:flex;align-items:center;gap:4px}\n"
".freq-input{background:#0d1117;border:1px solid #30363d;color:#e6edf3;\n"
"  font-size:20px;font-family:'SF Mono',Consolas,monospace;text-align:right;\n"
"  padding:6px 10px;border-radius:4px;width:160px;outline:none}\n"
".freq-input:focus{border-color:#61afef}\n"
".freq-unit{color:#7a818e;font-size:14px;margin-left:4px}\n"
".btn{background:#21262d;border:1px solid #30363d;color:#c8ccd4;padding:5px 10px;\n"
"  border-radius:4px;cursor:pointer;font-size:13px;font-family:inherit;\n"
"  transition:background .15s}\n"
".btn:hover{background:#30363d}\n"
".btn:active{background:#484f58}\n"
".btn-sm{padding:3px 8px;font-size:12px}\n"
".step-btns{display:flex;gap:2px}\n"
"input[type=range]{flex:1;accent-color:#61afef;height:6px;max-width:220px}\n"
".val{color:#e6edf3;font-family:'SF Mono',Consolas,monospace;font-size:13px;\n"
"  min-width:60px}\n"
".rssi-bar{flex:1;max-width:220px;height:8px;background:#0d1117;\n"
"  border-radius:4px;overflow:hidden;border:1px solid #21262d}\n"
".rssi-fill{height:100%;background:linear-gradient(90deg,#e5534b,#f0883e,#57ab5a);\n"
"  transition:width .3s;border-radius:4px}\n"
".status{display:flex;gap:16px;align-items:center}\n"
".status-dot{width:8px;height:8px;border-radius:50%;display:inline-block}\n"
".dot-ok{background:#57ab5a}\n"
".dot-warn{background:#f0883e}\n"
".dot-off{background:#484f58}\n"
".wf-wrap{position:relative;margin-bottom:2px}\n"
"#c{display:block;width:100%;image-rendering:pixelated;\n"
"  border-radius:4px;border:1px solid #2c313a}\n"
"#overlay{position:absolute;top:0;left:0;width:100%;height:100%;\n"
"  pointer-events:none}\n"
"#tooltip{position:absolute;display:none;background:rgba(13,17,23,0.9);\n"
"  border:1px solid #30363d;border-radius:4px;padding:4px 8px;\n"
"  color:#e6edf3;font-size:12px;font-family:'SF Mono',Consolas,monospace;\n"
"  white-space:nowrap;pointer-events:none;z-index:10}\n"
".freq-axis{display:flex;justify-content:space-between;color:#5c6370;\n"
"  font-size:11px;font-family:'SF Mono',Consolas,monospace;padding:2px 0}\n"
"</style></head>\n"
"<body>\n"
"<h1>tetra-rtlsdr <span>TETRA demodulator</span></h1>\n"
"\n"
"<div class=panel>\n"
"  <div class=row>\n"
"    <label>Frequency</label>\n"
"    <div class=freq-group>\n"
"      <input type=text id=freq class=freq-input value='392.640'>\n"
"      <span class=freq-unit>MHz</span>\n"
"    </div>\n"
"    <div class=step-btns>\n"
"      <button class=btn onclick='stepFreq(-0.025)'>-25k</button>\n"
"      <button class=btn onclick='stepFreq(+0.025)'>+25k</button>\n"
"    </div>\n"
"    <button class='btn btn-sm' onclick='setFreq()'>Tune</button>\n"
"  </div>\n"
"  <div class=row>\n"
"    <label>Gain</label>\n"
"    <input type=range id=gain min=0 max=49.6 step=0.1 value=32>\n"
"    <span class=val id=gainVal>32.0 dB</span>\n"
"    <button class='btn btn-sm' onclick='setGain()'>Set</button>\n"
"  </div>\n"
"  <div class=row>\n"
"    <label>IF Offset</label>\n"
"    <span class=val id=offsetVal>500000 Hz</span>\n"
"  </div>\n"
"  <div class=row>\n"
"    <label>RSSI</label>\n"
"    <div class=rssi-bar><div class=rssi-fill id=rssiFill style='width:50%'></div></div>\n"
"    <span class=val id=rssiVal>-- dB</span>\n"
"  </div>\n"
"  <div class=row>\n"
"    <label>Status</label>\n"
"    <div class=status>\n"
"      <span><span class='status-dot dot-off' id=statusDot></span> <span id=statusText>Connecting...</span></span>\n"
"      <span id=chInfo style='color:#61afef;font-size:12px'></span>\n"
"    </div>\n"
"  </div>\n"
"</div>\n"
"\n"
"<div class=wf-wrap>\n"
"  <canvas id=c></canvas>\n"
"  <canvas id=overlay></canvas>\n"
"  <div id=tooltip></div>\n"
"</div>\n"
"<div class=freq-axis><span id=fLo></span><span id=fCenter></span><span id=fHi></span></div>\n"
"\n"
"<script>\n"
"const NFFT=1024,ROWS=300;\n"
"const canvas=document.getElementById('c');\n"
"const ctx=canvas.getContext('2d');\n"
"canvas.width=NFFT; canvas.height=ROWS;\n"
"canvas.style.height=ROWS+'px';\n"
"ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,NFFT,ROWS);\n"
"\n"
"/* Overlay canvas for cursor/markers */\n"
"const overlay=document.getElementById('overlay');\n"
"const octx=overlay.getContext('2d');\n"
"overlay.width=NFFT; overlay.height=ROWS;\n"
"overlay.style.height=ROWS+'px';\n"
"const tooltip=document.getElementById('tooltip');\n"
"let cursorX=-1,lastPsd=null,curCenter=0,curRate=0;\n"
"\n"
"/* Viridis-inspired palette */\n"
"function psdColor(v){\n"
"  v=Math.max(0,Math.min(1,v));\n"
"  let r,g,b;\n"
"  if(v<0.25){r=68+v*4*(33-68);g=1+v*4*(144-1);b=84+v*4*(140-84);}\n"
"  else if(v<0.5){let t=(v-.25)*4;r=33+t*(94-33);g=144+t*(201-144);b=140+t*(98-140);}\n"
"  else if(v<0.75){let t=(v-.5)*4;r=94+t*(190-94);g=201+t*(222-201);b=98+t*(34-98);}\n"
"  else{let t=(v-.75)*4;r=190+t*(253-190);g=222+t*(231-222);b=34+t*(37-34);}\n"
"  return[r|0,g|0,b|0];\n"
"}\n"
"\n"
"let minDb=-70,maxDb=-20;\n"
"function addRow(psd){\n"
"  let mn=Math.min(...psd),mx=Math.max(...psd);\n"
"  minDb=minDb*0.97+mn*0.03;\n"
"  maxDb=maxDb*0.97+mx*0.03;\n"
"  let range=maxDb-minDb||1;\n"
"  let img=ctx.getImageData(0,0,NFFT,ROWS-1);\n"
"  ctx.putImageData(img,0,1);\n"
"  let row=ctx.createImageData(NFFT,1),d=row.data;\n"
"  for(let x=0;x<NFFT;x++){\n"
"    let v=(psd[x]-minDb)/range;\n"
"    let[r,g,b]=psdColor(v);\n"
"    d[x*4]=r;d[x*4+1]=g;d[x*4+2]=b;d[x*4+3]=255;\n"
"  }\n"
"  ctx.putImageData(row,0,0);\n"
"}\n"
"\n"
"/* Overlay: cursor line + tuned freq marker + hover tooltip */\n"
"function drawOverlay(){\n"
"  octx.clearRect(0,0,NFFT,ROWS);\n"
"  if(!curCenter||!curRate)return;\n"
"  let fLo=curCenter-curRate/2,fHi=curCenter+curRate/2;\n"
"  /* Tuned frequency marker (dashed cyan line) */\n"
"  let tf=parseFloat(document.getElementById('freq').value)*1e6;\n"
"  if(tf>=fLo&&tf<=fHi){\n"
"    let tx=((tf-fLo)/(fHi-fLo))*NFFT;\n"
"    octx.strokeStyle='rgba(97,175,239,0.6)';octx.lineWidth=1;\n"
"    octx.setLineDash([4,4]);octx.beginPath();\n"
"    octx.moveTo(tx,0);octx.lineTo(tx,ROWS);octx.stroke();\n"
"    octx.setLineDash([]);\n"
"    /* Label */\n"
"    octx.font='10px monospace';octx.fillStyle='rgba(97,175,239,0.8)';\n"
"    octx.textAlign='center';octx.fillText((tf/1e6).toFixed(3),tx,ROWS-4);\n"
"  }\n"
"  /* Cursor line */\n"
"  if(cursorX>=0&&cursorX<NFFT){\n"
"    octx.strokeStyle='rgba(200,204,212,0.4)';octx.lineWidth=1;\n"
"    octx.beginPath();octx.moveTo(cursorX,0);\n"
"    octx.lineTo(cursorX,ROWS);octx.stroke();\n"
"  }\n"
"}\n"
"\n"
"/* Mousemove on wrapper for hover info */\n"
"const wfWrap=document.querySelector('.wf-wrap');\n"
"wfWrap.style.cursor='crosshair';\n"
"wfWrap.addEventListener('mousemove',function(e){\n"
"  let rect=canvas.getBoundingClientRect();\n"
"  let xPx=e.clientX-rect.left;\n"
"  let x=xPx/rect.width;\n"
"  cursorX=Math.round(x*NFFT);\n"
"  if(!curCenter||!curRate){tooltip.style.display='none';return;}\n"
"  let fLo=curCenter-curRate/2,fHi=curCenter+curRate/2;\n"
"  let fHz=fLo+x*(fHi-fLo);\n"
"  let fMHz=(fHz/1e6).toFixed(4);\n"
"  let pwr=lastPsd&&cursorX>=0&&cursorX<NFFT?lastPsd[cursorX].toFixed(1):'--';\n"
"  tooltip.textContent=fMHz+' MHz  '+pwr+' dB';\n"
"  tooltip.style.display='block';\n"
"  let ty=Math.max(0,e.clientY-rect.top-28);\n"
"  let tx=Math.min(rect.width-tooltip.offsetWidth-4,xPx+8);\n"
"  tooltip.style.left=tx+'px';tooltip.style.top=ty+'px';\n"
"  drawOverlay();\n"
"});\n"
"wfWrap.addEventListener('mouseleave',function(){\n"
"  cursorX=-1;tooltip.style.display='none';drawOverlay();\n"
"});\n"
"\n"
"/* Controls */\n"
"const freqEl=document.getElementById('freq');\n"
"const gainEl=document.getElementById('gain');\n"
"const gainValEl=document.getElementById('gainVal');\n"
"const rssiValEl=document.getElementById('rssiVal');\n"
"const rssiFillEl=document.getElementById('rssiFill');\n"
"const offsetValEl=document.getElementById('offsetVal');\n"
"const statusDotEl=document.getElementById('statusDot');\n"
"const statusTextEl=document.getElementById('statusText');\n"
"\n"
"gainEl.oninput=function(){gainValEl.textContent=parseFloat(this.value).toFixed(1)+' dB';};\n"
"\n"
"freqEl.addEventListener('keydown',function(e){\n"
"  if(e.key==='Enter')setFreq();\n"
"});\n"
"\n"
"function stepFreq(delta){\n"
"  let f=parseFloat(freqEl.value)+delta;\n"
"  freqEl.value=f.toFixed(3);\n"
"  setFreq();\n"
"}\n"
"\n"
"function setFreq(){\n"
"  let fMHz=parseFloat(freqEl.value);\n"
"  if(isNaN(fMHz))return;\n"
"  let fHz=Math.round(fMHz*1e6);\n"
"  fetch('/api/freq',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},\n"
"    body:'freq_hz='+fHz}).then(r=>r.json()).then(d=>{\n"
"      if(d.ok)statusTextEl.textContent='Tuned to '+fMHz.toFixed(3)+' MHz';\n"
"      else statusTextEl.textContent='Tune failed';\n"
"  }).catch(()=>{statusTextEl.textContent='Error';});\n"
"}\n"
"\n"
"function setGain(){\n"
"  let g=parseFloat(gainEl.value);\n"
"  fetch('/api/gain',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},\n"
"    body:'gain_db='+g}).then(r=>r.json()).then(d=>{\n"
"      if(d.ok)statusTextEl.textContent='Gain set to '+g.toFixed(1)+' dB';\n"
"  }).catch(()=>{});\n"
"}\n"
"\n"
"/* SSE spectrum stream */\n"
"let connected=false;\n"
"const evts=new EventSource('/spectrum');\n"
"evts.onmessage=function(e){\n"
"  if(!connected){connected=true;statusDotEl.className='status-dot dot-ok';\n"
"    statusTextEl.textContent='Receiving';}\n"
"  const d=JSON.parse(e.data);\n"
"  curCenter=d.center;curRate=d.rate;lastPsd=d.psd;\n"
"  let lo=(d.center-d.rate/2)/1e6,hi=(d.center+d.rate/2)/1e6;\n"
"  document.getElementById('fLo').textContent=lo.toFixed(3)+' MHz';\n"
"  document.getElementById('fCenter').textContent=(d.center/1e6).toFixed(3)+' MHz';\n"
"  document.getElementById('fHi').textContent=hi.toFixed(3)+' MHz';\n"
"  rssiValEl.textContent=d.rssi.toFixed(1)+' dB';\n"
"  let pct=Math.max(0,Math.min(100,(d.rssi+80)/60*100));\n"
"  rssiFillEl.style.width=pct+'%';\n"
"  if(d.offset!==undefined)offsetValEl.textContent=d.offset+' Hz';\n"
"  addRow(d.psd);\n"
"  drawOverlay();\n"
"};\n"
"evts.onerror=function(){\n"
"  connected=false;\n"
"  statusDotEl.className='status-dot dot-warn';\n"
"  statusTextEl.textContent='Reconnecting...';\n"
"};\n"
"\n"
"/* Click waterfall: add channel (multi) or retune (single) */\n"
"let isMulti=false,nChannels=0,maxChannels=6;\n"
"wfWrap.addEventListener('click',function(e){\n"
"  let rect=canvas.getBoundingClientRect();\n"
"  let x=(e.clientX-rect.left)/rect.width;\n"
"  if(!curCenter||!curRate)return;\n"
"  let fLo=curCenter-curRate/2,fHi=curCenter+curRate/2;\n"
"  let fClicked=(fLo+x*(fHi-fLo))/1e6;\n"
"  fClicked=Math.round(fClicked*1000/25)*25/1000;\n"
"  if(isMulti){\n"
"    if(nChannels>=maxChannels){statusTextEl.textContent='Max channels reached';return;}\n"
"    let fHz=Math.round(fClicked*1e6);\n"
"    fetch('/api/channel',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},\n"
"      body:'freq_hz='+fHz}).then(r=>r.json()).then(d=>{\n"
"        statusTextEl.textContent=d.msg;\n"
"        if(d.channels!==undefined)nChannels=d.channels;\n"
"        updateChannelInfo();\n"
"    }).catch(()=>{});\n"
"  } else {\n"
"    freqEl.value=fClicked.toFixed(3);\n"
"    setFreq();\n"
"  }\n"
"  drawOverlay();\n"
"});\n"
"function updateChannelInfo(){\n"
"  let ci=document.getElementById('chInfo');\n"
"  if(!isMulti){ci.textContent='';setFreqDisabled(false);return;}\n"
"  ci.textContent=nChannels+'/'+maxChannels+' ch';\n"
"  setFreqDisabled(true);\n"
"}\n"
"function setFreqDisabled(dis){\n"
"  freqEl.disabled=dis;\n"
"  document.querySelectorAll('.step-btns .btn').forEach(b=>b.disabled=dis);\n"
"  let tb=document.querySelector('.btn-sm');\n"
"  if(tb){tb.disabled=dis;tb.textContent=dis?'Multi':'Tune';}\n"
"}\n"
"\n"
"/* Load initial status */\n"
"function pollStatus(){\n"
"  fetch('/api/status').then(r=>r.json()).then(d=>{\n"
"    if(d.freq)freqEl.value=(d.freq/1e6).toFixed(3);\n"
"    if(d.gain!==undefined){gainEl.value=d.gain;gainValEl.textContent=d.gain.toFixed(1)+' dB';}\n"
"    if(d.offset!==undefined)offsetValEl.textContent=d.offset+' Hz';\n"
"    if(d.multi!==undefined){isMulti=d.multi;nChannels=d.channels||0;maxChannels=d.max_channels||6;updateChannelInfo();}\n"
"  }).catch(()=>{});\n"
"}\n"
"pollStatus();\n"
"setInterval(function(){if(isMulti)pollStatus();},5000);\n"
"</script>\n"
"</body></html>\n";

/* ---- HTTP helpers -------------------------------------------------------- */

static int sendall(int fd, const char *buf, size_t n) {
    size_t sent = 0;
    while (sent < n) {
        ssize_t r = send(fd, buf + sent, n - sent, MSG_NOSIGNAL);
        if (r <= 0) return -1;
        sent += (size_t)r;
    }
    return 0;
}

static int send_str(int fd, const char *s) {
    return sendall(fd, s, strlen(s));
}

static void send_json(int fd, const char *json) {
    char hdr[256];
    snprintf(hdr, sizeof(hdr),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %zu\r\n"
             "Access-Control-Allow-Origin: *\r\n"
             "Connection: close\r\n\r\n",
             strlen(json));
    send_str(fd, hdr);
    send_str(fd, json);
}

/* Parse "key=value" from POST body, return value as double. Returns 0 on fail. */
static double parse_form_value(const char *body, const char *key) {
    char needle[64];
    snprintf(needle, sizeof(needle), "%s=", key);
    const char *p = strstr(body, needle);
    if (!p) return 0;
    return strtod(p + strlen(needle), NULL);
}

/* ---- Client handler ------------------------------------------------------ */

typedef struct {
    int           fd;
    spectrum_t   *spectrum;
    web_server_t *ws;
} client_arg_t;

struct web_server {
    int              listen_fd;
    int              port;
    spectrum_t      *spectrum;
    rtlsdr_src_t    *src;           /* NULL in file/tcp mode */
    rtltcp_src_t    *tcp;           /* NULL in local/file mode */
    channel_mgr_t   *chmgr;        /* NULL in single-channel mode */
    uint32_t         target_freq;   /* TETRA channel freq in Hz */
    float            freq_offset;   /* IF offset in Hz */
    float            gain_db;       /* current gain */
    pthread_t        thread;
    volatile int     running;
    pthread_mutex_t  lock;
};

static void *handle_client(void *arg) {
    client_arg_t *ca = (client_arg_t *)arg;
    int fd = ca->fd;
    spectrum_t *spectrum = ca->spectrum;
    web_server_t *ws = ca->ws;
    free(ca);

    /* Read HTTP request (headers + body) */
    char req[2048] = {0};
    ssize_t n = recv(fd, req, sizeof(req) - 1, 0);
    if (n <= 0) { close(fd); return NULL; }

    /* Determine method and path */
    int is_get  = (strncmp(req, "GET ", 4) == 0);
    int is_post = (strncmp(req, "POST ", 5) == 0);

    /* Find body (after \r\n\r\n) */
    const char *body = strstr(req, "\r\n\r\n");
    if (body) body += 4; else body = "";

    /* --- GET / (control panel) --- */
    if (is_get && (strncmp(req + 4, "/ ", 2) == 0 ||
                   strncmp(req + 4, "/\r", 2) == 0 ||
                   strncmp(req + 4, "/\n", 2) == 0)) {
        char hdr[256];
        snprintf(hdr, sizeof(hdr),
                 "HTTP/1.1 200 OK\r\n"
                 "Content-Type: text/html; charset=utf-8\r\n"
                 "Content-Length: %zu\r\n"
                 "Connection: close\r\n\r\n",
                 strlen(PANEL_HTML));
        send_str(fd, hdr);
        send_str(fd, PANEL_HTML);
    }

    /* --- GET /spectrum (SSE stream) --- */
    else if (is_get && strncmp(req + 4, "/spectrum", 9) == 0) {
        send_str(fd,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/event-stream\r\n"
            "Cache-Control: no-cache\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Connection: keep-alive\r\n\r\n");

        float psd[SPECTRUM_NFFT];
        uint64_t last_seq = 0;

        while (1) {
            if (spectrum_get(spectrum, psd, &last_seq) < 0) break;

            pthread_mutex_lock(&ws->lock);
            float offset = ws->freq_offset;
            pthread_mutex_unlock(&ws->lock);

            char json[SPECTRUM_NFFT * 8 + 256];
            int pos = 0;
            pos += snprintf(json + pos, sizeof(json) - pos,
                            "{\"center\":%.0f,\"rate\":%.0f,\"rssi\":%.1f,"
                            "\"offset\":%.0f,\"psd\":[",
                            spectrum->center_hz,
                            spectrum->rate_hz,
                            spectrum->rssi_db,
                            offset);

            for (int i = 0; i < SPECTRUM_NFFT; i++) {
                pos += snprintf(json + pos, sizeof(json) - (size_t)pos,
                                i ? ",%.1f" : "%.1f", psd[i]);
            }
            pos += snprintf(json + pos, sizeof(json) - (size_t)pos, "]}");

            char frame[SPECTRUM_NFFT * 8 + 300];
            int flen = snprintf(frame, sizeof(frame), "data: %s\n\n", json);
            if (flen <= 0) break;
            if (sendall(fd, frame, (size_t)flen) < 0) break;
        }
    }

    /* --- GET /api/status --- */
    else if (is_get && strncmp(req + 4, "/api/status", 11) == 0) {
        pthread_mutex_lock(&ws->lock);
        int nch = ws->chmgr ? channel_mgr_n_channels(ws->chmgr) : 0;
        int maxch = ws->chmgr ? CHMGR_MAX_CHANNELS : 0;
        int scanning = ws->chmgr ? channel_mgr_scan_active(ws->chmgr) : 0;
        char json[512];
        snprintf(json, sizeof(json),
                 "{\"freq\":%u,\"gain\":%.1f,\"offset\":%.0f,\"rssi\":%.1f,"
                 "\"multi\":%s,\"channels\":%d,\"max_channels\":%d,"
                 "\"scanning\":%s}",
                 ws->target_freq, ws->gain_db, ws->freq_offset,
                 spectrum ? spectrum->rssi_db : 0.0f,
                 ws->chmgr ? "true" : "false", nch, maxch,
                 scanning ? "true" : "false");
        pthread_mutex_unlock(&ws->lock);
        send_json(fd, json);
    }

    /* --- POST /api/freq --- */
    else if (is_post && strncmp(req + 5, "/api/freq", 9) == 0) {
        double freq_hz = parse_form_value(body, "freq_hz");
        int ok = 0;
        char msg[128] = "";

        /* Block frequency retune in multi-channel mode — it would
         * invalidate all active demod NCO offsets. */
        if (ws->chmgr) {
            snprintf(msg, sizeof(msg), "Retuning disabled in multi-channel mode");
        } else if (freq_hz > 0 && (ws->src || ws->tcp)) {
            pthread_mutex_lock(&ws->lock);
            uint32_t new_target = (uint32_t)freq_hz;
            uint32_t rtl_freq = new_target - (uint32_t)ws->freq_offset;
            int rc = ws->tcp ? rtltcp_src_set_freq(ws->tcp, rtl_freq)
                             : rtlsdr_src_set_freq(ws->src, rtl_freq);
            if (rc == 0) {
                ws->target_freq = new_target;
                if (spectrum) spectrum->center_hz = (double)rtl_freq;
                ok = 1;
                fprintf(stderr, "[web] Tuned to %.3f MHz (center: %.3f MHz)\n",
                        new_target / 1e6, rtl_freq / 1e6);
            }
            pthread_mutex_unlock(&ws->lock);
        }
        char json[256];
        snprintf(json, sizeof(json), "{\"ok\":%s,\"msg\":\"%s\"}",
                 ok ? "true" : "false", msg);
        send_json(fd, json);
    }

    /* --- POST /api/gain --- */
    else if (is_post && strncmp(req + 5, "/api/gain", 9) == 0) {
        double gain_db = parse_form_value(body, "gain_db");
        int ok = 0;
        if (gain_db >= 0 && (ws->src || ws->tcp)) {
            int gain_tenths = (gain_db == 0.0) ? 0 : (int)(gain_db * 10.0 + 0.5);
            if (ws->tcp) rtltcp_src_set_gain(ws->tcp, gain_tenths);
            else         rtlsdr_src_set_gain(ws->src, gain_tenths);
            pthread_mutex_lock(&ws->lock);
            ws->gain_db = (float)gain_db;
            pthread_mutex_unlock(&ws->lock);
            ok = 1;
            fprintf(stderr, "[web] Gain set to %.1f dB\n", gain_db);
        }
        char json[64];
        snprintf(json, sizeof(json), "{\"ok\":%s}", ok ? "true" : "false");
        send_json(fd, json);
    }

    /* --- POST /api/channel (multi-channel: add a channel) --- */
    else if (is_post && strncmp(req + 5, "/api/channel", 12) == 0) {
        double freq_hz = parse_form_value(body, "freq_hz");
        int ok = 0;
        char msg[128] = "";
        if (freq_hz > 0 && ws->chmgr) {
            pthread_mutex_lock(&ws->lock);
            int nch = channel_mgr_n_channels(ws->chmgr);
            if (nch >= CHMGR_MAX_CHANNELS) {
                snprintf(msg, sizeof(msg), "Max %d channels", CHMGR_MAX_CHANNELS);
            } else {
                int idx = channel_mgr_add(ws->chmgr, (uint32_t)freq_hz);
                if (idx >= 0) {
                    ok = 1;
                    snprintf(msg, sizeof(msg), "Added ch%d at %.3f MHz",
                             idx, freq_hz / 1e6);
                } else {
                    snprintf(msg, sizeof(msg), "Duplicate or out of band");
                }
            }
            pthread_mutex_unlock(&ws->lock);
        } else if (!ws->chmgr) {
            snprintf(msg, sizeof(msg), "Not in multi-channel mode");
        }
        char json[256];
        snprintf(json, sizeof(json), "{\"ok\":%s,\"msg\":\"%s\","
                 "\"channels\":%d}",
                 ok ? "true" : "false", msg,
                 ws->chmgr ? channel_mgr_n_channels(ws->chmgr) : 0);
        send_json(fd, json);
    }

    /* --- 404 --- */
    else {
        send_str(fd,
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: 9\r\n"
            "Connection: close\r\n\r\n"
            "Not Found");
    }

    close(fd);
    return NULL;
}

/* ---- Server thread ------------------------------------------------------- */

static void *server_thread(void *arg) {
    web_server_t *ws = (web_server_t *)arg;

    while (ws->running) {
        struct sockaddr_in cli;
        socklen_t cli_len = sizeof(cli);
        int client_fd = accept(ws->listen_fd,
                               (struct sockaddr *)&cli, &cli_len);
        if (client_fd < 0) {
            if (ws->running) perror("[web] accept");
            break;
        }

        client_arg_t *ca = malloc(sizeof(*ca));
        if (!ca) { close(client_fd); continue; }
        ca->fd       = client_fd;
        ca->spectrum = ws->spectrum;
        ca->ws       = ws;

        pthread_t tid;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        if (pthread_create(&tid, &attr, handle_client, ca) != 0) {
            free(ca);
            close(client_fd);
        }
        pthread_attr_destroy(&attr);
    }
    return NULL;
}

/* ---- Public API ---------------------------------------------------------- */

web_server_t *web_server_create(int port, spectrum_t *spectrum,
                                 rtlsdr_src_t *src,
                                 uint32_t target_freq, float freq_offset) {
    web_server_t *ws = calloc(1, sizeof(*ws));
    if (!ws) return NULL;

    ws->port        = port;
    ws->spectrum    = spectrum;
    ws->src         = src;
    ws->target_freq = target_freq;
    ws->freq_offset = freq_offset;
    ws->gain_db     = 32.0f;
    pthread_mutex_init(&ws->lock, NULL);

    ws->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (ws->listen_fd < 0) {
        perror("[web] socket");
        pthread_mutex_destroy(&ws->lock);
        free(ws);
        return NULL;
    }

    int one = 1;
    setsockopt(ws->listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons((uint16_t)port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(ws->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[web] bind");
        close(ws->listen_fd);
        pthread_mutex_destroy(&ws->lock);
        free(ws);
        return NULL;
    }

    if (listen(ws->listen_fd, 8) < 0) {
        perror("[web] listen");
        close(ws->listen_fd);
        pthread_mutex_destroy(&ws->lock);
        free(ws);
        return NULL;
    }

    fprintf(stderr, "[web] Control panel: http://localhost:%d/\n", port);

    ws->running = 1;
    if (pthread_create(&ws->thread, NULL, server_thread, ws) != 0) {
        perror("[web] pthread_create");
        close(ws->listen_fd);
        pthread_mutex_destroy(&ws->lock);
        free(ws);
        return NULL;
    }

    return ws;
}

void web_server_destroy(web_server_t *ws) {
    if (!ws) return;
    ws->running = 0;
    shutdown(ws->listen_fd, SHUT_RDWR);
    close(ws->listen_fd);
    pthread_join(ws->thread, NULL);
    pthread_mutex_destroy(&ws->lock);
    free(ws);
}

void web_server_set_freq(web_server_t *ws, uint32_t target_freq) {
    if (!ws) return;
    pthread_mutex_lock(&ws->lock);
    ws->target_freq = target_freq;
    pthread_mutex_unlock(&ws->lock);
}

uint32_t web_server_get_freq(web_server_t *ws) {
    if (!ws) return 0;
    pthread_mutex_lock(&ws->lock);
    uint32_t f = ws->target_freq;
    pthread_mutex_unlock(&ws->lock);
    return f;
}

void web_server_set_channel_mgr(web_server_t *ws, channel_mgr_t *chmgr) {
    if (!ws) return;
    pthread_mutex_lock(&ws->lock);
    ws->chmgr = chmgr;
    pthread_mutex_unlock(&ws->lock);
}

void web_server_set_tcp_source(web_server_t *ws, rtltcp_src_t *tcp) {
    if (!ws) return;
    pthread_mutex_lock(&ws->lock);
    ws->tcp = tcp;
    pthread_mutex_unlock(&ws->lock);
}
