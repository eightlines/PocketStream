/**
 * @file rtsp.c
 * @brief Manual RTSP/RTP client for H.264 over TCP interleaved transport.
 *
 * Implements OPTIONS → DESCRIBE → SETUP → PLAY handshake, then receives
 * RTP/AVP/TCP interleaved frames (RFC 2326 §10.12) and depacketizes H.264
 * NAL units (RFC 6184: single NAL, STAP-A, FU-A).
 *
 * Notable LIVE555 v2014 quirks handled here:
 *   - RTP data arrives before the PLAY 200 OK response (s_play_buf).
 *   - FU-A last fragment has M=1 but E=0 in the FU header.
 *   - FU-A chains are completed via the next S=1 (implicit end) when
 *     neither M=1 nor E=1 arrive (e.g. stream stops mid-frame).
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "h264.h"
#include "rtsp.h"

static const char *TAG = "RTSP";

// ─── Configuration ────────────────────────────────────────────────────────────

#define RTSP_HOST       "10.0.0.1"
#define RTSP_PORT       8554   // TD direct (LIVE555 v2014); change to 8555 for FFmpeg relay
#define RTSP_PATH       "/live"
#define RTSP_URL        "rtsp://" RTSP_HOST ":" "8554" RTSP_PATH

#define RTP_BUFFER_SIZE 65536
#define NAL_BUFFER_SIZE 131072  // Buffer for reassembling fragmented NALs

// RTP inactivity timeout (ms). Must be long enough to span the GOP interval
// (keyframe period) so we don't reconnect before the first IDR arrives.
// TD/LIVE555 keyframe interval is unknown — 30s if conservative, 5s is a
// reasonable default.  Adjust upward if the stream consistently sends an IDR
// later than this (set to at least keyframe_interval + 2s).
#define RTP_INACTIVITY_TIMEOUT 5000

// ─── Module state ─────────────────────────────────────────────────────────────

// H.264 decoder config stored for reconnects
static h264_decoder_config_t s_decoder_cfg;

// RTSP connection state
static int  s_rtsp_sock   = -1;
static int  s_rtp_sock    = -1;  // unused in TCP mode; closed defensively on reconnect
static int  s_cseq        = 0;
static char s_session_id[64]  = {0};
static char s_track_path[128] = {0};

// SPS/PPS from SDP sprop-parameter-sets
static uint8_t s_sps[128];
static int     s_sps_len = 0;
static uint8_t s_pps[64];
static int     s_pps_len = 0;

// RTP depacketization state
static uint8_t  *s_nal_buffer        = NULL;
static int       s_nal_length        = 0;
static uint16_t  s_last_seq          = 0;
static bool      s_first_packet      = true;
static bool      s_got_idr           = false;  // module-level IDR gate
static bool      s_skip_until_fu_start = true;

// IDR multi-slice accumulation.
// Preserved across reconnects so a P-slice in a new session can flush
// the IDR that was received in the previous session.
static uint8_t  *s_idr_accum_buf  = NULL;
static int       s_idr_accum_len  = 0;
static int       s_idr_slice_cnt  = 0;
static uint32_t  s_idr_last_ms    = 0;
// True once the current decoder instance has been given a complete IDR.
// Reset on every reconnect / decoder re-init.
static bool      s_decoder_has_idr = false;

// LIVE555 v2014 sends RTP interleaved data *before* the PLAY 200 OK on the
// TCP connection.  rtsp_request() saves ALL bytes from that recv() here.
// buffered_read_exact() drains this before reading from the socket so the
// RTP receiver sees a clean byte stream.
static uint8_t s_play_buf[2048];
static int     s_play_buf_len    = 0;
static int     s_play_buf_offset = 0;

// RTP timing / task handles
static volatile uint32_t s_last_rtp_time  = 0;
static volatile bool     s_rtp_task_exit  = false;
static TaskHandle_t      s_rtp_task_handle       = NULL;
static TaskHandle_t      s_supervisor_task_handle = NULL;

// Static RTSP buffers (avoid stack overflow)
static char s_rtsp_request[512];
static char s_rtsp_response[2048];
static char s_rtsp_uri[256];

// ─── Forward declarations ─────────────────────────────────────────────────────

static void feed_sps_pps_to_decoder(void);
static void rtp_receiver_task(void *arg);
static void rtsp_supervisor_task(void *arg);

// ─── TCP helpers ──────────────────────────────────────────────────────────────

// Read exactly `len` bytes from a TCP socket, retrying on EAGAIN.
// Returns `len` on success, 0 on connection close, -1 on error.
// Checks s_rtp_task_exit on each EAGAIN so the task can be stopped.
static int tcp_read_exact(int sock, uint8_t *buf, int len)
{
    int total = 0;
    while (total < len) {
        if (s_rtp_task_exit) return -1;
        int n = recv(sock, buf + total, len - total, 0);
        if (n > 0) { total += n; continue; }
        if (n == 0) return 0;
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
        return -1;
    }
    return total;
}

// Read `len` bytes, draining s_play_buf first, then from the socket.
static int buffered_read_exact(int sock, uint8_t *buf, int len)
{
    int total = 0;
    if (s_play_buf_offset < s_play_buf_len) {
        int avail = s_play_buf_len - s_play_buf_offset;
        int take  = (avail < len) ? avail : len;
        memcpy(buf, s_play_buf + s_play_buf_offset, take);
        s_play_buf_offset += take;
        total += take;
    }
    if (total < len) {
        int n = tcp_read_exact(sock, buf + total, len - total);
        if (n <= 0) return n;
        total += n;
    }
    return total;
}

// ─── Base64 ───────────────────────────────────────────────────────────────────

static const uint8_t b64_table[256] = {
    ['A']=0,['B']=1,['C']=2,['D']=3,['E']=4,['F']=5,['G']=6,['H']=7,
    ['I']=8,['J']=9,['K']=10,['L']=11,['M']=12,['N']=13,['O']=14,['P']=15,
    ['Q']=16,['R']=17,['S']=18,['T']=19,['U']=20,['V']=21,['W']=22,['X']=23,
    ['Y']=24,['Z']=25,['a']=26,['b']=27,['c']=28,['d']=29,['e']=30,['f']=31,
    ['g']=32,['h']=33,['i']=34,['j']=35,['k']=36,['l']=37,['m']=38,['n']=39,
    ['o']=40,['p']=41,['q']=42,['r']=43,['s']=44,['t']=45,['u']=46,['v']=47,
    ['w']=48,['x']=49,['y']=50,['z']=51,['0']=52,['1']=53,['2']=54,['3']=55,
    ['4']=56,['5']=57,['6']=58,['7']=59,['8']=60,['9']=61,['+']=62,['/']=63,
};

static int base64_decode(const char *in, int in_len, uint8_t *out, int out_max)
{
    int out_len = 0;
    uint32_t accum = 0;
    int bits = 0;
    for (int i = 0; i < in_len && out_len < out_max; i++) {
        char c = in[i];
        if (c == '=' || c == '\0') break;
        if (c == ' ' || c == '\n' || c == '\r') continue;
        accum = (accum << 6) | b64_table[(uint8_t)c];
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out[out_len++] = (accum >> bits) & 0xFF;
        }
    }
    return out_len;
}

// ─── SDP parsing ──────────────────────────────────────────────────────────────

static void parse_sprop_from_sdp(const char *sdp)
{
    const char *sprop = strstr(sdp, "sprop-parameter-sets=");
    if (!sprop) { ESP_LOGW(TAG, "No sprop-parameter-sets in SDP"); return; }
    sprop += 21;

    const char *end = sprop;
    while (*end && *end != ';' && *end != '\r' && *end != '\n' && *end != ' ') end++;

    const char *comma = strchr(sprop, ',');
    if (!comma || comma > end) { ESP_LOGW(TAG, "Invalid sprop-parameter-sets format"); return; }

    int sps_b64_len = comma - sprop;
    ESP_LOGI(TAG, "SPS base64 (%d chars): %.*s", sps_b64_len, sps_b64_len, sprop);
    s_sps_len = base64_decode(sprop, sps_b64_len, s_sps, sizeof(s_sps));
    ESP_LOGI(TAG, "Decoded SPS: %d bytes", s_sps_len);
    if (s_sps_len >= 4) {
        ESP_LOGI(TAG, "SPS: nal=%02X profile=%d constraints=%02X level=%d",
                 s_sps[0], s_sps[1], s_sps[2], s_sps[3]);
        ESP_LOGI(TAG, "SPS hex: %02X %02X %02X %02X %02X %02X %02X %02X",
                 s_sps[0], s_sps[1], s_sps[2], s_sps[3],
                 s_sps[4], s_sps[5], s_sps[6], s_sps[7]);
    }

    const char *pps_start = comma + 1;
    int pps_b64_len = end - pps_start;
    ESP_LOGI(TAG, "PPS base64 (%d chars): %.*s", pps_b64_len, pps_b64_len, pps_start);
    s_pps_len = base64_decode(pps_start, pps_b64_len, s_pps, sizeof(s_pps));
    ESP_LOGI(TAG, "Decoded PPS: %d bytes", s_pps_len);
    if (s_pps_len >= 2) {
        ESP_LOGI(TAG, "PPS hex: %02X %02X %02X %02X",
                 s_pps[0], s_pps[1],
                 s_pps_len > 2 ? s_pps[2] : 0,
                 s_pps_len > 3 ? s_pps[3] : 0);
    }
}

static void parse_track_from_sdp(const char *sdp)
{
    const char *control = strstr(sdp, "a=control:");
    while (control) {
        control += 10;
        if (*control != '*') {
            char *end = strstr(control, "\r\n");
            if (!end) end = strstr(control, "\n");
            if (end) {
                int len = end - control;
                if (len < (int)sizeof(s_track_path) - 1) {
                    strncpy(s_track_path, control, len);
                    s_track_path[len] = '\0';
                    ESP_LOGI(TAG, "Found track: %s", s_track_path);
                    return;
                }
            }
        }
        control = strstr(control, "a=control:");
    }
}

static void parse_session(const char *response)
{
    const char *session = strstr(response, "Session:");
    if (!session) return;
    session += 8;
    while (*session == ' ') session++;
    char *end = strstr(session, ";");
    if (!end) end = strstr(session, "\r\n");
    if (end) {
        int len = end - session;
        if (len < (int)sizeof(s_session_id) - 1) {
            strncpy(s_session_id, session, len);
            s_session_id[len] = '\0';
            ESP_LOGI(TAG, "Session ID: %s", s_session_id);
        }
    }
}

// ─── RTSP handshake ───────────────────────────────────────────────────────────

static int rtsp_request(const char *method, const char *uri,
                        const char *extra_headers,
                        char *response, int response_size)
{
    int len = snprintf(s_rtsp_request, sizeof(s_rtsp_request),
        "%s %s RTSP/1.0\r\n"
        "CSeq: %d\r\n"
        "User-Agent: ESP32-RTSP-Manual\r\n"
        "%s%s%s"
        "\r\n",
        method, uri, s_cseq++,
        s_session_id[0] ? "Session: " : "",
        s_session_id[0] ? s_session_id : "",
        s_session_id[0] ? "\r\n" : "");

    if (extra_headers && extra_headers[0]) {
        len -= 2;
        len += snprintf(s_rtsp_request + len, sizeof(s_rtsp_request) - len,
                        "%s\r\n", extra_headers);
    }

    ESP_LOGI(TAG, "TX:\n%s", s_rtsp_request);
    if (send(s_rtsp_sock, s_rtsp_request, len, 0) < 0) {
        ESP_LOGE(TAG, "Failed to send request");
        return -1;
    }

    int received = recv(s_rtsp_sock, response, response_size - 1, 0);
    if (received <= 0) { ESP_LOGE(TAG, "Failed to receive response"); return -1; }
    response[received] = '\0';

    // LIVE555 v2014: RTP interleaved data arrives before the PLAY 200 OK.
    // Save ALL received bytes so buffered_read_exact() can replay them.
    if (response[0] == '$') {
        int save = (received <= (int)sizeof(s_play_buf)) ? received : (int)sizeof(s_play_buf);
        memcpy(s_play_buf, response, save);
        s_play_buf_len    = save;
        s_play_buf_offset = 0;
        ESP_LOGI(TAG, "RX: RTP before PLAY 200 OK — saved %d bytes "
                      "(hdr: %02X %02X %02X %02X)",
                 save,
                 (uint8_t)response[0], (uint8_t)response[1],
                 (uint8_t)response[2], (uint8_t)response[3]);
        return received;
    }

    ESP_LOGI(TAG, "RX:\n%s", response);
    if (strstr(response, "200 OK") == NULL) {
        ESP_LOGE(TAG, "Request failed");
        return -1;
    }
    return received;
}

static int rtsp_connect(void)
{
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(RTSP_PORT),
    };
    inet_pton(AF_INET, RTSP_HOST, &addr.sin_addr);

    s_rtsp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_rtsp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket, errno=%d", errno);
        return -1;
    }
    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(s_rtsp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s_rtsp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(s_rtsp_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to connect to %s:%d", RTSP_HOST, RTSP_PORT);
        close(s_rtsp_sock);
        s_rtsp_sock = -1;
        return -1;
    }
    ESP_LOGI(TAG, "Connected to RTSP server %s:%d", RTSP_HOST, RTSP_PORT);
    return 0;
}

static int rtsp_handshake(void)
{
    if (rtsp_request("OPTIONS", RTSP_URL, NULL,
                     s_rtsp_response, sizeof(s_rtsp_response)) < 0) return -1;

    if (rtsp_request("DESCRIBE", RTSP_URL, "Accept: application/sdp\r\n",
                     s_rtsp_response, sizeof(s_rtsp_response)) < 0) return -1;

    parse_track_from_sdp(s_rtsp_response);
    parse_sprop_from_sdp(s_rtsp_response);

    if (s_track_path[0] == '\0') {
        ESP_LOGE(TAG, "Failed to find track in SDP");
        return -1;
    }
    snprintf(s_rtsp_uri, sizeof(s_rtsp_uri), "%s/%s", RTSP_URL, s_track_path);

    // TCP interleaved: RTP/RTCP multiplexed on the existing RTSP socket.
    if (rtsp_request("SETUP", s_rtsp_uri,
                     "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n",
                     s_rtsp_response, sizeof(s_rtsp_response)) < 0) return -1;
    parse_session(s_rtsp_response);

    if (rtsp_request("PLAY", RTSP_URL, "Range: npt=0.000-\r\n",
                     s_rtsp_response, sizeof(s_rtsp_response)) < 0) return -1;

    ESP_LOGI(TAG, "RTSP handshake complete");
    return 0;
}

// ─── Decoder feeding ──────────────────────────────────────────────────────────

static void feed_sps_pps_to_decoder(void)
{
    static uint8_t nal_buf[256];
    esp_err_t ret;
    if (s_sps_len > 0) {
        nal_buf[0] = 0; nal_buf[1] = 0; nal_buf[2] = 0; nal_buf[3] = 1;
        memcpy(nal_buf + 4, s_sps, s_sps_len);
        ESP_LOGI(TAG, "Feeding SPS to decoder (%d bytes)", s_sps_len + 4);
        ret = h264_decoder_decode(nal_buf, s_sps_len + 4);
        ESP_LOGI(TAG, "SPS decode result: %d", ret);
    }
    if (s_pps_len > 0) {
        nal_buf[0] = 0; nal_buf[1] = 0; nal_buf[2] = 0; nal_buf[3] = 1;
        memcpy(nal_buf + 4, s_pps, s_pps_len);
        ESP_LOGI(TAG, "Feeding PPS to decoder (%d bytes)", s_pps_len + 4);
        ret = h264_decoder_decode(nal_buf, s_pps_len + 4);
        ESP_LOGI(TAG, "PPS decode result: %d", ret);
    }
}

static void feed_nal_to_decoder(uint8_t *nal, int len)
{
    if (len < 1) return;
    uint8_t nal_type = nal[0] & 0x1F;
    static int nal_count = 0;
    static int skipped_count = 0;
    nal_count++;

    static const char *nal_names[] = {
        "?","slice","DPA","DPB","DPC","IDR","SEI","SPS","PPS","AUD",
        "EOSEQ","EOSTREAM","FILL","13","14","15","16","17","18","19",
        "20","21","22","23","24","25","26","27","28","29","30","31"
    };
    const char *name = (nal_type <= 31) ? nal_names[nal_type] : "other";

    if (len < 4 && (nal_type == 1 || nal_type == 5)) {
        ESP_LOGW(TAG, "SKIP: NAL #%d type=%d (%s) len=%d too small",
                 nal_count, nal_type, name, len);
        return;
    }

    // Ensure SPS/PPS precede the first slice
    static bool sps_pps_fed = false;
    if ((nal_type == 1 || nal_type == 5) && !sps_pps_fed) {
        ESP_LOGW(TAG, "Forcing SPS/PPS feed before first slice");
        feed_sps_pps_to_decoder();
        sps_pps_fed = true;
    }
    if (nal_type == 7 || nal_type == 8) {
        sps_pps_fed = true;
        if (nal_type == 7) {
            // New SPS = new GOP — reset leftover IDR accumulation
            s_idr_accum_len = 0;
            s_idr_slice_cnt = 0;
        }
    }

    // Gate: only allow P-slices after we have seen an IDR in this call chain
    static bool got_idr = false;
    if (nal_type == 5) got_idr = true;
    if (nal_type == 1 && !got_idr) {
        ESP_LOGW(TAG, "SKIP: NAL #%d type=1 before first IDR", nal_count);
        return;
    }

    if (nal_type == 5 && !s_got_idr) {
        ESP_LOGI(TAG, "First IDR received!");
        s_got_idr = true;
    }

    if (!s_got_idr && nal_type >= 1 && nal_type <= 4) {
        skipped_count++;
        if (skipped_count % 10 == 1)
            ESP_LOGW(TAG, "Waiting for IDR, skipped %d slices", skipped_count);
        return;
    }

    // IDR slice: accumulate into buffer; flush when the first P-slice arrives.
    // The buffer is intentionally preserved across reconnects.
    if (nal_type == 5) {
        if (!s_idr_accum_buf) {
            s_idr_accum_buf = heap_caps_malloc(NAL_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!s_idr_accum_buf) s_idr_accum_buf = malloc(NAL_BUFFER_SIZE);
        }
        uint32_t now_ms = esp_log_timestamp();
        if (s_idr_accum_len > 0 && (now_ms - s_idr_last_ms) > 200) {
            ESP_LOGI(TAG, "New IDR frame — clearing old accum (%d bytes, %d slices)",
                     s_idr_accum_len, s_idr_slice_cnt);
            s_idr_accum_len = 0;
            s_idr_slice_cnt = 0;
        }
        s_idr_last_ms = now_ms;
        if (s_idr_accum_buf && s_idr_accum_len + len + 4 <= NAL_BUFFER_SIZE) {
            uint8_t *p = s_idr_accum_buf + s_idr_accum_len;
            p[0] = 0; p[1] = 0; p[2] = 0; p[3] = 1;
            memcpy(p + 4, nal, len);
            s_idr_accum_len += len + 4;
            s_idr_slice_cnt++;
            ESP_LOGI(TAG, "IDR accum: slice %d len=%d buf_total=%d",
                     s_idr_slice_cnt, len, s_idr_accum_len);
        } else {
            ESP_LOGE(TAG, "IDR accum buffer full/NULL, dropping slice");
        }
        return;
    }

    // P-slice: flush accumulated IDR first so the decoder sees a complete
    // multi-slice IDR access unit before the first inter-frame.
    if (nal_type == 1 && s_idr_accum_len > 0 && s_idr_accum_buf) {
        ESP_LOGI(TAG, "Flushing %d IDR slices (%d bytes) to decoder",
                 s_idr_slice_cnt, s_idr_accum_len);
        esp_err_t idr_ret = h264_decoder_decode(s_idr_accum_buf, s_idr_accum_len);
        ESP_LOGI(TAG, "IDR flush result: %d (0=OK)", idr_ret);
        s_idr_accum_len = 0;
        s_idr_slice_cnt = 0;
        if (idr_ret == ESP_OK) s_decoder_has_idr = true;
    }

    // Do not decode a P-slice until this decoder instance has a valid IDR.
    if (nal_type == 1 && !s_decoder_has_idr) {
        static int s_no_idr_skip = 0;
        s_no_idr_skip++;
        if (s_no_idr_skip % 50 == 1)
            ESP_LOGW(TAG, "SKIP P-slice: no IDR in current decoder (skipped %d)",
                     s_no_idr_skip);
        return;
    }

    // Add start code and decode
    static uint8_t nal_with_start[NAL_BUFFER_SIZE + 4];
    nal_with_start[0] = 0; nal_with_start[1] = 0;
    nal_with_start[2] = 0; nal_with_start[3] = 1;
    memcpy(nal_with_start + 4, nal, len);

    esp_err_t ret = h264_decoder_decode(nal_with_start, len + 4);
    if (ret != ESP_OK && (nal_type == 7 || nal_type == 8))
        ESP_LOGE(TAG, "Decoder error %d for NAL type %d", ret, nal_type);
}

// ─── RTP depacketization ──────────────────────────────────────────────────────

static void process_rtp_packet(uint8_t *data, int len)
{
    if (len < 12) return;

    uint8_t version   = (data[0] >> 6) & 0x03;
    uint8_t extension = (data[0] >> 4) & 0x01;
    uint8_t cc        = data[0] & 0x0F;
    uint16_t seq      = ((uint16_t)data[2] << 8) | data[3];
    // RTP Marker bit — LIVE555 v2014 sets M=1 on the last FU-A fragment
    // but does NOT reliably set E=1 in the FU header.
    uint8_t m_bit     = (data[1] >> 7) & 0x01;

    s_last_rtp_time = esp_log_timestamp();

    if (version != 2) { ESP_LOGW(TAG, "Invalid RTP version: %d", version); return; }

    int header_len = 12 + (cc * 4);
    if (extension) {
        if (len < header_len + 4) return;
        int ext_len = ((int)data[header_len + 2] << 8) | data[header_len + 3];
        header_len += 4 + (ext_len * 4);
    }
    if (len <= header_len) return;

    uint8_t *payload    = data + header_len;
    int      payload_len = len - header_len;

    // Sequence-number gap detection
    static int total_gaps = 0, total_lost = 0;
    if (!s_first_packet) {
        uint16_t expected = (s_last_seq + 1) & 0xFFFF;
        if (seq != expected) {
            int lost = (seq - expected) & 0xFFFF;
            total_gaps++;
            total_lost += lost;
            s_nal_length = 0;
            s_skip_until_fu_start = true;
            if (total_gaps % 50 == 1)
                ESP_LOGW(TAG, "Packet loss: %d gaps, %d lost total",
                         total_gaps, total_lost);
        }
    }
    s_first_packet = false;
    s_last_seq = seq;

    uint8_t nal_type = payload[0] & 0x1F;

    static int rtp_proc_count = 0;
    rtp_proc_count++;
    if (rtp_proc_count % 100 == 1)
        ESP_LOGI(TAG, "RTP #%d: nal_type=%d payload_len=%d",
                 rtp_proc_count, nal_type, payload_len);

    if (nal_type >= 1 && nal_type <= 23) {
        // Single NAL unit
        feed_nal_to_decoder(payload, payload_len);

    } else if (nal_type == 24) {
        // STAP-A: single-time aggregation
        int offset = 1;
        while (offset + 2 < payload_len) {
            uint16_t nal_size = ((uint16_t)payload[offset] << 8) | payload[offset + 1];
            offset += 2;
            if (offset + nal_size <= payload_len) {
                feed_nal_to_decoder(payload + offset, nal_size);
                offset += nal_size;
            } else break;
        }

    } else if (nal_type == 28) {
        // FU-A: fragmentation unit
        if (payload_len < 2) return;

        uint8_t fu_header    = payload[1];
        uint8_t start_bit    = (fu_header >> 7) & 0x01;
        uint8_t end_bit      = (fu_header >> 6) & 0x01;
        uint8_t real_nal_type = fu_header & 0x1F;

        static int fua_count = 0, fua_start_count = 0;
        static int fua_end_count = 0, fua_mbit_count = 0;
        fua_count++;
        if (start_bit) fua_start_count++;
        if (end_bit)   fua_end_count++;
        if (m_bit)     fua_mbit_count++;

        // Log chain starts and ends always; periodic summary every 100 packets.
        // Per-packet logging saturates UART at 115200 baud (~8.7 ms/line),
        // causing TCP receive-window backpressure that stalls LIVE555.
        if (start_bit) {
            ESP_LOGI(TAG, "FU-A chain start #%d: type=%d len=%d",
                     fua_start_count, real_nal_type, payload_len);
        } else if (end_bit || m_bit) {
            ESP_LOGI(TAG, "FU-A chain end: count=%d type=%d E=%d M=%d total_nal=%d",
                     fua_count, real_nal_type, end_bit, m_bit, s_nal_length);
        } else if (fua_count % 100 == 0) {
            ESP_LOGI(TAG, "FU-A summary: total=%d start=%d end=%d mbit=%d",
                     fua_count, fua_start_count, fua_end_count, fua_mbit_count);
        }

        if (s_skip_until_fu_start && !start_bit) return;

        if (start_bit) {
            // LIVE555 v2014 sets neither E nor M on the last fragment.
            // When the next chain's S=1 arrives, treat it as implicit end
            // of the previous chain and feed whatever was assembled.
            if (s_nal_length > 0) {
                uint8_t prev_type = s_nal_buffer[0] & 0x1F;
                ESP_LOGI(TAG, "FU-A implicit end: prev_type=%d len=%d, next_type=%d",
                         prev_type, s_nal_length, real_nal_type);
                feed_nal_to_decoder(s_nal_buffer, s_nal_length);
                s_nal_length = 0;
            }
            s_skip_until_fu_start = false;
            s_nal_length = 0;
            s_nal_buffer[0] = (payload[0] & 0xE0) | real_nal_type;
            s_nal_length = 1;
        }

        if (s_nal_length > 0 && s_nal_length + payload_len - 2 < NAL_BUFFER_SIZE) {
            memcpy(s_nal_buffer + s_nal_length, payload + 2, payload_len - 2);
            s_nal_length += payload_len - 2;
        }

        if (end_bit || m_bit) {
            if (s_nal_length > 0) {
                ESP_LOGI(TAG, "FU-A complete: type=%d total_len=%d (E=%d M=%d)",
                         real_nal_type, s_nal_length, end_bit, m_bit);
                feed_nal_to_decoder(s_nal_buffer, s_nal_length);
                s_nal_length = 0;
            }
        }

    } else {
        ESP_LOGW(TAG, "Unsupported NAL type: %d", nal_type);
    }
}

// ─── Interleaved RTP reader ───────────────────────────────────────────────────

// Read one complete RTP packet from the RTSP TCP connection using RFC 2326
// §10.12 framing: $ <channel> <len_hi> <len_lo> <data>.
// Returns RTP payload length (>0), 0 for RTCP/skip, -1 on error.
static int read_interleaved_rtp(uint8_t *buf, int buf_size)
{
    uint8_t hdr[4];
    int skip_count = 0;
    while (true) {
        if (buffered_read_exact(s_rtsp_sock, hdr, 1) <= 0) {
            if (skip_count > 0)
                ESP_LOGI(TAG, "Byte scanner: error after %d skipped bytes", skip_count);
            return -1;
        }
        if (hdr[0] == '$') break;
        skip_count++;
        if (skip_count == 1 || skip_count % 200 == 0)
            ESP_LOGI(TAG, "Byte scanner: skip #%d byte=0x%02X '%c'",
                     skip_count, hdr[0],
                     (hdr[0] >= 32 && hdr[0] < 127) ? hdr[0] : '.');
    }
    if (skip_count > 0)
        ESP_LOGI(TAG, "Byte scanner: found '$' after %d skipped bytes", skip_count);

    if (buffered_read_exact(s_rtsp_sock, hdr + 1, 3) <= 0) return -1;

    uint8_t  channel = hdr[1];
    uint16_t length  = ((uint16_t)hdr[2] << 8) | hdr[3];
    if (length == 0) return 0;

    if (length > buf_size) {
        ESP_LOGW(TAG, "Interleaved frame too large: %u bytes (ch %u)", length, channel);
        uint8_t discard[64];
        uint16_t rem = length;
        while (rem > 0) {
            int chunk = rem < (uint16_t)sizeof(discard) ? rem : (uint16_t)sizeof(discard);
            if (buffered_read_exact(s_rtsp_sock, discard, chunk) <= 0) return -1;
            rem -= chunk;
        }
        return 0;
    }

    if (buffered_read_exact(s_rtsp_sock, buf, length) <= 0) return -1;

    if (channel != 0) {
        ESP_LOGI(TAG, "RTCP packet: ch=%u len=%u (discarded)", channel, length);
        return 0;
    }
    return length;
}

// ─── RTP receiver task ────────────────────────────────────────────────────────

static void rtp_receiver_task(void *arg)
{
    uint8_t *buffer = heap_caps_malloc(RTP_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buffer) buffer = heap_caps_malloc(RTP_BUFFER_SIZE, MALLOC_CAP_8BIT);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate RTP buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP interleaved RTP receiver started");
    int packet_count = 0;

    while (!s_rtp_task_exit) {
        int n = read_interleaved_rtp(buffer, RTP_BUFFER_SIZE);
        if (s_rtp_task_exit) break;
        if (n < 0) { ESP_LOGE(TAG, "TCP recv error: errno=%d", errno); break; }
        if (n == 0) continue;

        packet_count++;
        if (packet_count % 100 == 0)
            ESP_LOGI(TAG, "TCP RTP packets: %d (len=%d)", packet_count, n);
        process_rtp_packet(buffer, n);
    }

    free(buffer);
    ESP_LOGI(TAG, "TCP receiver exited");
    s_rtp_task_handle = NULL;
    vTaskDelete(NULL);
}

// ─── Reconnect logic ──────────────────────────────────────────────────────────

static void do_reconnect(void)
{
    // Deinit decoder before closing socket (in case decode is in progress)
    h264_decoder_deinit();

    // Signal receiver task to exit
    if (s_rtp_task_handle) s_rtp_task_exit = true;

    // Close socket — immediately unblocks recv() in the receiver task
    if (s_rtsp_sock >= 0) { close(s_rtsp_sock); s_rtsp_sock = -1; }

    // Wait for receiver task to exit (should happen within a few ms)
    for (int i = 0; s_rtp_task_handle && i < 100; i++)
        vTaskDelay(pdMS_TO_TICKS(10));
    s_rtp_task_exit = false;

    // Close UDP socket if somehow set (TCP mode — defensive)
    if (s_rtp_sock >= 0) { close(s_rtp_sock); s_rtp_sock = -1; }

    // Reinitialize decoder with the stored config
    if (h264_decoder_init(&s_decoder_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinitialize H.264 decoder");
        return;
    }

    // Clear session + pre-buffer BEFORE handshake
    s_session_id[0] = '\0';
    s_play_buf_len = s_play_buf_offset = 0;

    vTaskDelay(pdMS_TO_TICKS(2000));

    if (rtsp_connect() < 0) {
        ESP_LOGE(TAG, "Reconnect failed");
        return;
    }
    if (rtsp_handshake() < 0) {
        ESP_LOGE(TAG, "Handshake failed after reconnect");
        if (s_rtsp_sock >= 0) { close(s_rtsp_sock); s_rtsp_sock = -1; }
        return;
    }
    feed_sps_pps_to_decoder();

    // Reset depacketizer state — do NOT clear s_idr_accum_buf/len/cnt.
    // An IDR accumulated in the previous session is still valid and will
    // be flushed when the first P-slice of the new session arrives.
    s_first_packet        = true;
    s_nal_length          = 0;
    s_skip_until_fu_start = true;
    s_got_idr             = false;
    s_decoder_has_idr     = false;

    // Restart receiver task
    s_rtp_task_exit = false;
    s_last_rtp_time = esp_log_timestamp();
    xTaskCreatePinnedToCore(rtp_receiver_task, "rtp_rx", 8192, NULL, 5,
                            &s_rtp_task_handle, 1);
    ESP_LOGI(TAG, "RTSP reconnection complete.");
}

// ─── Supervisor task ──────────────────────────────────────────────────────────

static void rtsp_supervisor_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = esp_log_timestamp();

        if (s_rtsp_sock < 0) {
            ESP_LOGW(TAG, "RTSP socket closed, reconnecting...");
            do_reconnect();
        } else if ((now - s_last_rtp_time) > RTP_INACTIVITY_TIMEOUT) {
            ESP_LOGW(TAG, "RTP inactivity timeout (%d ms), reconnecting...",
                     RTP_INACTIVITY_TIMEOUT);
            do_reconnect();
        }
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────

esp_err_t rtsp_client_start(const h264_decoder_config_t *decoder_cfg)
{
    s_decoder_cfg = *decoder_cfg;

    // Allocate NAL reassembly buffer
    s_nal_buffer = heap_caps_malloc(NAL_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_nal_buffer) s_nal_buffer = heap_caps_malloc(NAL_BUFFER_SIZE, MALLOC_CAP_8BIT);
    if (!s_nal_buffer) {
        ESP_LOGE(TAG, "Failed to allocate NAL buffer");
        return ESP_FAIL;
    }

    // Initialize H.264 decoder
    if (h264_decoder_init(&s_decoder_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize H.264 decoder");
        free(s_nal_buffer); s_nal_buffer = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Connecting to %s", RTSP_URL);
    ESP_LOGI(TAG, "===========================================");

    // Clear pre-buffer BEFORE handshake so rtsp_request() can populate it
    s_play_buf_len = s_play_buf_offset = 0;

    if (rtsp_connect() < 0) {
        ESP_LOGE(TAG, "Failed to connect to RTSP server");
        h264_decoder_deinit();
        free(s_nal_buffer); s_nal_buffer = NULL;
        return ESP_FAIL;
    }
    if (rtsp_handshake() < 0) {
        ESP_LOGE(TAG, "RTSP handshake failed");
        if (s_rtsp_sock >= 0) { close(s_rtsp_sock); s_rtsp_sock = -1; }
        h264_decoder_deinit();
        free(s_nal_buffer); s_nal_buffer = NULL;
        return ESP_FAIL;
    }
    feed_sps_pps_to_decoder();

    // Start RTP receiver (do NOT clear s_play_buf — handshake may have
    // populated it with the first RTP/RTCP header from the PLAY response)
    s_rtp_task_exit = false;
    s_last_rtp_time = esp_log_timestamp();
    xTaskCreatePinnedToCore(rtp_receiver_task, "rtp_rx", 8192, NULL, 8,
                            &s_rtp_task_handle, 1);

    // Start supervisor (handles inactivity timeout and reconnects)
    xTaskCreatePinnedToCore(rtsp_supervisor_task, "rtsp_super", 4096, NULL, 4,
                            &s_supervisor_task_handle, 0);

    ESP_LOGI(TAG, "RTSP client started.");
    return ESP_OK;
}

void rtsp_client_stop(void)
{
    // Stop supervisor task
    if (s_supervisor_task_handle) {
        vTaskDelete(s_supervisor_task_handle);
        s_supervisor_task_handle = NULL;
    }
    // Stop receiver task
    s_rtp_task_exit = true;
    if (s_rtsp_sock >= 0) { close(s_rtsp_sock); s_rtsp_sock = -1; }
    for (int i = 0; s_rtp_task_handle && i < 200; i++)
        vTaskDelay(pdMS_TO_TICKS(10));
    s_rtp_task_exit = false;

    if (s_rtp_sock >= 0) { close(s_rtp_sock); s_rtp_sock = -1; }

    free(s_nal_buffer); s_nal_buffer = NULL;
    h264_decoder_deinit();
    ESP_LOGI(TAG, "RTSP client stopped.");
}
