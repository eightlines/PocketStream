/**
 * @file main_rtsp_manual.c
 * @brief Manual RTSP client - implements handshake directly
 *
 * The esp_rtsp library fails during SETUP. This implementation
 * does the RTSP handshake manually via raw sockets, then receives
 * RTP packets and depacketizes H.264 NAL units.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "ethernet.h"
#include "display.h"
#include "h264.h"
#include "yuv_rgb.h"

static const char *TAG = "RTSP";

// RTSP Configuration
#define RTSP_HOST       "10.0.0.1"
#define RTSP_PORT       8554   // TD direct (LIVE555 v2014); change to 8555 for FFmpeg relay
#define RTSP_PATH       "/live"
#define RTSP_URL        "rtsp://" RTSP_HOST ":" "8554" RTSP_PATH

// Video configuration
#define VIDEO_WIDTH     128 // Default/fallback, not used after dynamic alloc
#define VIDEO_HEIGHT    128
#define DISPLAY_WIDTH   720
#define DISPLAY_HEIGHT  720

// RTP configuration
#define RTP_LOCAL_PORT  5004  // Client RTP port (RTCP on 5005)
#define RTP_BUFFER_SIZE 65536
#define NAL_BUFFER_SIZE 131072  // Buffer for reassembling fragmented NALs

// Display buffers
static uint16_t *s_rgb_buffer = NULL;
static uint16_t *s_display_buffer = NULL;
static uint32_t s_frame_count = 0;

// Track current input frame size
static uint16_t s_input_width = 0;
static uint16_t s_input_height = 0;

// RTP receiver task handle
static TaskHandle_t s_rtp_task_handle = NULL;

// RTP receiver task exit flag
static volatile bool s_rtp_task_exit = false;

// RTSP state
static int s_rtsp_sock = -1;
static int s_rtp_sock = -1;
static int s_cseq = 0;
static char s_session_id[64] = {0};
static char s_track_path[128] = {0};

// RTP depacketization state

static uint8_t *s_nal_buffer = NULL;
static int s_nal_length = 0;
static uint16_t s_last_seq = 0;
static bool s_first_packet = true;
static bool s_got_idr = false;  // Wait for IDR before decoding
static bool s_skip_until_fu_start = true;  // Skip partial NAL after sync

// IDR multi-slice accumulation: OBS encodes IDR frames as multiple slice NALs.
// Feeding them one-by-one causes "primary already decoded" errors because the
// decoder treats each IDR NAL as a new frame. Instead, accumulate all consecutive
// IDR slices and feed them as one concatenated buffer so the decoder sees a
// complete multi-slice access unit.
static uint8_t *s_idr_accum_buf = NULL;
static int      s_idr_accum_len = 0;
static int      s_idr_slice_cnt = 0;
// Timestamp of last accumulated IDR slice — used to detect when a new IDR
// frame starts vs. a continuation slice of the current frame (>200 ms gap
// means new frame, so the buffer is reset before appending).
static uint32_t s_idr_last_ms = 0;
// True once the current decoder instance has been given a complete IDR.
// Reset to false every time the decoder is re-initialized (on reconnect).
// P-slices are skipped until this becomes true.
static bool     s_decoder_has_idr = false;

// LIVE555 v2014 sends RTP interleaved data *before* the PLAY 200 OK on the TCP
// connection.  rtsp_request() stashes ALL bytes it receives with the PLAY
// response here.  buffered_read_exact() drains this buffer before reading from
// the socket, so the RTP receiver sees a clean byte stream regardless of how
// many payload bytes arrived together with the interleaved header.
static uint8_t s_play_buf[2048];
static int     s_play_buf_len    = 0;  // valid bytes in s_play_buf
static int     s_play_buf_offset = 0;  // next byte to consume

// RTP inactivity timeout (ms). Must be long enough to span the GOP interval
// (keyframe period) so we don't reconnect before the first IDR arrives.
// TD/LIVE555 keyframe interval is unknown — 30s if conservative, 5s is a
// reasonable default.  Adjust upward if the stream consistently sends an IDR
// later than this (set to at least keyframe_interval + 2s).
#define RTP_INACTIVITY_TIMEOUT 5000
static volatile uint32_t s_last_rtp_time = 0;

// Scale frame with aspect ratio preservation
static void scale_frame_fit_height(const uint16_t *src, uint16_t *dst,
                                   uint16_t src_w, uint16_t src_h,
                                   uint16_t dst_w, uint16_t dst_h)
{
    uint32_t scaled_w = (uint32_t)src_w * dst_h / src_h;
    int32_t x_offset = ((int32_t)scaled_w - dst_w) / 2;
    
    for (uint16_t y = 0; y < dst_h; y++) {
        uint16_t src_y = (y * src_h) / dst_h;
        for (uint16_t x = 0; x < dst_w; x++) {
            int32_t scaled_x = x + x_offset;
            if (scaled_x < 0) scaled_x = 0;
            if (scaled_x >= (int32_t)scaled_w) scaled_x = scaled_w - 1;
            
            uint16_t src_x = (scaled_x * src_w) / scaled_w;
            dst[y * dst_w + x] = src[src_y * src_w + src_x];
        }
    }
}

// H.264 frame decoded callback
static void on_frame_decoded(uint8_t *frame_data, uint16_t width, uint16_t height, uint32_t size, void *user_ctx)
{
    if (frame_data == NULL || size == 0) {
        return;
    }

    // Infer dimensions from I420 buffer size when decoder doesn't report them.
    // esp_h264_dec_out_frame_t has no width/height fields, so h264.c passes 0,0.
    // I420: size = W * H * 3/2  =>  W*H = size*2/3. For square frames: W=H=sqrt(W*H).
    if (width == 0 || height == 0) {
        uint32_t pixels = (size * 2) / 3;
        uint16_t side = (uint16_t)sqrtf((float)pixels);
        if (side > 0 && (uint32_t)side * side == pixels) {
            width = height = side;
        } else {
            // Non-square: fall back to last known dimensions
            width  = s_input_width  ? s_input_width  : 256;
            height = s_input_height ? s_input_height : 256;
        }
    }

    // Dynamic YUV buffer for decoded frame
    static uint8_t *s_yuv_buffer = NULL;
    static size_t s_yuv_size = 0;
    size_t required_yuv_size = width * height * 3 / 2;
    if (width != s_input_width || height != s_input_height || s_yuv_buffer == NULL || s_yuv_size != required_yuv_size) {
        if (s_yuv_buffer) {
            free(s_yuv_buffer);
            s_yuv_buffer = NULL;
        }
        s_yuv_buffer = malloc(required_yuv_size);
        s_yuv_size = required_yuv_size;
        if (!s_yuv_buffer) {
            ESP_LOGE(TAG, "Failed to allocate YUV buffer for %dx%d", width, height);
            return;
        }
    }
    // Use `size` (actual decoder output) not `required_yuv_size` to avoid buffer overread
    memcpy(s_yuv_buffer, frame_data, size);

    // If input resolution changed, reallocate RGB buffer and log detected resolution
    if (width != s_input_width || height != s_input_height || s_rgb_buffer == NULL) {
        ESP_LOGI(TAG, "Detected video resolution: %dx%d", width, height);
        if (s_rgb_buffer) {
            free(s_rgb_buffer);
            s_rgb_buffer = NULL;
        }
        size_t rgb_size = rgb565_buffer_size(width, height);
        s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rgb_buffer) s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_rgb_buffer) {
            ESP_LOGE(TAG, "Failed to allocate RGB buffer for %dx%d", width, height);
            return;
        }
        s_input_width = width;
        s_input_height = height;
        ESP_LOGI(TAG, "Allocated RGB buffer for %dx%d", width, height);
    }

    s_frame_count++;
    ESP_LOGI(TAG, "Frame #%lu decoded: %dx%d (%lu bytes)", (unsigned long)s_frame_count, width, height, (unsigned long)size);

    // Dump a simple checksum and a few Y values from the decoded frame
    uint32_t y_sum = 0;
    for (int i = 0; i < width * height; i += 31) y_sum += s_yuv_buffer[i];
    ESP_LOGI(TAG, "Y plane checksum: 0x%08lx, Y[0]=%u Y[1]=%u Y[100]=%u Y[last]=%u", (unsigned long)y_sum, s_yuv_buffer[0], s_yuv_buffer[1], s_yuv_buffer[100], s_yuv_buffer[width*height-1]);

    // Convert I420 to RGB565
    i420_to_rgb565(s_yuv_buffer, s_rgb_buffer, width, height);

    // Dump a simple checksum and a few RGB565 values
    uint32_t rgb_sum = 0;
    for (int i = 0; i < width * height; i += 31) rgb_sum += s_rgb_buffer[i];
    ESP_LOGI(TAG, "RGB565 checksum: 0x%08lx, RGB[0]=0x%04x RGB[1]=0x%04x RGB[100]=0x%04x RGB[last]=0x%04x", (unsigned long)rgb_sum, s_rgb_buffer[0], s_rgb_buffer[1], s_rgb_buffer[100], s_rgb_buffer[width*height-1]);

    // Draw to display
    if (width == DISPLAY_WIDTH && height == DISPLAY_HEIGHT) {
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_rgb_buffer);
    } else {
        scale_frame_fit_height(s_rgb_buffer, s_display_buffer, width, height, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_display_buffer);
    }
    display_wait_refresh_done();
}

// Static buffers for RTSP (avoid stack overflow)
static char s_rtsp_request[512];
static char s_rtsp_response[2048];
static char s_rtsp_uri[256];

// Send RTSP request and receive response
static int rtsp_request(const char *method, const char *uri, const char *extra_headers, char *response, int response_size)
{
    char *request = s_rtsp_request;
    int len = snprintf(request, sizeof(s_rtsp_request),
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
        // Insert extra headers before final \r\n
        len -= 2;
        len += snprintf(request + len, sizeof(s_rtsp_request) - len, "%s\r\n", extra_headers);
    }
    
    ESP_LOGI(TAG, "TX:\n%s", request);
    
    if (send(s_rtsp_sock, request, len, 0) < 0) {
        ESP_LOGE(TAG, "Failed to send request");
        return -1;
    }
    
    // Receive response
    int received = recv(s_rtsp_sock, response, response_size - 1, 0);
    if (received <= 0) {
        ESP_LOGE(TAG, "Failed to receive response");
        return -1;
    }
    response[received] = '\0';
    
    // LIVE555 v2014 sends the first RTP interleaved frame header on the TCP
    // connection before the PLAY 200 OK response.  Save those 4 bytes so the
    // RTP receiver task can read the first frame's payload cleanly.  After the
    // receiver reads that payload via tcp_read_exact(), the PLAY 200 OK text
    // will be next in the stream; the byte scanner in read_interleaved_rtp()
    // handles non-'$' bytes by skipping them, so it will resync on the next
    // real interleaved frame without any special treatment here.
    if (response[0] == '$') {
        // LIVE555 v2014: RTP interleaved data arrived before PLAY 200 OK.
        // Save ALL received bytes so buffered_read_exact() can replay them.
        // The recv() above may have returned the 4-byte interleaved header plus
        // some or all of the first RTP payload in one TCP segment — saving only
        // 4 bytes would silently discard the payload and desync the stream.
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
    
    // Check for 200 OK
    if (strstr(response, "200 OK") == NULL) {
        ESP_LOGE(TAG, "Request failed");
        return -1;
    }
    
    return received;
}

// Base64 decode table
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

// Base64 decode
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

// SPS/PPS storage
static uint8_t s_sps[128];
static int s_sps_len = 0;
static uint8_t s_pps[64];
static int s_pps_len = 0;

// Parse sprop-parameter-sets from SDP
static void parse_sprop_from_sdp(const char *sdp)
{
    const char *sprop = strstr(sdp, "sprop-parameter-sets=");
    if (!sprop) {
        ESP_LOGW(TAG, "No sprop-parameter-sets in SDP");
        return;
    }
        sprop += 21;  // Skip "sprop-parameter-sets="
    
    // Find end of parameter (next ; or \r\n or space)
    const char *end = sprop;
    while (*end && *end != ';' && *end != '\r' && *end != '\n' && *end != ' ') end++;
    
    // Parse comma-separated base64 values (SPS,PPS)
    const char *comma = strchr(sprop, ',');
    if (!comma || comma > end) {
        ESP_LOGW(TAG, "Invalid sprop-parameter-sets format");
        return;
    }
    
    // Decode SPS (first part)
    int sps_b64_len = comma - sprop;
    ESP_LOGI(TAG, "SPS base64 (%d chars): %.*s", sps_b64_len, sps_b64_len, sprop);
    s_sps_len = base64_decode(sprop, sps_b64_len, s_sps, sizeof(s_sps));
    ESP_LOGI(TAG, "Decoded SPS: %d bytes", s_sps_len);
        if (s_sps_len >= 3) {
            // uint8_t nal_type = s_sps[0] & 0x1F; // Unused, removed to fix warning
            uint8_t profile = s_sps[1];
            uint8_t constraints = s_sps[2];
            uint8_t level = (s_sps_len > 3) ? s_sps[3] : 0;
            ESP_LOGI(TAG, "SPS: nal=%02X profile=%d constraints=%02X level=%d", 
                     s_sps[0], profile, constraints, level);
            ESP_LOGI(TAG, "SPS hex: %02X %02X %02X %02X %02X %02X %02X %02X",
                     s_sps[0], s_sps[1], s_sps[2], s_sps[3], 
                     s_sps[4], s_sps[5], s_sps[6], s_sps[7]);
        }
    
    // Decode PPS (second part) 
    const char *pps_start = comma + 1;
    int pps_b64_len = end - pps_start;
    ESP_LOGI(TAG, "PPS base64 (%d chars): %.*s", pps_b64_len, pps_b64_len, pps_start);
    s_pps_len = base64_decode(pps_start, pps_b64_len, s_pps, sizeof(s_pps));
    ESP_LOGI(TAG, "Decoded PPS: %d bytes", s_pps_len);
    if (s_pps_len >= 2) {
        ESP_LOGI(TAG, "PPS hex: %02X %02X %02X %02X", 
                 s_pps[0], s_pps[1], s_pps_len > 2 ? s_pps[2] : 0, s_pps_len > 3 ? s_pps[3] : 0);
    }
}

// Feed SPS and PPS to decoder
static void feed_sps_pps_to_decoder(void)
{
    static uint8_t nal_buf[256];
    esp_err_t ret;
    
    if (s_sps_len > 0) {
        // Add start code + SPS
        nal_buf[0] = 0; nal_buf[1] = 0; nal_buf[2] = 0; nal_buf[3] = 1;
        memcpy(nal_buf + 4, s_sps, s_sps_len);
        ESP_LOGI(TAG, "Feeding SPS to decoder (%d bytes)", s_sps_len + 4);
        ESP_LOGI(TAG, "SPS first bytes: %02X %02X %02X %02X %02X", 
                 s_sps[0], s_sps[1], s_sps[2], s_sps[3], s_sps[4]);
        ret = h264_decoder_decode(nal_buf, s_sps_len + 4);
        ESP_LOGI(TAG, "SPS decode result: %d", ret);
    }
    
    if (s_pps_len > 0) {
        // Add start code + PPS
        nal_buf[0] = 0; nal_buf[1] = 0; nal_buf[2] = 0; nal_buf[3] = 1;
        memcpy(nal_buf + 4, s_pps, s_pps_len);
        ESP_LOGI(TAG, "Feeding PPS to decoder (%d bytes)", s_pps_len + 4);
        ESP_LOGI(TAG, "PPS first bytes: %02X %02X %02X %02X", 
                 s_pps[0], s_pps[1], s_pps_len > 2 ? s_pps[2] : 0, s_pps_len > 3 ? s_pps[3] : 0);
        ret = h264_decoder_decode(nal_buf, s_pps_len + 4);
        ESP_LOGI(TAG, "PPS decode result: %d", ret);
    }
}

// Parse track path from SDP
static void parse_track_from_sdp(const char *sdp)
{
    // Look for a=control:track1 (or similar)
    const char *control = strstr(sdp, "a=control:");
    while (control) {
        control += 10;  // Skip "a=control:"
        
        // Skip if it's the global control (*)
        if (*control != '*') {
            // Found track control
            char *end = strstr(control, "\r\n");
            if (!end) end = strstr(control, "\n");
            if (end) {
                int len = end - control;
                if (len < sizeof(s_track_path) - 1) {
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

// Parse session ID from SETUP response
static void parse_session(const char *response)
{
    const char *session = strstr(response, "Session:");
    if (session) {
        session += 8;
        while (*session == ' ') session++;
        
        char *end = strstr(session, ";");
        if (!end) end = strstr(session, "\r\n");
        if (end) {
            int len = end - session;
            if (len < sizeof(s_session_id) - 1) {
                strncpy(s_session_id, session, len);
                s_session_id[len] = '\0';
                ESP_LOGI(TAG, "Session ID: %s", s_session_id);
            }
        }
    }
}

// Connect to RTSP server
static int rtsp_connect(void)
{
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(RTSP_PORT),
    };
    inet_pton(AF_INET, RTSP_HOST, &server_addr.sin_addr);
    
    s_rtsp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_rtsp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create RTSP socket, errno=%d", errno);
        return -1;
    }
    
    // Set timeout
    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(s_rtsp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s_rtsp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    
    if (connect(s_rtsp_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to connect to RTSP server");
        close(s_rtsp_sock);
        return -1;
    }
    
    ESP_LOGI(TAG, "Connected to RTSP server %s:%d", RTSP_HOST, RTSP_PORT);
    return 0;
}

// Setup RTP receiver socket
static int rtp_setup(void)
{
    struct sockaddr_in rtp_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(RTP_LOCAL_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    
    s_rtp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_rtp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create RTP socket");
        return -1;
    }
    
    // Increase receive buffer to reduce packet loss
    int rcvbuf = 256 * 1024;  // 256KB
    int ret = setsockopt(s_rtp_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    if (ret < 0) {
        ESP_LOGW(TAG, "setsockopt SO_RCVBUF failed: errno=%d", errno);
    }

    // Check actual buffer size
    int actual_buf = 0;
    socklen_t optlen = sizeof(actual_buf);
    getsockopt(s_rtp_sock, SOL_SOCKET, SO_RCVBUF, &actual_buf, &optlen);
    ESP_LOGI(TAG, "RTP socket receive buffer requested: %d bytes, actual: %d bytes", rcvbuf, actual_buf);
    
    if (bind(s_rtp_sock, (struct sockaddr *)&rtp_addr, sizeof(rtp_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind RTP socket");
        close(s_rtp_sock);
        return -1;
    }
    
    // Set receive timeout
    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(s_rtp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    ESP_LOGI(TAG, "RTP socket bound to port %d", RTP_LOCAL_PORT);
    return 0;
}

// Perform RTSP handshake
static int rtsp_handshake(void)
{
    char *response = s_rtsp_response;
    char *uri = s_rtsp_uri;
    
    // OPTIONS
    if (rtsp_request("OPTIONS", RTSP_URL, NULL, response, sizeof(s_rtsp_response)) < 0) {
        return -1;
    }
    
    // DESCRIBE
    if (rtsp_request("DESCRIBE", RTSP_URL, "Accept: application/sdp\r\n", response, sizeof(s_rtsp_response)) < 0) {
        return -1;
    }
    
    // Parse track from SDP
    parse_track_from_sdp(response);
    
    // Extract SPS/PPS from sprop-parameter-sets
    parse_sprop_from_sdp(response);
    
    if (s_track_path[0] == '\0') {
        ESP_LOGE(TAG, "Failed to find track in SDP");
        return -1;
    }
    
    // Build track URI
    snprintf(uri, sizeof(s_rtsp_uri), "%s/%s", RTSP_URL, s_track_path);
    
    // SETUP with TCP interleaved transport.
    // RTP/RTCP are multiplexed over the existing RTSP TCP connection on
    // channels 0 and 1, framed as  $ <ch> <len_hi> <len_lo> <data>.
    // This eliminates the UDP packet loss that prevented IDR frames from
    // ever being fully assembled.
    const char *transport = "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n";

    if (rtsp_request("SETUP", uri, transport, response, sizeof(s_rtsp_response)) < 0) {
        return -1;
    }
    
    // Parse session ID
    parse_session(response);
    
    // PLAY (Session header added automatically by rtsp_request)
    if (rtsp_request("PLAY", RTSP_URL, "Range: npt=0.000-\r\n", response, sizeof(s_rtsp_response)) < 0) {
        return -1;
    }
    
    ESP_LOGI(TAG, "RTSP handshake complete - streaming should begin");
    return 0;
}

// Feed a complete NAL unit to decoder
static void feed_nal_to_decoder(uint8_t *nal, int len)
{
    if (len < 1) return;
    uint8_t nal_type = nal[0] & 0x1F;
    static int nal_count = 0;
    static int skipped_count = 0;
    nal_count++;



    const char *nal_names[] = {"?", "slice", "DPA", "DPB", "DPC", "IDR", "SEI", "SPS", "PPS", "AUD", "EOSEQ", "EOSTREAM", "FILL", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31"};
    const char *name = (nal_type <= 31) ? nal_names[nal_type] : "other";

    // Filter out truly malformed NALs (minimum slice header is ~4 bytes)
    if (len < 4 && (nal_type == 1 || nal_type == 5)) {
        ESP_LOGW(TAG, "SKIP: NAL #%d type=%d (%s) len=%d too small to be valid slice", nal_count, nal_type, name, len);
        return;
    }

    // Always feed SPS/PPS to decoder before any slices
    static bool sps_pps_fed = false;
    if ((nal_type == 1 || nal_type == 5) && !sps_pps_fed) {
        ESP_LOGW(TAG, "Forcing SPS/PPS feed before first slice");
        feed_sps_pps_to_decoder();
        sps_pps_fed = true;
    }
    if (nal_type == 7 || nal_type == 8) {
        sps_pps_fed = true;
        if (nal_type == 7) {
            // New SPS = new GOP - reset any leftover IDR accumulation
            s_idr_accum_len = 0;
            s_idr_slice_cnt = 0;
        }
    }

    // Only allow decoding after first IDR
    static bool got_idr = false;
    if (nal_type == 5) {
        got_idr = true;
    }
    if ((nal_type == 1) && !got_idr) {
        ESP_LOGW(TAG, "SKIP: NAL #%d type=1 (slice) before first IDR", nal_count);
        return;
    }

    // Track IDR reception
    if (nal_type == 5 && !s_got_idr) {
        ESP_LOGI(TAG, "First IDR received!");
        s_got_idr = true;
    }

    // Skip non-IDR slices until we get a valid IDR
    if (!s_got_idr && nal_type >= 1 && nal_type <= 4) {
        skipped_count++;
        if (skipped_count % 10 == 1) {
            ESP_LOGW(TAG, "Waiting for IDR, skipped %d slices", skipped_count);
        }
        return;
    }

    // IDR slice: accumulate all slices of this keyframe into one buffer.
    // They will be fed to the decoder together when the first P-slice arrives.
    // The buffer is preserved across reconnects so that a P-slice received in
    // a later session can still use a recently-accumulated IDR.
    if (nal_type == 5) {
        if (!s_idr_accum_buf) {
            s_idr_accum_buf = heap_caps_malloc(NAL_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!s_idr_accum_buf) s_idr_accum_buf = malloc(NAL_BUFFER_SIZE);
        }
        // Detect a new IDR frame: consecutive slices of the same frame arrive
        // within ~10 ms of each other; anything beyond 200 ms is a new frame.
        uint32_t now_ms = esp_log_timestamp();
        if (s_idr_accum_len > 0 && (now_ms - s_idr_last_ms) > 200) {
            ESP_LOGI(TAG, "New IDR frame detected — clearing old accum (%d bytes, %d slices)", s_idr_accum_len, s_idr_slice_cnt);
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
            ESP_LOGI(TAG, "IDR accum: slice %d len=%d buf_total=%d", s_idr_slice_cnt, len, s_idr_accum_len);
        } else {
            ESP_LOGE(TAG, "IDR accum buffer full/NULL, dropping slice");
        }
        return;  // Do not feed to decoder yet
    }

    // P-slice: flush all buffered IDR slices first so the decoder sees a
    // complete multi-slice IDR access unit before the first inter-frame.
    // The IDR buffer may have been accumulated in a previous RTSP session
    // (it is intentionally preserved across reconnects).
    if (nal_type == 1 && s_idr_accum_len > 0 && s_idr_accum_buf) {
        ESP_LOGI(TAG, "Flushing %d IDR slices (%d bytes) to decoder", s_idr_slice_cnt, s_idr_accum_len);
        esp_err_t idr_ret = h264_decoder_decode(s_idr_accum_buf, s_idr_accum_len);
        ESP_LOGI(TAG, "IDR flush result: %d (0=OK)", idr_ret);
        s_idr_accum_len = 0;
        s_idr_slice_cnt = 0;
        if (idr_ret == ESP_OK) {
            s_decoder_has_idr = true;
        }
    }

    // Do not decode a P-slice until the current decoder instance has a valid
    // IDR reference frame. Without this guard, P-slices decoded after a
    // reconnect (where decoder was re-initialized) produce no output because
    // TinyH264 has no reference frame yet.
    if (nal_type == 1 && !s_decoder_has_idr) {
        static int s_no_idr_skip = 0;
        s_no_idr_skip++;
        if (s_no_idr_skip % 50 == 1) {
            ESP_LOGW(TAG, "SKIP P-slice: no IDR reference in current decoder (skipped %d)", s_no_idr_skip);
        }
        return;
    }

    // Add start code and decode
    static uint8_t nal_with_start[NAL_BUFFER_SIZE + 4];
    nal_with_start[0] = 0;
    nal_with_start[1] = 0;
    nal_with_start[2] = 0;
    nal_with_start[3] = 1;
    memcpy(nal_with_start + 4, nal, len);

    esp_err_t ret = h264_decoder_decode(nal_with_start, len + 4);
    if (ret != ESP_OK && (nal_type == 7 || nal_type == 8)) {
        ESP_LOGE(TAG, "Decoder returned error %d for NAL type %d", ret, nal_type);
    }
}

// Process RTP packet and extract H.264 NAL units
// H.264 RTP payload format (RFC 6184)
static void process_rtp_packet(uint8_t *data, int len)
{
    uint8_t extension = (data[0] >> 4) & 0x01;
    uint8_t cc = data[0] & 0x0F;
    uint16_t seq = (data[2] << 8) | data[3];
    if (len < 12) return;  // RTP header is 12 bytes minimum
    // Update last RTP time for inactivity timeout
    s_last_rtp_time = esp_log_timestamp();

    uint8_t version = (data[0] >> 6) & 0x03;
    if (version != 2) {
        ESP_LOGW(TAG, "Invalid RTP version: %d", version);
        return;
    }

    int header_len = 12 + (cc * 4);
    if (extension) {
        if (len < header_len + 4) return;
        int ext_len = (data[header_len + 2] << 8) | data[header_len + 3];
        header_len += 4 + (ext_len * 4);
    }
    if (len <= header_len) return;

    uint8_t *payload = data + header_len;
    int payload_len = len - header_len;

    // RTP Marker bit (data[1] bit 7): marks the last RTP packet of each access unit.
    // RFC 6184: for FU-A, M=1 coincides with E=1 in a well-behaved encoder.
    // LIVE555 v2014 sets M=1 on the last FU-A fragment but does NOT reliably set E=1
    // in the FU header — so we use M as the chain-completion trigger.
    uint8_t m_bit = (data[1] >> 7) & 0x01;

    // Check for sequence number gaps (track but don't spam logs)
    static int total_gaps = 0;
    static int total_lost = 0;
    if (!s_first_packet) {
        uint16_t expected = (s_last_seq + 1) & 0xFFFF;
        if (seq != expected) {
            int lost = (seq - expected) & 0xFFFF;
            total_gaps++;
            total_lost += lost;
            // Reset NAL buffer on gap - wait for next clean start
            s_nal_length = 0;
            s_skip_until_fu_start = true;
            // Log only every 50 gaps
            if (total_gaps % 50 == 1) {
                ESP_LOGW(TAG, "Packet loss: %d gaps, %d packets lost total", total_gaps, total_lost);
            }
        }
    }
    s_first_packet = false;
    s_last_seq = seq;

    uint8_t nal_type = payload[0] & 0x1F;

    // Log RTP packet processing periodically
    static int rtp_proc_count = 0;
    rtp_proc_count++;
    if (rtp_proc_count % 100 == 1) {
        ESP_LOGI(TAG, "RTP processing #%d: nal_type=%d payload_len=%d", rtp_proc_count, nal_type, payload_len);
    }

    if (nal_type >= 1 && nal_type <= 23) {
        // Single NAL unit - feed directly
        feed_nal_to_decoder(payload, payload_len);
    }
    else if (nal_type == 24) {
        // STAP-A - Single-Time Aggregation Packet
        int offset = 1;  // Skip STAP-A header
        while (offset + 2 < payload_len) {
            uint16_t nal_size = (payload[offset] << 8) | payload[offset + 1];
            offset += 2;
            if (offset + nal_size <= payload_len) {
                feed_nal_to_decoder(payload + offset, nal_size);
                offset += nal_size;
            } else {
                break;
            }
        }
    }
    else if (nal_type == 28) {
        // FU-A - Fragmentation Unit
        if (payload_len < 2) return;

        uint8_t fu_header = payload[1];
        uint8_t start_bit = (fu_header >> 7) & 0x01;
        uint8_t end_bit = (fu_header >> 6) & 0x01;
        uint8_t real_nal_type = fu_header & 0x1F;

        static int fua_count = 0;
        static int fua_start_count = 0;
        static int fua_end_count = 0;
        static int fua_mbit_count = 0;
        fua_count++;
        if (start_bit) fua_start_count++;
        if (end_bit) fua_end_count++;
        if (m_bit) fua_mbit_count++;

        // Log chain starts and ends always; periodic summary every 100 packets.
        // Per-packet logging (even for first N packets) saturates UART at
        // 115200 baud (~8.7 ms/line), causing TCP receive-window backpressure
        // that stalls LIVE555 and makes the stream appear to stop.
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

        // Skip until we see a fresh start (handles sync mid-NAL)
        if (s_skip_until_fu_start && !start_bit) {
            return;  // Still looking for a clean start
        }

        if (start_bit) {
            // LIVE555 v2014 sets neither E bit nor M bit on the last FU-A fragment.
            // When the next chain's start arrives, treat it as the implicit end of
            // the previous chain — feed whatever was assembled so far.
            if (s_nal_length > 0) {
                uint8_t prev_type = s_nal_buffer[0] & 0x1F;
                ESP_LOGI(TAG, "FU-A implicit end: prev_type=%d len=%d, next_type=%d", prev_type, s_nal_length, real_nal_type);
                feed_nal_to_decoder(s_nal_buffer, s_nal_length);
                s_nal_length = 0;
            }
            // First fragment - start new NAL
            s_skip_until_fu_start = false;
            s_nal_length = 0;
            // Reconstruct NAL header
            s_nal_buffer[0] = (payload[0] & 0xE0) | real_nal_type;
            s_nal_length = 1;
        }

        // Append fragment data (skip FU indicator and FU header)
        if (s_nal_length > 0 && s_nal_length + payload_len - 2 < NAL_BUFFER_SIZE) {
            memcpy(s_nal_buffer + s_nal_length, payload + 2, payload_len - 2);
            s_nal_length += payload_len - 2;
        }

        // Complete the NAL on E bit (RFC 6184) or M bit (LIVE555 v2014 workaround).
        // LIVE555 v2014 sets M=1 on last fragment but often omits E=1 in FU header.
        if (end_bit || m_bit) {
            if (s_nal_length > 0) {
                ESP_LOGI(TAG, "FU-A complete: type=%d, total_len=%d (E=%d M=%d)", real_nal_type, s_nal_length, end_bit, m_bit);
                feed_nal_to_decoder(s_nal_buffer, s_nal_length);
                s_nal_length = 0;
            }
        }
    }
    else {
        ESP_LOGW(TAG, "Unsupported NAL type: %d", nal_type);
    }
}

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
        if (n == 0) return 0;  // Connection closed
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  // Timeout, retry
        return -1;  // Real socket error
    }
    return total;
}

// Read `len` bytes, draining s_play_buf (pre-received PLAY-response data) first,
// then continuing from the socket.  This ensures any payload bytes that arrived
// in the same TCP segment as the PLAY response are not silently discarded.
static int buffered_read_exact(int sock, uint8_t *buf, int len)
{
    int total = 0;
    // 1. Drain pre-buffered bytes.
    if (s_play_buf_offset < s_play_buf_len) {
        int avail = s_play_buf_len - s_play_buf_offset;
        int take  = (avail < len) ? avail : len;
        memcpy(buf, s_play_buf + s_play_buf_offset, take);
        s_play_buf_offset += take;
        total += take;
    }
    // 2. Read the rest from the socket.
    if (total < len) {
        int n = tcp_read_exact(sock, buf + total, len - total);
        if (n <= 0) return n;  // closed or error
        total += n;
    }
    return total;
}

// Read one complete RTP packet from the RTSP TCP connection using the
// RTSP interleaved binary framing (RFC 2326 §10.12):
//   $ <1-byte channel> <2-byte big-endian length> <RTP or RTCP data>
// Returns the RTP payload length (>0), 0 for RTCP/skip, or -1 on error.
static int read_interleaved_rtp(uint8_t *buf, int buf_size)
{
    uint8_t hdr[4];

    // Scan for the '$' (0x24) interleaved-frame magic byte.
    // Non-'$' bytes may be PLAY 200 OK text or RTSP keepalive responses —
    // skip them.  buffered_read_exact() drains any bytes pre-received with
    // the PLAY response before reading from the socket.
    int skip_count = 0;
    while (true) {
        if (buffered_read_exact(s_rtsp_sock, hdr, 1) <= 0) {
            if (skip_count > 0)
                ESP_LOGI(TAG, "Byte scanner: read error after skipping %d bytes", skip_count);
            return -1;
        }
        if (hdr[0] == '$') break;
        skip_count++;
        if (skip_count == 1 || skip_count % 200 == 0) {
            ESP_LOGI(TAG, "Byte scanner: skip #%d byte=0x%02X '%c'",
                     skip_count, hdr[0], (hdr[0] >= 32 && hdr[0] < 127) ? hdr[0] : '.');
        }
    }
    if (skip_count > 0) {
        ESP_LOGI(TAG, "Byte scanner: found '$' after %d skipped bytes", skip_count);
    }

    if (buffered_read_exact(s_rtsp_sock, hdr + 1, 3) <= 0) return -1;

    uint8_t  channel = hdr[1];
    uint16_t length  = ((uint16_t)hdr[2] << 8) | hdr[3];

    if (length == 0) return 0;

    if (length > buf_size) {
        // Frame too large for our buffer — discard it.
        ESP_LOGW(TAG, "Interleaved frame too large: %u bytes (ch %u)", length, channel);
        uint8_t discard[64];
        uint16_t rem = length;
        while (rem > 0) {
            int chunk = rem < (uint16_t)sizeof(discard) ? rem : sizeof(discard);
            if (buffered_read_exact(s_rtsp_sock, discard, chunk) <= 0) return -1;
            rem -= chunk;
        }
        return 0;
    }

    if (buffered_read_exact(s_rtsp_sock, buf, length) <= 0) return -1;

    // Channel 0 = RTP video, channel 1 = RTCP — we only process RTP.
    if (channel != 0) {
        ESP_LOGI(TAG, "RTCP packet: ch=%u len=%u (discarded)", channel, length);
        return 0;
    }
    return length;
}

// TCP interleaved RTP receiver task
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
        if (n < 0) {
            ESP_LOGE(TAG, "TCP recv error: errno=%d", errno);
            break;
        }
        if (n == 0) continue;  // RTCP or skipped frame

        packet_count++;
        if (packet_count <= 50 || packet_count % 100 == 0) {
            ESP_LOGI(TAG, "TCP RTP packets: %d (len=%d)", packet_count, n);
        }
        process_rtp_packet(buffer, n);
    }

    free(buffer);
    ESP_LOGI(TAG, "TCP receiver exited");
    s_rtp_task_handle = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Ethernet
    init_ethernet();
    ESP_LOGI(TAG, "Waiting for IP...");
    xEventGroupWaitBits(s_eth_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Initialize display
    ESP_LOGI(TAG, "Initialize LCD device");
    init_display();
    
    // Allocate buffers
    // Only allocate display buffer and NAL buffer here; RGB buffer is dynamic
    size_t display_size = rgb565_buffer_size(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_display_buffer) s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    // Allocate NAL reassembly buffer
    s_nal_buffer = heap_caps_malloc(NAL_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_nal_buffer) s_nal_buffer = heap_caps_malloc(NAL_BUFFER_SIZE, MALLOC_CAP_8BIT);
    if (!s_nal_buffer) {
        ESP_LOGE(TAG, "Failed to allocate NAL buffer");
        return;
    }

    ESP_LOGI(TAG, "Buffers allocated (display: %dx%d)", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    
    // Initialize H.264 decoder
    h264_decoder_config_t h264_cfg = {
        .on_frame_decoded = on_frame_decoded,
        .user_ctx = NULL,
    };
    ret = h264_decoder_init(&h264_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize H.264 decoder");
        return;
    }
    ESP_LOGI(TAG, "H.264 decoder initialized");
    
    // Connect to RTSP server and perform handshake
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Connecting to RTSP server...");
    ESP_LOGI(TAG, "URL: %s", RTSP_URL);
    ESP_LOGI(TAG, "===========================================");
    
    // Clear pre-buffer BEFORE handshake so rtsp_request() can populate it from
    // the PLAY response.  If cleared after, the saved bytes are thrown away and
    // the byte scanner has to skip the RTCP body + PLAY 200 OK text instead.
    s_play_buf_len = s_play_buf_offset = 0;
    if (rtsp_connect() < 0) {
        ESP_LOGE(TAG, "Failed to connect to RTSP server");
        return;
    }

    if (rtsp_handshake() < 0) {
        ESP_LOGE(TAG, "RTSP handshake failed");
        close(s_rtsp_sock);
        return;
    }

    // Feed SPS/PPS to decoder before receiving any frames
    feed_sps_pps_to_decoder();

    // Start RTP receiver task (do NOT clear s_play_buf here — handshake may have
    // populated it with the first RTP/RTCP interleaved header from the PLAY response)
    s_rtp_task_exit = false;
    s_last_rtp_time = esp_log_timestamp();
    xTaskCreatePinnedToCore(rtp_receiver_task, "rtp_rx", 8192, NULL, 8, &s_rtp_task_handle, 1);

    // Main loop - monitor status and handle reconnection
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Frames decoded: %lu", (unsigned long)s_frame_count);

        // Check for RTP inactivity timeout or TCP receiver task exit
        uint32_t now = esp_log_timestamp();
        bool reconnect_needed = false;
        if (s_rtsp_sock < 0) {
            ESP_LOGW(TAG, "RTSP socket closed, attempting to reconnect...");
            reconnect_needed = true;
        } else if ((now - s_last_rtp_time) > RTP_INACTIVITY_TIMEOUT) {
            ESP_LOGW(TAG, "RTP inactivity timeout (no packets for %d ms), reconnecting...", RTP_INACTIVITY_TIMEOUT);
            reconnect_needed = true;
        }

        if (reconnect_needed) {
            // Clean up decoder and buffers if needed
            h264_decoder_deinit();
            // Signal the RTP task to stop.
            if (s_rtp_task_handle != NULL) {
                s_rtp_task_exit = true;
            }
            // Close the RTSP socket BEFORE waiting for the task.
            // The TCP interleaved receiver blocks inside recv(s_rtsp_sock).
            // Closing the socket immediately unblocks that call so the task
            // can exit instead of waiting up to SO_RCVTIMEO seconds.
            if (s_rtsp_sock >= 0) {
                close(s_rtsp_sock);
                s_rtsp_sock = -1;
            }
            // Now wait for the task to exit (should happen within a few ms).
            if (s_rtp_task_handle != NULL) {
                int wait_count = 0;
                while (s_rtp_task_handle != NULL && wait_count++ < 100) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                s_rtp_task_exit = false;
            }
            // s_rtp_sock is unused (TCP mode) but close if somehow set.
            if (s_rtp_sock >= 0) {
                close(s_rtp_sock);
                s_rtp_sock = -1;
            }
            // Re-initialize decoder
            h264_decoder_config_t h264_cfg = {
                .on_frame_decoded = on_frame_decoded,
                .user_ctx = NULL,
            };
            ret = h264_decoder_init(&h264_cfg);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reinitialize H.264 decoder");
                continue;
            }
            // Clear Session ID and pre-buffer BEFORE handshake.
            s_session_id[0] = '\0';
            s_play_buf_len = s_play_buf_offset = 0;  // clear stale bytes from previous session
            // Attempt to reconnect
            vTaskDelay(pdMS_TO_TICKS(2000));
            if (rtsp_connect() < 0) {
                ESP_LOGE(TAG, "Failed to reconnect to RTSP server");
                continue;
            }
            if (rtsp_handshake() < 0) {
                ESP_LOGE(TAG, "RTSP handshake failed after reconnect");
                if (s_rtsp_sock >= 0) {
                    close(s_rtsp_sock);
                    s_rtsp_sock = -1;
                }
                continue;
            }
            feed_sps_pps_to_decoder();
            // Reset RTP depacketizer state so the new session doesn't generate a
            // massive false sequence-number gap against the old session's last seq.
            s_first_packet = true;
            s_nal_length = 0;
            s_skip_until_fu_start = true;
            s_got_idr = false;
            // Intentionally do NOT clear s_idr_accum_len / s_idr_slice_cnt.
            // Any IDR accumulated before the reconnect is still valid for the
            // same stream and can be used with the freshly initialized decoder
            // so the first P-slice after reconnect gets a reference frame.
            s_decoder_has_idr = false;  // New decoder instance needs IDR before P-slices
            // Restart RTP receiver task (do NOT clear s_play_buf here)
            s_rtp_task_exit = false;
            s_last_rtp_time = esp_log_timestamp();
            xTaskCreatePinnedToCore(rtp_receiver_task, "rtp_rx", 8192, NULL, 5, &s_rtp_task_handle, 1);
            ESP_LOGI(TAG, "RTSP reconnection complete.");
        }
    }
}
