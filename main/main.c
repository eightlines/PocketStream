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
#define RTSP_PORT       8554
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

// RTP inactivity timeout (ms)
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
    
    // Check if we got interleaved data instead of RTSP response
    // This can happen after PLAY when server starts streaming immediately
    if (response[0] == '$') {
        ESP_LOGI(TAG, "RX: Interleaved data started (streaming began!) - %d bytes", received);
        // Note: This data is lost but the receiver will pick up the next packets
        // We could buffer it but it adds complexity. Just let it resync.
        return received;  // Return positive to indicate success
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
    
    // SETUP with TCP interleaved transport (no UDP packet loss!)
    char transport[128];
    snprintf(transport, sizeof(transport), "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
    
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



    // Log all NAL types and lengths
    const char *nal_names[] = {"?", "slice", "DPA", "DPB", "DPC", "IDR", "SEI", "SPS", "PPS", "AUD", "EOSEQ", "EOSTREAM", "FILL", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31"};
    const char *name = (nal_type <= 31) ? nal_names[nal_type] : "other";
    ESP_LOGI(TAG, "NAL #%d: type=%d (%s), len=%d", nal_count, nal_type, name, len);

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

    ESP_LOGI(TAG, "FEED_NAL: #%d type=%d len=%d first8=%02X %02X %02X %02X %02X %02X %02X %02X", nal_count, nal_type, len,
        nal[0], len>1?nal[1]:0, len>2?nal[2]:0, len>3?nal[3]:0, len>4?nal[4]:0, len>5?nal[5]:0, len>6?nal[6]:0, len>7?nal[7]:0);

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

    // Log NAL info periodically or for important types
    if (nal_type == 7 || nal_type == 8 || nal_type == 5 || nal_type == 1 || nal_count % 50 == 1) {
        const char *nal_names[] = {"?", "slice", "DPA", "DPB", "DPC", "IDR", "SEI", "SPS", "PPS"};
        const char *name = (nal_type <= 8) ? nal_names[nal_type] : "other";
        ESP_LOGI(TAG, "NAL #%d: type=%d (%s), len=%d", nal_count, nal_type, name, len);
        // Dump first 16 bytes of raw NAL for type 1 and 5
        if ((nal_type == 1 || nal_type == 5) && len >= 16) {
            ESP_LOGI(TAG, "NAL #%d: type=%d first16: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                nal_count, nal_type,
                nal[0], nal[1], nal[2], nal[3], nal[4], nal[5], nal[6], nal[7],
                nal[8], nal[9], nal[10], nal[11], nal[12], nal[13], nal[14], nal[15]);
        }
    }

    // IDR slice: accumulate all slices of this keyframe into one buffer.
    // They will be fed to the decoder together when the first P-slice arrives.
    if (nal_type == 5) {
        if (!s_idr_accum_buf) {
            s_idr_accum_buf = heap_caps_malloc(NAL_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!s_idr_accum_buf) s_idr_accum_buf = malloc(NAL_BUFFER_SIZE);
        }
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
    if (nal_type == 1 && s_idr_accum_len > 0 && s_idr_accum_buf) {
        ESP_LOGI(TAG, "Flushing %d IDR slices (%d bytes) to decoder", s_idr_slice_cnt, s_idr_accum_len);
        esp_err_t idr_ret = h264_decoder_decode(s_idr_accum_buf, s_idr_accum_len);
        ESP_LOGI(TAG, "IDR flush result: %d (0=OK)", idr_ret);
        s_idr_accum_len = 0;
        s_idr_slice_cnt = 0;
    }

    // Add start code and decode
    static uint8_t nal_with_start[NAL_BUFFER_SIZE + 4];
    nal_with_start[0] = 0;
    nal_with_start[1] = 0;
    nal_with_start[2] = 0;
    nal_with_start[3] = 1;
    memcpy(nal_with_start + 4, nal, len);

    esp_err_t ret = h264_decoder_decode(nal_with_start, len + 4);
    ESP_LOGI(TAG, "h264_decoder_decode() returned %d for NAL type %d, len %d", ret, nal_type, len);
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
        ESP_LOGI(TAG, "RTP NAL: type=%d len=%d", nal_type, payload_len);
        feed_nal_to_decoder(payload, payload_len);
    }
    else if (nal_type == 24) {
        // STAP-A - Single-Time Aggregation Packet
        ESP_LOGI(TAG, "STAP-A packet, len=%d", payload_len);
        int offset = 1;  // Skip STAP-A header
        while (offset + 2 < payload_len) {
            uint16_t nal_size = (payload[offset] << 8) | payload[offset + 1];
            offset += 2;
            if (offset + nal_size <= payload_len) {
                uint8_t stap_nal_type = payload[offset] & 0x1F;
                ESP_LOGI(TAG, "STAP-A NAL: type=%d len=%d", stap_nal_type, nal_size);
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
        fua_count++;
        if (start_bit) fua_start_count++;
        if (end_bit) fua_end_count++;

        // Log every 100 packets with summary
        if (fua_count % 100 == 1) {
            ESP_LOGI(TAG, "FU-A summary: total=%d start=%d end=%d", fua_count, fua_start_count, fua_end_count);
        }

        if (start_bit) {
            ESP_LOGI(TAG, "FU-A START: type=%d", real_nal_type);
        }
        if (end_bit) {
            ESP_LOGI(TAG, "FU-A END: type=%d", real_nal_type);
        }

        // Skip until we see a fresh start (handles sync mid-NAL)
        if (s_skip_until_fu_start && !start_bit) {
            return;  // Still looking for a clean start
        }

        if (start_bit) {
            // First fragment - start new NAL
            s_skip_until_fu_start = false;  // Found a clean start
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

        if (end_bit) {
            ESP_LOGI(TAG, "FU-A END received: type=%d, nal_len=%d", real_nal_type, s_nal_length);
            if (s_nal_length > 0) {
                // Last fragment - feed complete NAL
                ESP_LOGI(TAG, "FU-A complete: type=%d, total_len=%d", real_nal_type, s_nal_length);
                feed_nal_to_decoder(s_nal_buffer, s_nal_length);
                s_nal_length = 0;
            }
        }
    }
    else {
        ESP_LOGW(TAG, "Unsupported NAL type: %d", nal_type);
    }
}

// Read exact number of bytes from TCP socket
static int tcp_read_exact(int sock, uint8_t *buf, int len)
{
    int total = 0;
    while (total < len) {
        int n = recv(sock, buf + total, len - total, 0);
        if (n <= 0) return n;
        total += n;
    }
    return total;
}

// TCP interleaved RTP receiver task
// Reads RTP packets from RTSP TCP connection (interleaved mode)
static void rtp_receiver_task(void *arg)
{
    uint8_t *buffer = heap_caps_malloc(RTP_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buffer) {
        buffer = heap_caps_malloc(RTP_BUFFER_SIZE, MALLOC_CAP_8BIT);
    }
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate RTP buffer");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "TCP interleaved RTP receiver started");

    // Set socket to blocking with longer timeout
    struct timeval tv = { .tv_sec = 30, .tv_usec = 0 };
    setsockopt(s_rtsp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int packet_count = 0;
    uint8_t header[4];
    int n;

    // First, sync to '$' marker (may have lost bytes from PLAY response)
    ESP_LOGI(TAG, "Syncing to interleaved frames...");
    uint8_t sync_byte;
    int sync_count = 0;
    while (sync_count < 10000 && !s_rtp_task_exit) {
        n = recv(s_rtsp_sock, &sync_byte, 1, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "Sync failed: read error");
            free(buffer);
            s_rtp_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
        sync_count++;
        if (sync_byte == '$') {
            ESP_LOGI(TAG, "Synced after %d bytes", sync_count);
            header[0] = '$';
            // Read rest of header
            n = tcp_read_exact(s_rtsp_sock, header + 1, 3);
            if (n != 3) {
                ESP_LOGE(TAG, "Failed to read header");
                continue;
            }
            break;
        }
    }

    while (!s_rtp_task_exit) {
        // We have header[0..3] = $ + channel + length
        uint8_t channel = header[1];
        uint16_t length = (header[2] << 8) | header[3];
        
        if (length > RTP_BUFFER_SIZE) {
            ESP_LOGE(TAG, "Interleaved packet too large: %d", length);
            // Try to resync
            goto resync;
        }
        
        // Read RTP packet data
        n = tcp_read_exact(s_rtsp_sock, buffer, length);
        if (s_rtp_task_exit) break;
        if (n != length) {
            ESP_LOGE(TAG, "Failed to read interleaved data");
            break;
        }
        
        // Channel 0 = RTP, Channel 1 = RTCP
        if (channel == 0) {
            packet_count++;
            if (packet_count % 100 == 1) {
                ESP_LOGI(TAG, "TCP RTP packets: %d (len=%d)", packet_count, length);
            }
            process_rtp_packet(buffer, length);
        } else if (channel == 1) {
            // Log RTCP packets
            static int rtcp_count = 0;
            rtcp_count++;
            if (rtcp_count % 10 == 1) {
                ESP_LOGI(TAG, "RTCP packet #%d (len=%d)", rtcp_count, length);
            }
        }
        
        // Read next header
        n = tcp_read_exact(s_rtsp_sock, header, 4);
        if (s_rtp_task_exit) break;
        if (n != 4) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                ESP_LOGW(TAG, "TCP receive timeout - no data for 10s");
                // Try to recover by waiting for more data
                n = tcp_read_exact(s_rtsp_sock, header, 4);
                if (s_rtp_task_exit) break;
                if (n != 4) {
                    ESP_LOGE(TAG, "Second timeout - connection dead");
                    break;
                }
            } else {
                ESP_LOGE(TAG, "TCP read error: %d", errno);
                break;
            }
        }
        
        // Check for '$' interleaved marker
        if (header[0] != '$') {
resync:
            ESP_LOGW(TAG, "Lost sync, resyncing... (got 0x%02X)", header[0]);
            // Reset NAL state - next NAL might be partial
            s_nal_length = 0;
            s_skip_until_fu_start = true;
            // Scan for next '$'
            while (!s_rtp_task_exit) {
                n = recv(s_rtsp_sock, &sync_byte, 1, 0);
                if (s_rtp_task_exit) break;
                if (n <= 0) break;
                if (sync_byte == '$') {
                    header[0] = '$';
                    n = tcp_read_exact(s_rtsp_sock, header + 1, 3);
                    if (s_rtp_task_exit) break;
                    if (n == 3) break;
                }
            }
        }
    }
    
    free(buffer);
    ESP_LOGE(TAG, "TCP receiver exited");
    // Always close RTSP socket if not already closed
    if (s_rtsp_sock >= 0) {
        close(s_rtsp_sock);
        s_rtsp_sock = -1;
    }
    // Clear task handle before deleting
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
    
    // Start RTP receiver task
    s_rtp_task_exit = false;
    s_last_rtp_time = esp_log_timestamp();
    xTaskCreatePinnedToCore(rtp_receiver_task, "rtp_rx", 8192, NULL, 8, &s_rtp_task_handle, 1);

    // Main loop - monitor status and handle reconnection
    uint32_t last_keepalive = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Frames decoded: %lu", (unsigned long)s_frame_count);

        // Send RTSP keepalive (OPTIONS) every 20 seconds
        uint32_t now = esp_log_timestamp();
        if (now - last_keepalive > 20000 && s_rtsp_sock >= 0) {
            rtsp_request("OPTIONS", RTSP_URL, NULL, s_rtsp_response, sizeof(s_rtsp_response));
            last_keepalive = now;
        }

        // Check for RTP inactivity timeout or TCP receiver task exit
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
            // Signal previous RTP receiver task to exit if running
            if (s_rtp_task_handle != NULL) {
                s_rtp_task_exit = true;
                // Wait for the task to exit
                int wait_count = 0;
                while (s_rtp_task_handle != NULL && wait_count++ < 100) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                s_rtp_task_exit = false;
            }
            // Always close RTSP socket if not already closed
            if (s_rtsp_sock >= 0) {
                close(s_rtsp_sock);
                s_rtsp_sock = -1;
            }
            // Always close RTP socket if not already closed
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
            // Clear Session ID before handshake
            s_session_id[0] = '\0';
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
            // Restart RTP receiver task
            s_rtp_task_exit = false;
            s_last_rtp_time = esp_log_timestamp();
            xTaskCreatePinnedToCore(rtp_receiver_task, "rtp_rx", 8192, NULL, 5, &s_rtp_task_handle, 1);
            ESP_LOGI(TAG, "RTSP reconnection complete.");
        }
    }
}
