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

static const char *TAG = "RTSP_MANUAL";

// RTSP Configuration
#define RTSP_HOST       "10.0.0.1"
#define RTSP_PORT       8554
#define RTSP_PATH       "/live"
#define RTSP_URL        "rtsp://" RTSP_HOST ":" "8554" RTSP_PATH

// Video configuration
#define VIDEO_WIDTH     720
#define VIDEO_HEIGHT    720
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
    
    s_frame_count++;
    if (s_frame_count % 30 == 1) {
        ESP_LOGI(TAG, "Frame #%lu decoded", (unsigned long)s_frame_count);
    }
    
    // Convert I420 to RGB565
    i420_to_rgb565(frame_data, s_rgb_buffer, VIDEO_WIDTH, VIDEO_HEIGHT);
    
    // Draw to display
    if (VIDEO_WIDTH == DISPLAY_WIDTH && VIDEO_HEIGHT == DISPLAY_HEIGHT) {
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_rgb_buffer);
    } else {
        scale_frame_fit_height(s_rgb_buffer, s_display_buffer, VIDEO_WIDTH, VIDEO_HEIGHT, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_display_buffer);
    }
    display_wait_refresh_done();
}

// Send RTSP request and receive response
static int rtsp_request(const char *method, const char *uri, const char *extra_headers, char *response, int response_size)
{
    char request[512];
    int len = snprintf(request, sizeof(request),
        "%s %s RTSP/1.0\r\n"
        "CSeq: %d\r\n"
        "User-Agent: ESP32-RTSP-Manual\r\n"
        "%s%s"
        "\r\n",
        method, uri, s_cseq++,
        s_session_id[0] ? "Session: " : "",
        s_session_id[0] ? s_session_id : "");
    
    if (extra_headers && extra_headers[0]) {
        // Insert extra headers before final \r\n
        len -= 2;
        len += snprintf(request + len, sizeof(request) - len, "%s\r\n", extra_headers);
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
    
    ESP_LOGI(TAG, "RX:\n%s", response);
    
    // Check for 200 OK
    if (strstr(response, "200 OK") == NULL) {
        ESP_LOGE(TAG, "Request failed");
        return -1;
    }
    
    return received;
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
        ESP_LOGE(TAG, "Failed to create RTSP socket");
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
    char response[2048];
    char uri[256];
    
    // OPTIONS
    if (rtsp_request("OPTIONS", RTSP_URL, NULL, response, sizeof(response)) < 0) {
        return -1;
    }
    
    // DESCRIBE
    if (rtsp_request("DESCRIBE", RTSP_URL, "Accept: application/sdp\r\n", response, sizeof(response)) < 0) {
        return -1;
    }
    
    // Parse track from SDP
    parse_track_from_sdp(response);
    if (s_track_path[0] == '\0') {
        ESP_LOGE(TAG, "Failed to find track in SDP");
        return -1;
    }
    
    // Build track URI
    snprintf(uri, sizeof(uri), "%s/%s", RTSP_URL, s_track_path);
    
    // Setup RTP socket before SETUP request
    if (rtp_setup() < 0) {
        return -1;
    }
    
    // SETUP with UDP transport
    char transport[128];
    snprintf(transport, sizeof(transport), "Transport: RTP/AVP;unicast;client_port=%d-%d\r\n",
             RTP_LOCAL_PORT, RTP_LOCAL_PORT + 1);
    
    if (rtsp_request("SETUP", uri, transport, response, sizeof(response)) < 0) {
        return -1;
    }
    
    // Parse session ID
    parse_session(response);
    
    // PLAY
    char session_hdr[128];
    snprintf(session_hdr, sizeof(session_hdr), "Session: %s\r\nRange: npt=0.000-\r\n", s_session_id);
    if (rtsp_request("PLAY", RTSP_URL, session_hdr, response, sizeof(response)) < 0) {
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
    ESP_LOGD(TAG, "NAL type %d, len %d", nal_type, len);
    
    // Add start code and decode
    static uint8_t nal_with_start[NAL_BUFFER_SIZE + 4];
    nal_with_start[0] = 0;
    nal_with_start[1] = 0;
    nal_with_start[2] = 0;
    nal_with_start[3] = 1;
    memcpy(nal_with_start + 4, nal, len);
    
    h264_decoder_decode(nal_with_start, len + 4);
}

// Process RTP packet and extract H.264 NAL units
// H.264 RTP payload format (RFC 6184)
static void process_rtp_packet(uint8_t *data, int len)
{
    if (len < 12) return;  // RTP header is 12 bytes minimum
    
    // Parse RTP header
    uint8_t version = (data[0] >> 6) & 0x03;
    uint8_t padding = (data[0] >> 5) & 0x01;
    uint8_t extension = (data[0] >> 4) & 0x01;
    uint8_t cc = data[0] & 0x0F;
    uint8_t marker = (data[1] >> 7) & 0x01;
    uint8_t pt = data[1] & 0x7F;
    uint16_t seq = (data[2] << 8) | data[3];
    uint32_t timestamp = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    
    if (version != 2) {
        ESP_LOGW(TAG, "Invalid RTP version: %d", version);
        return;
    }
    
    // Calculate payload offset
    int header_len = 12 + (cc * 4);
    if (extension) {
        // Skip extension header
        if (len < header_len + 4) return;
        int ext_len = (data[header_len + 2] << 8) | data[header_len + 3];
        header_len += 4 + (ext_len * 4);
    }
    
    if (len <= header_len) return;
    
    uint8_t *payload = data + header_len;
    int payload_len = len - header_len;
    
    // Check for sequence number gaps
    if (!s_first_packet) {
        uint16_t expected = (s_last_seq + 1) & 0xFFFF;
        if (seq != expected) {
            ESP_LOGW(TAG, "Seq gap: expected %d, got %d", expected, seq);
            // Reset NAL buffer on gap
            s_nal_length = 0;
        }
    }
    s_first_packet = false;
    s_last_seq = seq;
    
    // Parse NAL unit header (first byte of payload)
    uint8_t nal_type = payload[0] & 0x1F;
    uint8_t nri = (payload[0] >> 5) & 0x03;
    
    ESP_LOGD(TAG, "RTP seq=%d ts=%lu pt=%d marker=%d nal_type=%d len=%d",
             seq, (unsigned long)timestamp, pt, marker, nal_type, payload_len);
    
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
        
        if (start_bit) {
            // First fragment - start new NAL
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
        
        if (end_bit && s_nal_length > 0) {
            // Last fragment - feed complete NAL
            feed_nal_to_decoder(s_nal_buffer, s_nal_length);
            s_nal_length = 0;
        }
    }
    else {
        ESP_LOGW(TAG, "Unsupported NAL type: %d", nal_type);
    }
}

// RTP receiver task
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
    
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);
    
    ESP_LOGI(TAG, "RTP receiver started on port %d", RTP_LOCAL_PORT);
    
    int packet_count = 0;
    while (1) {
        int len = recvfrom(s_rtp_sock, buffer, RTP_BUFFER_SIZE, 0,
                          (struct sockaddr *)&src_addr, &addr_len);
        
        if (len > 0) {
            packet_count++;
            if (packet_count % 100 == 1) {
                ESP_LOGI(TAG, "RTP packets received: %d", packet_count);
            }
            process_rtp_packet(buffer, len);
        } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "RTP receive error: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    free(buffer);
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
    size_t rgb_size = rgb565_buffer_size(VIDEO_WIDTH, VIDEO_HEIGHT);
    s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_rgb_buffer) s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_rgb_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer");
        return;
    }
    
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
    
    ESP_LOGI(TAG, "Buffers allocated");
    
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
    
    // Start RTP receiver task
    xTaskCreatePinnedToCore(rtp_receiver_task, "rtp_rx", 8192, NULL, 5, NULL, 1);
    
    // Main loop - monitor status
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Frames decoded: %lu", (unsigned long)s_frame_count);
    }
}
