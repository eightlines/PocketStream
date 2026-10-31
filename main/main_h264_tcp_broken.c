/**
 * @file main.c
 * @brief H.264 video streaming receiver with TCP transport
 * 
 * Combines:
 * - TCP transport for reliable delivery (no packet loss)
 * - H.264 decoding (TinyH264 - proven working)
 * - Length-prefixed framing for frame boundaries
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "ethernet.h"
#include "display.h"
#include "h264.h"
#include "yuv_rgb.h"

static const char *TAG = "H264_TCP";

// Stream parameters
#define VIDEO_WIDTH     320
#define VIDEO_HEIGHT    320
#define DISPLAY_WIDTH   720
#define DISPLAY_HEIGHT  720
#define TCP_PORT        3333
#define MAX_FRAME_SIZE  (128 * 1024)  // 128KB max per H.264 frame

// Display buffers
static uint16_t *s_rgb_buffer = NULL;      // Decoded frame (small)
static uint16_t *s_display_buffer = NULL;  // Scaled frame (full screen)
static uint32_t s_frame_count = 0;
static int64_t s_last_stats_time = 0;

// Scale frame with aspect ratio preservation
static void scale_frame(const uint16_t *src, uint16_t *dst,
                        uint16_t src_w, uint16_t src_h,
                        uint16_t dst_w, uint16_t dst_h)
{
    // Calculate scale factor to fit height
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

// Frame decoded callback
static void on_frame_decoded(uint8_t *frame_data, uint16_t width, uint16_t height, uint32_t size, void *user_ctx)
{
    uint16_t w = VIDEO_WIDTH;
    uint16_t h = VIDEO_HEIGHT;
    
    if (frame_data == NULL || size == 0) {
        return;
    }
    
    s_frame_count++;
    
    // Convert I420 to RGB565
    i420_to_rgb565(frame_data, s_rgb_buffer, w, h);
    
    // Scale to full screen
    scale_frame(s_rgb_buffer, s_display_buffer, w, h, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    
    // Draw to display
    esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_display_buffer);
    display_wait_refresh_done();
}

// Receive exactly n bytes from socket
static int recv_exact(int sock, uint8_t *buf, size_t len)
{
    size_t received = 0;
    while (received < len) {
        int r = recv(sock, buf + received, len - received, 0);
        if (r <= 0) return r;
        received += r;
    }
    return received;
}

// TCP receiver task
static void tcp_receiver_task(void *pvParameters)
{
    uint8_t *rx_buffer = heap_caps_malloc(MAX_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rx_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate receive buffer");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "TCP receiver ready, waiting for connections on port %d", TCP_PORT);
    
    while (1) {
        // Create TCP server socket
        int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (server_sock < 0) {
            ESP_LOGE(TAG, "Failed to create socket");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        int opt = 1;
        setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in bind_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(TCP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };
        
        if (bind(server_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            ESP_LOGE(TAG, "Bind failed");
            close(server_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        if (listen(server_sock, 1) < 0) {
            ESP_LOGE(TAG, "Listen failed");
            close(server_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        ESP_LOGI(TAG, "Listening on port %d...", TCP_PORT);
        
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);
        
        if (client_sock < 0) {
            close(server_sock);
            continue;
        }
        
        ESP_LOGI(TAG, "Client connected from %s", inet_ntoa(client_addr.sin_addr));
        
        int nodelay = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        
        s_last_stats_time = esp_timer_get_time();
        s_frame_count = 0;
        
        // Receive raw H.264 stream (no framing, just feed chunks to decoder)
        while (1) {
            int len = recv(client_sock, rx_buffer, 4096, 0);
            if (len <= 0) {
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }
            
            // Feed directly to H.264 decoder
            h264_decoder_decode(rx_buffer, len);
            
            // Print stats every second
            int64_t now = esp_timer_get_time();
            if (now - s_last_stats_time >= 1000000) {
                ESP_LOGI(TAG, "FPS: %lu", (unsigned long)s_frame_count);
                s_frame_count = 0;
                s_last_stats_time = now;
            }
        }
        
        close(client_sock);
        close(server_sock);
    }
    
    free(rx_buffer);
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
    ESP_LOGI(TAG, "Initializing display...");
    init_display();
    
    // Allocate RGB565 buffer for decoded frames
    size_t rgb_size = rgb565_buffer_size(VIDEO_WIDTH, VIDEO_HEIGHT);
    ESP_LOGI(TAG, "Allocating decoded frame buffer: %u bytes", (unsigned)rgb_size);
    s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (s_rgb_buffer == NULL) {
        s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_SPIRAM);
    }
    if (s_rgb_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer");
        return;
    }
    
    // Allocate display buffer for scaled frames
    size_t display_size = rgb565_buffer_size(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    ESP_LOGI(TAG, "Allocating display buffer: %u bytes", (unsigned)display_size);
    s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_display_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate display buffer");
        return;
    }
    
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
    
    // Start TCP receiver task
    xTaskCreate(tcp_receiver_task, "tcp_rx", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "H.264 TCP receiver ready!");
    ESP_LOGI(TAG, "Stream command: .\\stream_h264_tcp.ps1");
    ESP_LOGI(TAG, "===========================================");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
