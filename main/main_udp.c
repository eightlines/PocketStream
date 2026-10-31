#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "ethernet.h"
#include "display.h"
#include "h264.h"
#include "yuv_rgb.h"

static const char *TAG = "P4_VIDEO";

#define VIDEO_WIDTH     720
#define VIDEO_HEIGHT    720
#define DISPLAY_WIDTH   720
#define DISPLAY_HEIGHT  720
#define UDP_PORT        3333
#define UDP_BUFFER_SIZE 65536  // Larger buffer for 720x720 frames

// Display buffers
static uint16_t *s_rgb_buffer = NULL;      // Decoded frame (small)
static uint16_t *s_display_buffer = NULL;  // Scaled frame (full screen)
static uint32_t s_frame_count = 0;

// Scale frame with aspect ratio preservation and center crop
// Fits source to display height, then center-crops horizontally
static void scale_frame_fit_height(const uint16_t *src, uint16_t *dst,
                                   uint16_t src_w, uint16_t src_h,
                                   uint16_t dst_w, uint16_t dst_h)
{
    // Calculate scale factor to fit height
    // scaled_width = src_w * dst_h / src_h
    uint32_t scaled_w = (uint32_t)src_w * dst_h / src_h;
    
    // Calculate horizontal offset to center (in scaled coordinates)
    int32_t x_offset = ((int32_t)scaled_w - dst_w) / 2;
    
    for (uint16_t y = 0; y < dst_h; y++) {
        uint16_t src_y = (y * src_h) / dst_h;
        for (uint16_t x = 0; x < dst_w; x++) {
            // Map display x to scaled x, then to source x
            int32_t scaled_x = x + x_offset;
            if (scaled_x < 0) scaled_x = 0;
            if (scaled_x >= (int32_t)scaled_w) scaled_x = scaled_w - 1;
            
            uint16_t src_x = (scaled_x * src_w) / scaled_w;
            dst[y * dst_w + x] = src[src_y * src_w + src_x];
        }
    }
}

// Frame decoded callback - convert YUV to RGB, scale, and display
static void on_frame_decoded(uint8_t *frame_data, uint16_t width, uint16_t height, uint32_t size, void *user_ctx)
{
    // Use known dimensions since decoder may not provide them
    uint16_t w = VIDEO_WIDTH;
    uint16_t h = VIDEO_HEIGHT;
    
    if (frame_data == NULL || size == 0) {
        return;
    }
    
    s_frame_count++;
    if (s_frame_count % 30 == 1) {  // Log every 30 frames
        ESP_LOGI(TAG, "Frame #%lu decoded", (unsigned long)s_frame_count);
    }
    
    // Convert I420 to RGB565
    i420_to_rgb565(frame_data, s_rgb_buffer, w, h);
    
    // If sizes match, draw directly; otherwise scale
    if (w == DISPLAY_WIDTH && h == DISPLAY_HEIGHT) {
        // Direct copy - no scaling needed
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_rgb_buffer);
    } else {
        // Scale to full screen (fit height, center crop width)
        scale_frame_fit_height(s_rgb_buffer, s_display_buffer, w, h, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_display_buffer);
    }
    display_wait_refresh_done();
}

// UDP receiver task
static void udp_receiver_task(void *pvParameters)
{
    uint8_t *rx_buffer = heap_caps_malloc(UDP_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (rx_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UDP receive buffer");
        vTaskDelete(NULL);
        return;
    }
    
    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        free(rx_buffer);
        vTaskDelete(NULL);
        return;
    }
    
    // Bind to port
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: errno %d", errno);
        close(sock);
        free(rx_buffer);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "UDP socket listening on port %d", UDP_PORT);
    
    // Receive loop
    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        
        int len = recvfrom(sock, rx_buffer, UDP_BUFFER_SIZE, 0,
                          (struct sockaddr *)&source_addr, &addr_len);
        
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            continue;
        }
        
        if (len > 0) {
            ESP_LOGD(TAG, "Received %d bytes", len);
            // Feed data to H.264 decoder
            h264_decoder_decode(rx_buffer, len);
        }
    }
    
    // Cleanup (unreachable)
    close(sock);
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
    ESP_LOGI(TAG, "Initialize LCD device");
    init_display();
    
    // Allocate RGB565 buffer for decoded frames (720x720 = ~1MB - needs PSRAM)
    size_t rgb_size = rgb565_buffer_size(VIDEO_WIDTH, VIDEO_HEIGHT);
    ESP_LOGI(TAG, "Allocating decoded frame buffer: %u bytes", (unsigned)rgb_size);
    s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_rgb_buffer == NULL) {
        // Fall back to internal if PSRAM not available
        s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (s_rgb_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer");
        return;
    }
    
    // Allocate display buffer for scaled frames (large: 720x720 = ~1MB)
    size_t display_size = rgb565_buffer_size(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    ESP_LOGI(TAG, "Allocating display buffer: %u bytes", (unsigned)display_size);
    s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_display_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate display buffer - trying internal memory");
        s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (s_display_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate display buffer");
        return;
    }
    ESP_LOGI(TAG, "Buffers allocated successfully");
    
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
    
    // Start UDP receiver task
    ESP_LOGI(TAG, "Starting UDP receiver on port %d...", UDP_PORT);
    xTaskCreate(udp_receiver_task, "udp_rx", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Ready to receive H.264 stream");
    
    // Main loop - just idle
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}