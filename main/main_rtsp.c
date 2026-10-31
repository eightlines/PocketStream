/**
 * @file main_rtsp.c
 * @brief RTSP client video receiver for ESP32-P4
 *
 * Receives H.264 video stream directly via RTSP protocol.
 * No external ffmpeg bridge needed!
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "ethernet.h"
#include "display.h"
#include "h264.h"
#include "yuv_rgb.h"
#include "esp_rtsp.h"
#include "media_lib_adapter.h"  // For media_lib_add_default_adapter()

static const char *TAG = "RTSP_CLIENT";

// Configure your RTSP stream URL here
#define RTSP_URL        "rtsp://10.0.0.1:8554/live"  // Change this!

#define VIDEO_WIDTH     720
#define VIDEO_HEIGHT    720
#define DISPLAY_WIDTH   720
#define DISPLAY_HEIGHT  720

// Display buffers
static uint16_t *s_rgb_buffer = NULL;
static uint16_t *s_display_buffer = NULL;
static uint32_t s_frame_count = 0;
static esp_rtsp_handle_t s_rtsp_handle = NULL;

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
    uint16_t w = VIDEO_WIDTH;
    uint16_t h = VIDEO_HEIGHT;
    
    if (frame_data == NULL || size == 0) {
        return;
    }
    
    s_frame_count++;
    if (s_frame_count % 30 == 1) {
        ESP_LOGI(TAG, "Frame #%lu decoded", (unsigned long)s_frame_count);
    }
    
    // Convert I420 to RGB565
    i420_to_rgb565(frame_data, s_rgb_buffer, w, h);
    
    // Draw to display
    if (w == DISPLAY_WIDTH && h == DISPLAY_HEIGHT) {
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_rgb_buffer);
    } else {
        scale_frame_fit_height(s_rgb_buffer, s_display_buffer, w, h, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_display_buffer);
    }
    display_wait_refresh_done();
}

// RTSP video data callback - receives depacketized H.264 NAL units
static int rtsp_receive_video(unsigned char *data, int len, void *ctx)
{
    if (data == NULL || len <= 0) {
        return 0;
    }
    
    ESP_LOGD(TAG, "RTSP video: %d bytes", len);
    
    // Feed H.264 NAL units to decoder
    h264_decoder_decode(data, len);
    
    return 0;
}

// RTSP state callback
static int rtsp_state_handler(esp_rtsp_state_t state, void *ctx)
{
    const char *state_names[] = {
        "NONE", "OPTIONS", "ANNOUNCE", "DESCRIBING", "DESCRIBE",
        "SETUP", "PLAY", "RECORD", "TEARDOWN"
    };
    
    if (state < sizeof(state_names)/sizeof(state_names[0])) {
        ESP_LOGI(TAG, "RTSP state: %s", state_names[state]);
    } else {
        ESP_LOGI(TAG, "RTSP state: %d", state);
    }
    
    return 0;
}

// RTSP stream codec info callback
static int rtsp_stream_codec(esp_rtsp_aud_info_t* aud_info, esp_rtsp_video_info_t* vid_info, void *ctx)
{
    if (vid_info != NULL) {
        ESP_LOGI(TAG, "Video stream: %dx%d @ %d fps, codec=%d",
                 vid_info->width, vid_info->height, vid_info->fps, vid_info->vcodec);
    }
    if (aud_info != NULL) {
        ESP_LOGI(TAG, "Audio stream: %d Hz, %d ch, codec=%d",
                 aud_info->sample_rate, aud_info->channel, aud_info->aud_codec);
    }
    return 0;
}

void app_main(void)
{
    // Initialize media library adapters (required for esp_media_protocols)
    esp_err_t ret = media_lib_add_default_adapter();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize media library adapters");
    }
    
    ret = nvs_flash_init();
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
    
    // Allocate RGB565 buffer (720x720 = ~1MB - needs PSRAM)
    size_t rgb_size = rgb565_buffer_size(VIDEO_WIDTH, VIDEO_HEIGHT);
    ESP_LOGI(TAG, "Allocating frame buffer: %u bytes", (unsigned)rgb_size);
    s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_rgb_buffer == NULL) {
        s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (s_rgb_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer");
        return;
    }
    
    // Allocate display buffer
    size_t display_size = rgb565_buffer_size(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_display_buffer == NULL) {
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
    ESP_LOGI(TAG, "H.264 decoder initialized");
    
    // Configure RTSP client callbacks
    esp_rtsp_data_cb_t data_cb = {
        .receive_video = rtsp_receive_video,
        .receive_audio = NULL,
        .send_video = NULL,
        .send_audio = NULL,
        .stream_codec = rtsp_stream_codec,
    };
    
    // Configure video info (expected format)
    esp_rtsp_video_info_t video_info = {
        .vcodec = RTSP_VCODEC_H264,
        .width = VIDEO_WIDTH,
        .height = VIDEO_HEIGHT,
        .fps = 30,
        .len = 128 * 1024,  // Max frame size
    };
    
    // Configure RTSP client
    esp_rtsp_config_t rtsp_cfg = {
        .ctx = NULL,
        .video_enable = true,
        .audio_enable = false,
        .uri = RTSP_URL,
        .local_port = 0,  // Auto-select
        .local_addr = NULL,
        .stack_size = 8192,
        .task_prio = 5,
        .mode = RTSP_CLIENT_PLAY,
        .video_info = &video_info,
        .data_cb = &data_cb,
        .state = rtsp_state_handler,
        .trans = RTSP_TRANSPORT_TCP,  // TCP is more reliable
    };
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Starting RTSP client...");
    ESP_LOGI(TAG, "URL: %s", RTSP_URL);
    ESP_LOGI(TAG, "===========================================");
    
    // Start RTSP client
    s_rtsp_handle = esp_rtsp_client_start(&rtsp_cfg);
    if (s_rtsp_handle == NULL) {
        ESP_LOGE(TAG, "Failed to start RTSP client!");
        ESP_LOGE(TAG, "Check RTSP URL and ensure stream is available");
        return;
    }
    
    ESP_LOGI(TAG, "RTSP client started successfully");
    
    // Main loop - monitor status
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Frames decoded: %lu", (unsigned long)s_frame_count);
    }
}
