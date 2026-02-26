#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "ethernet.h"
#include "display.h"
#include "h264.h"
#include "yuv_rgb.h"
#include "rtsp.h"

static const char *TAG = "App";

#define DISPLAY_WIDTH  720
#define DISPLAY_HEIGHT 720

// Display and frame buffers
static uint16_t *s_display_buffer = NULL;  // fixed 720×720 for lcd_panel_draw_bitmap
static uint16_t *s_rgb_buffer     = NULL;  // reallocated when stream resolution changes
static uint32_t  s_frame_count    = 0;
static uint16_t  s_input_width    = 0;
static uint16_t  s_input_height   = 0;

// ─── Scaling ──────────────────────────────────────────────────────────────────

// Fit-height scale with horizontal crop/pad to fill DISPLAY_WIDTH×DISPLAY_HEIGHT.
static void scale_frame_fit_height(const uint16_t *src, uint16_t *dst,
                                   uint16_t src_w, uint16_t src_h,
                                   uint16_t dst_w, uint16_t dst_h)
{
    uint32_t scaled_w = (uint32_t)src_w * dst_h / src_h;
    int32_t  x_offset = ((int32_t)scaled_w - dst_w) / 2;
    for (uint16_t y = 0; y < dst_h; y++) {
        uint16_t src_y = (y * src_h) / dst_h;
        for (uint16_t x = 0; x < dst_w; x++) {
            int32_t sx = x + x_offset;
            if (sx < 0) sx = 0;
            if (sx >= (int32_t)scaled_w) sx = scaled_w - 1;
            dst[y * dst_w + x] = src[src_y * src_w + (sx * src_w / scaled_w)];
        }
    }
}

// ─── Frame callback (called from H.264 decoder on each decoded frame) ─────────

static void on_frame_decoded(uint8_t *frame_data, uint16_t width, uint16_t height,
                             uint32_t size, void *user_ctx)
{
    if (!frame_data || size == 0) return;

    // Infer dimensions from I420 buffer size when decoder reports 0,0.
    // I420: size = W×H×3/2  →  W×H = size×2/3. For square frames: side = √(W×H).
    if (width == 0 || height == 0) {
        uint32_t pixels = (size * 2) / 3;
        uint16_t side   = (uint16_t)sqrtf((float)pixels);
        if (side > 0 && (uint32_t)side * side == pixels) {
            width = height = side;
        } else {
            width  = s_input_width  ? s_input_width  : 256;
            height = s_input_height ? s_input_height : 256;
        }
    }

    // YUV working buffer (reallocated on resolution change)
    static uint8_t *s_yuv_buffer = NULL;
    static size_t   s_yuv_size   = 0;
    size_t required = (size_t)width * height * 3 / 2;
    if (width != s_input_width || height != s_input_height ||
            !s_yuv_buffer || s_yuv_size != required) {
        free(s_yuv_buffer);
        s_yuv_buffer = malloc(required);
        s_yuv_size   = required;
        if (!s_yuv_buffer) {
            ESP_LOGE(TAG, "Failed to allocate YUV buffer for %dx%d", width, height);
            return;
        }
    }
    memcpy(s_yuv_buffer, frame_data, size);

    // RGB565 buffer (reallocated on resolution change)
    if (width != s_input_width || height != s_input_height || !s_rgb_buffer) {
        ESP_LOGI(TAG, "Detected video resolution: %dx%d", width, height);
        free(s_rgb_buffer); s_rgb_buffer = NULL;
        size_t rgb_size = rgb565_buffer_size(width, height);
        s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size,
                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rgb_buffer)
            s_rgb_buffer = heap_caps_aligned_calloc(64, 1, rgb_size,
                               MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_rgb_buffer) {
            ESP_LOGE(TAG, "Failed to allocate RGB buffer for %dx%d", width, height);
            return;
        }
        s_input_width  = width;
        s_input_height = height;
    }

    s_frame_count++;
    ESP_LOGI(TAG, "Frame #%lu: %dx%d (%lu bytes)",
             (unsigned long)s_frame_count, width, height, (unsigned long)size);

    // Debug checksums
    uint32_t y_sum = 0;
    for (int i = 0; i < (int)(width * height); i += 31) y_sum += s_yuv_buffer[i];
    ESP_LOGI(TAG, "Y checksum=0x%08lx Y[0]=%u Y[1]=%u",
             (unsigned long)y_sum, s_yuv_buffer[0], s_yuv_buffer[1]);

    i420_to_rgb565(s_yuv_buffer, s_rgb_buffer, width, height);

    uint32_t rgb_sum = 0;
    for (int i = 0; i < (int)(width * height); i += 31) rgb_sum += s_rgb_buffer[i];
    ESP_LOGI(TAG, "RGB565 checksum=0x%08lx RGB[0]=0x%04x",
             (unsigned long)rgb_sum, s_rgb_buffer[0]);

    if (width == DISPLAY_WIDTH && height == DISPLAY_HEIGHT) {
        esp_lcd_panel_draw_bitmap(get_panel_handle(),
                                  0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_rgb_buffer);
    } else {
        scale_frame_fit_height(s_rgb_buffer, s_display_buffer,
                               width, height, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        esp_lcd_panel_draw_bitmap(get_panel_handle(),
                                  0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, s_display_buffer);
    }
    display_wait_refresh_done();
}

// ─── Entry point ──────────────────────────────────────────────────────────────

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_ethernet();
    ESP_LOGI(TAG, "Waiting for IP...");
    xEventGroupWaitBits(s_eth_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "Initializing display");
    init_display();

    size_t display_size = rgb565_buffer_size(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size,
                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_display_buffer)
        s_display_buffer = heap_caps_aligned_calloc(64, 1, display_size,
                               MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_display_buffer) {
        ESP_LOGE(TAG, "Failed to allocate display buffer");
        return;
    }

    h264_decoder_config_t h264_cfg = {
        .on_frame_decoded = on_frame_decoded,
        .user_ctx         = NULL,
    };
    if (rtsp_client_start(&h264_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RTSP client");
        return;
    }

    // All ongoing work is handled by the RTSP supervisor and receiver tasks.
    vTaskDelete(NULL);
}
