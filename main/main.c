#include <stdlib.h>
#include <string.h>
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
#include "ps_diag.h"

static const char *TAG = "App";

#define DISPLAY_WIDTH  720
#define DISPLAY_HEIGHT 720

// Display and frame buffers
static uint16_t *s_display_buffer = NULL;  // fixed 720×720 for lcd_panel_draw_bitmap
static uint16_t *s_rgb_buffer     = NULL;  // reallocated when stream resolution changes
static uint32_t  s_frame_count    = 0;
static uint16_t  s_input_width    = 0;
static uint16_t  s_input_height   = 0;

// ─── Diagnostics ──────────────────────────────────────────────────────────────

ps_perf_t g_ps_perf = {0};

void ps_perf_frame_done(void)
{
#if PS_PERF
    g_ps_perf.frames++;
    int64_t now = esp_timer_get_time();
    if (g_ps_perf.window_start_us == 0) { g_ps_perf.window_start_us = now; return; }
    int64_t elapsed = now - g_ps_perf.window_start_us;
    if (elapsed < 1000000) return;

    uint32_t n = g_ps_perf.frames ? g_ps_perf.frames : 1;
    float secs = elapsed / 1e6f;
    // Per-frame averages in ms. "net" is time spent blocked waiting for data —
    // high net with low fps means the sender/network is the bottleneck.
    ESP_LOGI("PERF",
             "%dx%d %.1f fps %lu kbps | ms/frame: net %.1f dec %.1f (max %.1f) "
             "yuv %.1f scale %.1f disp %.1f | rtp gaps %lu lost %lu",
             s_input_width, s_input_height,
             g_ps_perf.frames / secs,
             (unsigned long)(g_ps_perf.bytes * 8 / 1000 / secs),
             g_ps_perf.stage_us[PS_STAGE_NET]     / 1000.0f / n,
             g_ps_perf.stage_us[PS_STAGE_DECODE]  / 1000.0f / n,
             g_ps_perf.decode_max_us / 1000.0f,
             g_ps_perf.stage_us[PS_STAGE_CONVERT] / 1000.0f / n,
             g_ps_perf.stage_us[PS_STAGE_SCALE]   / 1000.0f / n,
             g_ps_perf.stage_us[PS_STAGE_DISPLAY] / 1000.0f / n,
             (unsigned long)g_ps_perf.rtp_gaps, (unsigned long)g_ps_perf.rtp_lost);

    memset(&g_ps_perf, 0, sizeof(g_ps_perf));
    g_ps_perf.window_start_us = now;
#endif
}

// ─── Scaling ──────────────────────────────────────────────────────────────────

// Fit-height nearest-neighbour scale with horizontal crop/pad to fill
// dst_w×dst_h. Column mapping is precomputed once per input size (no divide
// per pixel), and destination rows that map to the same source row are copied
// instead of recomputed (~2.8× fewer rows at 256→720).
static void scale_frame_fit_height(const uint16_t *src, uint16_t *dst,
                                   uint16_t src_w, uint16_t src_h,
                                   uint16_t dst_w, uint16_t dst_h)
{
    static uint16_t *x_lut = NULL;
    static uint16_t  lut_src_w = 0, lut_src_h = 0, lut_dst_w = 0, lut_dst_h = 0;

    if (!x_lut || lut_src_w != src_w || lut_src_h != src_h ||
            lut_dst_w != dst_w || lut_dst_h != dst_h) {
        free(x_lut);
        x_lut = malloc(dst_w * sizeof(uint16_t));
        if (!x_lut) return;
        uint32_t scaled_w = (uint32_t)src_w * dst_h / src_h;
        int32_t  x_offset = ((int32_t)scaled_w - dst_w) / 2;
        for (uint16_t x = 0; x < dst_w; x++) {
            int32_t sx = x + x_offset;
            if (sx < 0) sx = 0;
            if (sx >= (int32_t)scaled_w) sx = scaled_w - 1;
            x_lut[x] = (uint16_t)((uint32_t)sx * src_w / scaled_w);
        }
        lut_src_w = src_w; lut_src_h = src_h;
        lut_dst_w = dst_w; lut_dst_h = dst_h;
    }

    int32_t prev_src_y = -1;
    for (uint16_t y = 0; y < dst_h; y++) {
        int32_t   src_y = ((uint32_t)y * src_h) / dst_h;
        uint16_t *drow  = dst + (size_t)y * dst_w;
        if (src_y == prev_src_y) {
            memcpy(drow, drow - dst_w, dst_w * sizeof(uint16_t));
            continue;
        }
        const uint16_t *srow = src + (size_t)src_y * src_w;
        for (uint16_t x = 0; x < dst_w; x++) drow[x] = srow[x_lut[x]];
        prev_src_y = src_y;
    }
}

// ─── Frame callback (called from H.264 decoder on each decoded frame) ─────────

static void on_frame_decoded(uint8_t *frame_data, uint16_t width, uint16_t height,
                             uint32_t size, void *user_ctx)
{
    if (!frame_data || size == 0) return;

    // Fallback when the decoder couldn't report the size: infer it from the
    // I420 buffer size (W×H = size×2/3), assuming a square frame.
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
    if (size < (uint32_t)width * height * 3 / 2) {
        ESP_LOGW(TAG, "Short frame: %lu bytes for %dx%d", (unsigned long)size, width, height);
        return;
    }

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
    PS_LOGV(TAG, "Frame #%lu: %dx%d (%lu bytes)",
            (unsigned long)s_frame_count, width, height, (unsigned long)size);

#if PS_DEBUG
    uint32_t y_sum = 0;
    for (int i = 0; i < (int)(width * height); i += 31) y_sum += frame_data[i];
    ESP_LOGI(TAG, "Y checksum=0x%08lx Y[0]=%u Y[1]=%u",
             (unsigned long)y_sum, frame_data[0], frame_data[1]);
#endif

    // Convert straight from the decoder's output buffer — it stays valid for
    // the duration of this callback, so no intermediate copy is needed.
    int64_t t = ps_now_us();
    i420_to_rgb565(frame_data, s_rgb_buffer, width, height);
    ps_perf_add(PS_STAGE_CONVERT, t);

#if PS_DEBUG
    uint32_t rgb_sum = 0;
    for (int i = 0; i < (int)(width * height); i += 31) rgb_sum += s_rgb_buffer[i];
    ESP_LOGI(TAG, "RGB565 checksum=0x%08lx RGB[0]=0x%04x",
             (unsigned long)rgb_sum, s_rgb_buffer[0]);
#endif

    const uint16_t *out = s_rgb_buffer;
    if (width != DISPLAY_WIDTH || height != DISPLAY_HEIGHT) {
        t = ps_now_us();
        scale_frame_fit_height(s_rgb_buffer, s_display_buffer,
                               width, height, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        ps_perf_add(PS_STAGE_SCALE, t);
        out = s_display_buffer;
    }

    t = ps_now_us();
    esp_lcd_panel_draw_bitmap(get_panel_handle(),
                              0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, out);
    display_wait_refresh_done();
    ps_perf_add(PS_STAGE_DISPLAY, t);

    ps_perf_frame_done();
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
