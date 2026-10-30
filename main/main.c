#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "driver/ppa.h"
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

// Hardware render path: PPA scales RGB565 straight into the off-screen frame
// buffer, which is then swapped in. Falls back to CPU scale + copy if any
// part is unavailable.
static ppa_client_handle_t s_ppa_srm      = NULL;
static ppa_client_handle_t s_ppa_fill     = NULL;
static bool                s_hw_render    = false;
static int                 s_clear_pending = 0;  // back buffers still to clear after a size change

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

// ─── Hardware render (PPA + double buffer) ────────────────────────────────────

static void render_init(void)
{
    if (!display_has_double_buffer()) {
        ESP_LOGW(TAG, "Render: CPU scale + copy (no double buffer)");
        return;
    }
    ppa_client_config_t srm_cfg = { .oper_type = PPA_OPERATION_SRM };
    if (ppa_register_client(&srm_cfg, &s_ppa_srm) != ESP_OK) {
        ESP_LOGW(TAG, "Render: CPU scale + copy (PPA SRM unavailable)");
        return;
    }
    ppa_client_config_t fill_cfg = { .oper_type = PPA_OPERATION_FILL };
    if (ppa_register_client(&fill_cfg, &s_ppa_fill) != ESP_OK) s_ppa_fill = NULL;
    s_hw_render = true;
    ESP_LOGI(TAG, "Render: PPA scale into double-buffered frame buffer");
}

// Fit-height scale of an RGB565 frame into the back buffer, centred, with
// horizontal crop (wide input) or black bars (narrow input), then swap.
// Returns false if the PPA rejected the job; the caller then uses the CPU path.
static bool render_hw(const uint16_t *rgb, uint16_t w, uint16_t h)
{
    int64_t t = ps_now_us();
    uint16_t *fb = display_acquire_back_buffer();
    ps_perf_add(PS_STAGE_DISPLAY, t);
    if (!fb) return false;

    t = ps_now_us();
    // PPA scales in 1/16 steps; round down so the result never overflows the
    // screen (256→720 is ×2.8125 = 45/16 and 360→720 is ×2: both exact).
    float scale = floorf((float)DISPLAY_HEIGHT * 16.0f / h) / 16.0f;
    if (scale < 1.0f / 16.0f) scale = 1.0f / 16.0f;

    uint32_t blk_w = w, in_off_x = 0;
    if ((uint32_t)(scale * w) > DISPLAY_WIDTH) {       // wider than the screen: crop centre
        blk_w = (uint32_t)(DISPLAY_WIDTH / scale);
        while (blk_w > 0 && (uint32_t)(scale * blk_w) > DISPLAY_WIDTH) blk_w--;
        in_off_x = (w - blk_w) / 2;
    }
    uint32_t out_w = (uint32_t)(scale * blk_w);
    uint32_t out_h = (uint32_t)(scale * h);
    uint32_t out_x = (DISPLAY_WIDTH  - out_w) / 2;
    uint32_t out_y = (DISPLAY_HEIGHT - out_h) / 2;

    // After a size change, clear each buffer once so old pixels don't linger
    // in any bars around the picture.
    if (s_clear_pending > 0) {
        if (s_ppa_fill && (out_w < DISPLAY_WIDTH || out_h < DISPLAY_HEIGHT)) {
            ppa_fill_oper_config_t fill = {
                .out = {
                    .buffer = fb, .buffer_size = display_frame_buffer_size(),
                    .pic_w = DISPLAY_WIDTH, .pic_h = DISPLAY_HEIGHT,
                    .fill_cm = PPA_FILL_COLOR_MODE_RGB565,
                },
                .fill_block_w = DISPLAY_WIDTH, .fill_block_h = DISPLAY_HEIGHT,
                .fill_color_val = 0,
                .mode = PPA_TRANS_MODE_BLOCKING,
            };
            ppa_do_fill(s_ppa_fill, &fill);
        }
        s_clear_pending--;
    }

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = rgb, .pic_w = w, .pic_h = h,
            .block_w = blk_w, .block_h = h,
            .block_offset_x = in_off_x, .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = fb, .buffer_size = display_frame_buffer_size(),
            .pic_w = DISPLAY_WIDTH, .pic_h = DISPLAY_HEIGHT,
            .block_offset_x = out_x, .block_offset_y = out_y,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = scale,
        .scale_y = scale,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    esp_err_t err = ppa_do_scale_rotate_mirror(s_ppa_srm, &op);
    ps_perf_add(PS_STAGE_SCALE, t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPA scale failed (%s) for %dx%d — switching to CPU path",
                 esp_err_to_name(err), w, h);
        s_hw_render = false;
        return false;
    }

    t = ps_now_us();
    display_present(fb);
    ps_perf_add(PS_STAGE_DISPLAY, t);
    return true;
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
        s_clear_pending = 2;   // both frame buffers
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

    if (s_hw_render && render_hw(s_rgb_buffer, width, height)) {
        ps_perf_frame_done();
        return;
    }

    // CPU fallback: software scale, then copy into the single frame buffer.
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
    render_init();

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
