#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
static uint16_t  s_input_width    = 0;    // render task: size of s_rgb_buffer
static uint16_t  s_input_height   = 0;
static uint16_t  s_last_dec_w     = 0;    // receiver task: last decoded size
static uint16_t  s_last_dec_h     = 0;

// ─── Two-core pipeline ────────────────────────────────────────────────────────
// Receiver task (core 1): network → depacketize → decode → copy the frame into
//                         a free slot → hand it to the render task.
// Render task (core 0):   slot → YUV→RGB → (slot freed) → PPA scale → swap.
// A slot index is owned by exactly one side at a time; the queues pass it.
#define PS_FRAME_SLOTS    2
#define PS_RENDER_CORE    0
#define PS_RENDER_PRIO    6      // above speech (5), below the receiver (8)
#define PS_RENDER_STACK   6144

// 1: the receiver repacks each frame into the P4's packed YUV420 layout while
// copying it into the slot, and the PPA converts YUV→RGB *and* scales in one
// hardware pass (no CPU colour conversion on the render core).
// 0: compact I420 slots, CPU YUV→RGB, PPA scales RGB565.
#ifndef PS_PPA_YUV
#define PS_PPA_YUV        1
#endif

// Planes of an I420 picture (possibly a window into a larger decoded frame).
typedef struct {
    const uint8_t *y, *u, *v;
    int y_stride, uv_stride;
} yuv_view_t;

typedef struct {
    uint8_t  *yuv;     // visible area only: compact I420, or packed YUV420
    size_t    cap;     // allocated bytes
    uint16_t  w, h;
    bool      packed;  // true: packed YUV420 for the PPA
} ps_frame_slot_t;

static ps_frame_slot_t s_slots[PS_FRAME_SLOTS];
static QueueHandle_t   s_free_q   = NULL;   // slot indices ready to be filled
static QueueHandle_t   s_ready_q  = NULL;   // slot indices ready to be shown
static bool            s_pipelined = false; // false: render inline on the receiver

// Hardware render path: PPA scales RGB565 straight into the off-screen frame
// buffer, which is then swapped in. Falls back to CPU scale + copy if any
// part is unavailable.
static ppa_client_handle_t s_ppa_srm      = NULL;
static ppa_client_handle_t s_ppa_fill     = NULL;
static bool                s_hw_render    = false;
static volatile bool       s_ppa_yuv_ok   = PS_PPA_YUV;  // PPA accepts packed YUV420 input
static int                 s_clear_pending = 0;  // back buffers still to clear after a size change

// ─── Diagnostics ──────────────────────────────────────────────────────────────

ps_perf_t g_ps_perf = {0};

void ps_perf_frame_done(void)
{
#if PS_PERF
    static ps_perf_t prev;
    static int64_t   window_start_us = 0;

    g_ps_perf.shown++;
    int64_t now = esp_timer_get_time();
    if (window_start_us == 0) { window_start_us = now; prev = g_ps_perf; return; }
    int64_t elapsed = now - window_start_us;
    if (elapsed < 1000000) return;

    // Snapshot (fields owned by the receiver task are only read here).
    __asm__ volatile("" ::: "memory");
    ps_perf_t cur = g_ps_perf;
    float secs = elapsed / 1e6f;
    uint32_t shown   = cur.shown   - prev.shown;
    uint32_t decoded = cur.decoded - prev.decoded;
    uint32_t dropped = cur.dropped - prev.dropped;
    float per_dec  = decoded ? 1000.0f * decoded : 1000.0f;  // µs total → ms per frame
    float per_show = shown   ? 1000.0f * shown   : 1000.0f;
#define D(stage) ((float)(cur.stage_us[stage] - prev.stage_us[stage]))

    ESP_LOGI("PERF",
             "%dx%d %.1f fps (decoded %.1f, dropped %lu) %lu kbps | "
             "rx core ms: net %.1f dec %.1f (max %.1f) copy %.1f | "
             "render core ms: yuv %.1f scale %.1f disp %.1f | rtp gaps %lu lost %lu",
             s_input_width, s_input_height,
             shown / secs, decoded / secs, (unsigned long)dropped,
             (unsigned long)((cur.bytes - prev.bytes) * 8 / 1000 / secs),
             D(PS_STAGE_NET) / per_dec, D(PS_STAGE_DECODE) / per_dec,
             cur.decode_max_us / 1000.0f, D(PS_STAGE_COPY) / per_dec,
             D(PS_STAGE_CONVERT) / per_show, D(PS_STAGE_SCALE) / per_show,
             D(PS_STAGE_DISPLAY) / per_show,
             (unsigned long)(cur.rtp_gaps - prev.rtp_gaps),
             (unsigned long)(cur.rtp_lost - prev.rtp_lost));
#undef D
    prev = cur;
    window_start_us = now;
    g_ps_perf.window++;
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
    ESP_LOGI(TAG, "Render: PPA %s into double-buffered frame buffer",
             s_ppa_yuv_ok ? "YUV420→RGB565 convert + scale" : "scale");
}

// Fit-height scale of an RGB565 frame into the back buffer, centred, with
// horizontal crop (wide input) or black bars (narrow input), then swap.
// Returns false if the PPA rejected the job; the caller then uses the CPU path.
static bool render_hw(const void *src, ppa_srm_color_mode_t src_cm, uint16_t w, uint16_t h)
{
    const bool yuv_in = (src_cm == PPA_SRM_COLOR_MODE_YUV420);
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
    if (yuv_in) { blk_w &= ~1u; in_off_x &= ~1u; }   // PPA: YUV420 blocks must be even
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
            .buffer = src, .pic_w = w, .pic_h = h,
            .block_w = blk_w, .block_h = h,
            .block_offset_x = in_off_x, .block_offset_y = 0,
            .srm_cm = src_cm,
            // H.264 from x264/OBS is limited-range BT.601 by default.
            .yuv_range = PPA_COLOR_RANGE_LIMIT,
            .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
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
        if (yuv_in) {
            ESP_LOGE(TAG, "PPA YUV420 input rejected (%s) for %dx%d — converting on the CPU instead",
                     esp_err_to_name(err), w, h);
            s_ppa_yuv_ok = false;
        } else {
            ESP_LOGE(TAG, "PPA scale failed (%s) for %dx%d — switching to CPU path",
                     esp_err_to_name(err), w, h);
            s_hw_render = false;
        }
        return false;
    }

    t = ps_now_us();
    display_present(fb);
    ps_perf_add(PS_STAGE_DISPLAY, t);
    return true;
}

// ─── Render (render task, or receiver task if the pipeline isn't running) ───

// Convert, scale and show one picture: planar I420 (pic) or packed YUV420
// (packed). If slot >= 0 it is returned to the free queue as soon as the YUV
// data has been consumed.
static void render_frame(const yuv_view_t *pic, const uint8_t *packed,
                         uint16_t width, uint16_t height, int slot)
{
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
            s_input_width = s_input_height = 0;
            if (slot >= 0) { uint8_t i = slot; xQueueSend(s_free_q, &i, 0); }
            return;
        }
        s_input_width  = width;
        s_input_height = height;
        s_clear_pending = 2;   // both frame buffers
    }

    s_frame_count++;
    PS_LOGV(TAG, "Frame #%lu: %dx%d", (unsigned long)s_frame_count, width, height);

    int64_t t;
    if (packed) {
        // PPA converts and scales in one pass; the slot is read by the PPA,
        // so it can only be released afterwards.
        if (s_hw_render && s_ppa_yuv_ok &&
                render_hw(packed, PPA_SRM_COLOR_MODE_YUV420, width, height)) {
            if (slot >= 0) { uint8_t i = slot; xQueueSend(s_free_q, &i, 0); }
            ps_perf_frame_done();
            return;
        }
        t = ps_now_us();
        yuv420_packed_to_rgb565(packed, s_rgb_buffer, width, height);
        ps_perf_add(PS_STAGE_CONVERT, t);
    } else {
#if PS_DEBUG
        uint32_t y_sum = 0;
        for (int r = 0; r < height; r += 8) y_sum += pic->y[r * pic->y_stride];
        ESP_LOGI(TAG, "Y checksum=0x%08lx Y[0]=%u Y[1]=%u",
                 (unsigned long)y_sum, pic->y[0], pic->y[1]);
#endif
        t = ps_now_us();
        yuv420_to_rgb565_fast(pic->y, pic->u, pic->v, pic->y_stride, pic->uv_stride,
                              s_rgb_buffer, width, height);
        ps_perf_add(PS_STAGE_CONVERT, t);
    }

    // YUV consumed: let the receiver refill this slot while we scale.
    if (slot >= 0) { uint8_t i = slot; xQueueSend(s_free_q, &i, 0); }

#if PS_DEBUG
    uint32_t rgb_sum = 0;
    for (int i = 0; i < (int)(width * height); i += 31) rgb_sum += s_rgb_buffer[i];
    ESP_LOGI(TAG, "RGB565 checksum=0x%08lx RGB[0]=0x%04x",
             (unsigned long)rgb_sum, s_rgb_buffer[0]);
#endif

    if (s_hw_render && render_hw(s_rgb_buffer, PPA_SRM_COLOR_MODE_RGB565, width, height)) {
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

static void render_task(void *arg)
{
    ESP_LOGI(TAG, "Render task started (core %d)", xPortGetCoreID());
    while (1) {
        uint8_t i;
        if (xQueueReceive(s_ready_q, &i, portMAX_DELAY) != pdTRUE) continue;
        ps_frame_slot_t *s = &s_slots[i];
        if (s->packed) {
            render_frame(NULL, s->yuv, s->w, s->h, i);
            continue;
        }
        size_t ysz = (size_t)s->w * s->h, csz = (size_t)((s->w + 1) / 2) * ((s->h + 1) / 2);
        yuv_view_t pic = {
            .y = s->yuv, .u = s->yuv + ysz, .v = s->yuv + ysz + csz,
            .y_stride = s->w, .uv_stride = (s->w + 1) / 2,
        };
        render_frame(&pic, NULL, s->w, s->h, i);
    }
}

static void pipeline_start(void)
{
    s_free_q  = xQueueCreate(PS_FRAME_SLOTS, sizeof(uint8_t));
    s_ready_q = xQueueCreate(PS_FRAME_SLOTS, sizeof(uint8_t));
    if (!s_free_q || !s_ready_q) {
        ESP_LOGW(TAG, "Pipeline: no queues, rendering on the receiver task");
        return;
    }
    for (uint8_t i = 0; i < PS_FRAME_SLOTS; i++) xQueueSend(s_free_q, &i, 0);
    if (xTaskCreatePinnedToCore(render_task, "render", PS_RENDER_STACK, NULL,
                                PS_RENDER_PRIO, NULL, PS_RENDER_CORE) != pdPASS) {
        ESP_LOGW(TAG, "Pipeline: no render task, rendering on the receiver task");
        return;
    }
    s_pipelined = true;
}

// ─── Frame callback (receiver task, called from the H.264 decoder) ───────────

static void on_frame_decoded(const h264_frame_t *frame, void *user_ctx)
{
    if (!frame || !frame->data || frame->size == 0) return;

    uint16_t coded_w = frame->coded_w, coded_h = frame->coded_h;
    uint16_t x0 = frame->x, y0 = frame->y;
    uint16_t width = frame->width, height = frame->height;

    // Fallback when the decoder couldn't report the size: infer it from the
    // I420 buffer size (W×H = size×2/3), assuming a square, uncropped frame.
    if (coded_w == 0 || coded_h == 0 || width == 0 || height == 0) {
        uint32_t pixels = (frame->size * 2) / 3;
        uint16_t side   = (uint16_t)sqrtf((float)pixels);
        if (side > 0 && (uint32_t)side * side == pixels) {
            coded_w = coded_h = width = height = side;
        } else {
            coded_w = width  = s_last_dec_w ? s_last_dec_w : 256;
            coded_h = height = s_last_dec_h ? s_last_dec_h : 256;
        }
        x0 = y0 = 0;
    }
    if (frame->size < (uint32_t)coded_w * coded_h * 3 / 2 ||
            x0 + width > coded_w || y0 + height > coded_h) {
        ESP_LOGW(TAG, "Bad frame geometry: %lu bytes, coded %dx%d, visible %dx%d at %d,%d",
                 (unsigned long)frame->size, coded_w, coded_h, width, height, x0, y0);
        return;
    }
    s_last_dec_w = coded_w;
    s_last_dec_h = coded_h;
    if (PS_PERF) g_ps_perf.decoded++;

    // Visible window inside the decoder's buffer (crop offsets are even).
    const uint8_t *Y = frame->data;
    const uint8_t *U = Y + (size_t)coded_w * coded_h;
    const uint8_t *V = U + (size_t)(coded_w / 2) * (coded_h / 2);
    yuv_view_t src = {
        .y = Y + (size_t)y0 * coded_w + x0,
        .u = U + (size_t)(y0 / 2) * (coded_w / 2) + x0 / 2,
        .v = V + (size_t)(y0 / 2) * (coded_w / 2) + x0 / 2,
        .y_stride = coded_w, .uv_stride = coded_w / 2,
    };

    if (!s_pipelined) {
        // frame->data is only valid during this callback, so render it now.
        render_frame(&src, NULL, width, height, -1);
        return;
    }

    // The decoder reuses its buffer on the next call, so copy the visible
    // area into a compact slot (this is also where the crop happens).
    uint8_t i;
    if (xQueueReceive(s_free_q, &i, 0) != pdTRUE) {
        if (PS_PERF) g_ps_perf.dropped++;   // render still busy with both slots
        return;
    }
    ps_frame_slot_t *slot = &s_slots[i];
    int cw = (width + 1) / 2, ch = (height + 1) / 2;
    bool pack = s_hw_render && s_ppa_yuv_ok && !(width & 1) && !(height & 1);
    size_t need = (size_t)width * height + 2 * (size_t)cw * ch;  // same size either way
    if (slot->cap < need) {
        free(slot->yuv);
        slot->yuv = heap_caps_malloc(need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!slot->yuv) slot->yuv = malloc(need);
        slot->cap = slot->yuv ? need : 0;
        if (!slot->yuv) {
            ESP_LOGE(TAG, "Failed to allocate frame slot for %dx%d", width, height);
            xQueueSend(s_free_q, &i, 0);
            return;
        }
    }
    int64_t t = ps_now_us();
    if (pack) {
        i420_to_yuv420_packed(src.y, src.u, src.v, src.y_stride, src.uv_stride,
                              slot->yuv, width, height);
    } else {
        uint8_t *dy = slot->yuv, *du = dy + (size_t)width * height, *dv = du + (size_t)cw * ch;
        if (src.y_stride == width) {
            memcpy(dy, src.y, (size_t)width * height);
        } else {
            for (int r = 0; r < height; r++) memcpy(dy + (size_t)r * width, src.y + (size_t)r * src.y_stride, width);
        }
        for (int r = 0; r < ch; r++) {
            memcpy(du + (size_t)r * cw, src.u + (size_t)r * src.uv_stride, cw);
            memcpy(dv + (size_t)r * cw, src.v + (size_t)r * src.uv_stride, cw);
        }
    }
    ps_perf_add(PS_STAGE_COPY, t);
    slot->packed = pack;
    slot->w = width;
    slot->h = height;
    xQueueSend(s_ready_q, &i, 0);
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
    pipeline_start();

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
