#include "yuv_rgb.h"
#include <stdbool.h>

// Clamp value to 0-255 range
static inline uint8_t clamp_uint8(int value)
{
    if (value < 0) return 0;
    if (value > 255) return 255;
    return (uint8_t)value;
}

void yuv420_to_rgb565(const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane,
                      uint16_t *rgb_out, uint16_t width, uint16_t height)
{
    uint16_t uv_width = width / 2;
    
    for (uint16_t y = 0; y < height; y++) {
        for (uint16_t x = 0; x < width; x++) {
            // Get Y value
            uint8_t Y = y_plane[y * width + x];
            
            // Get U and V values (subsampled 2x2)
            uint16_t uv_x = x / 2;
            uint16_t uv_y = y / 2;
            uint8_t U = u_plane[uv_y * uv_width + uv_x];
            uint8_t V = v_plane[uv_y * uv_width + uv_x];
            
            // YUV to RGB conversion (BT.601)
            int C = Y;
            int D = U - 128;
            int E = V - 128;
            
            int R = C + ((359 * E) >> 8);
            int G = C - ((88 * D + 183 * E) >> 8);
            int B = C + ((454 * D) >> 8);
            
            // Clamp to 0-255
            uint8_t r = clamp_uint8(R);
            uint8_t g = clamp_uint8(G);
            uint8_t b = clamp_uint8(B);
            
            // Pack into RGB565: RRRRRGGG GGGBBBBB
            uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            rgb_out[y * width + x] = rgb565;
        }
    }
}

void i420_to_rgb565(const uint8_t *i420_data, uint16_t *rgb_out, uint16_t width, uint16_t height)
{
    // I420 layout: Y plane, then U plane, then V plane
    size_t y_size = (size_t)width * height;
    size_t uv_size = y_size / 4;
    
    const uint8_t *y_plane = i420_data;
    const uint8_t *u_plane = i420_data + y_size;
    const uint8_t *v_plane = i420_data + y_size + uv_size;
    
    yuv420_to_rgb565(y_plane, u_plane, v_plane, rgb_out, width, height);
}

// ─── Fast path ────────────────────────────────────────────────────────────────

// Colorimetry. H.264 from x264 / NVENC is normally limited range (Y 16..235,
// U/V 16..240) unless the SPS says otherwise; BT.709 vs BT.601 comes from the
// SPS matrix coefficients. The legacy converter above is full-range BT.601.
static bool s_full_range = false;
static bool s_bt709      = false;

// Clamp-and-pack tables, indexed by (value + LUT_OFS). The widest case is
// limited-range BT.709 blue: Y' -18..278 plus up to ±271 of chroma.
#define LUT_OFS  384
#define LUT_SIZE 1024
static uint16_t s_r_lut[LUT_SIZE], s_g_lut[LUT_SIZE], s_b_lut[LUT_SIZE];
static int16_t  s_y_lut[256];       // Y → scaled Y' + LUT_OFS
static int      s_cr, s_cgu, s_cgv, s_cb, s_round;   // chroma coefficients ×256
static bool     s_lut_ready = false;

static void build_luts(void)
{
    for (int i = 0; i < LUT_SIZE; i++) {
        uint8_t c = clamp_uint8(i - LUT_OFS);
        s_r_lut[i] = (uint16_t)((c & 0xF8) << 8);
        s_g_lut[i] = (uint16_t)((c & 0xFC) << 3);
        s_b_lut[i] = (uint16_t)(c >> 3);
    }
    if (s_full_range) {
        for (int y = 0; y < 256; y++) s_y_lut[y] = (int16_t)(y + LUT_OFS);
        if (s_bt709) { s_cr = 403; s_cgu = 48;  s_cgv = 120; s_cb = 475; s_round = 128; }
        else         { s_cr = 359; s_cgu = 88;  s_cgv = 183; s_cb = 454; s_round = 0;   } // = legacy
    } else {
        // Y' = 1.164 (Y - 16); chroma scaled by 255/224.
        for (int y = 0; y < 256; y++) s_y_lut[y] = (int16_t)(((298 * (y - 16) + 128) >> 8) + LUT_OFS);
        if (s_bt709) { s_cr = 459; s_cgu = 55;  s_cgv = 136; s_cb = 541; s_round = 128; }
        else         { s_cr = 409; s_cgu = 100; s_cgv = 208; s_cb = 516; s_round = 128; }
    }
    s_lut_ready = true;
}

void yuv_rgb_set_colorimetry(bool full_range, bool bt709)
{
    if (s_lut_ready && full_range == s_full_range && bt709 == s_bt709) return;
    s_full_range = full_range;
    s_bt709 = bt709;
    build_luts();
}

#define CHROMA(D, E) \
    int rd = (s_cr * (E) + s_round) >> 8; \
    int gd = (s_cgu * (D) + s_cgv * (E) + s_round) >> 8; \
    int bd = (s_cb * (D) + s_round) >> 8;
#define PIX(Yl) (uint16_t)(s_r_lut[(Yl) + rd] | s_g_lut[(Yl) - gd] | s_b_lut[(Yl) + bd])

void yuv420_to_rgb565_fast(const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane,
                           int y_stride, int uv_stride,
                           uint16_t *rgb_out, int width, int height)
{
    if (!s_lut_ready) build_luts();

    // Odd sizes or unaligned output: plain per-pixel loop with the same math.
    if ((width & 1) || (height & 1) || ((uintptr_t)rgb_out & 3)) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int Yl = s_y_lut[y_plane[y * y_stride + x]];
                int D = u_plane[(y / 2) * uv_stride + x / 2] - 128;
                int E = v_plane[(y / 2) * uv_stride + x / 2] - 128;
                CHROMA(D, E)
                rgb_out[y * width + x] = PIX(Yl);
            }
        }
        return;
    }

    for (int y = 0; y < height; y += 2) {
        const uint8_t *y0 = y_plane + y * y_stride;
        const uint8_t *y1 = y0 + y_stride;
        const uint8_t *u  = u_plane + (y / 2) * uv_stride;
        const uint8_t *v  = v_plane + (y / 2) * uv_stride;
        uint32_t *o0 = (uint32_t *)(rgb_out + y * width);
        uint32_t *o1 = (uint32_t *)(rgb_out + (y + 1) * width);
        for (int x = 0; x < width; x += 2) {
            int D = *u++ - 128;
            int E = *v++ - 128;
            CHROMA(D, E)
            int a = s_y_lut[y0[x]], b = s_y_lut[y0[x + 1]];
            int c = s_y_lut[y1[x]], d = s_y_lut[y1[x + 1]];
            *o0++ = (uint32_t)PIX(a) | ((uint32_t)PIX(b) << 16);   // little-endian: left pixel low
            *o1++ = (uint32_t)PIX(c) | ((uint32_t)PIX(d) << 16);
        }
    }
}

// ─── Packed YUV420 (P4 hardware layout) ───────────────────────────────────────

void i420_to_yuv420_packed(const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane,
                           int y_stride, int uv_stride,
                           uint8_t *out, int width, int height)
{
    const int row_bytes = width * 3 / 2;
    for (int y = 0; y < height; y += 2) {
        const uint8_t *y0 = y_plane + y * y_stride;
        const uint8_t *y1 = y0 + y_stride;
        const uint8_t *u  = u_plane + (y / 2) * uv_stride;
        const uint8_t *v  = v_plane + (y / 2) * uv_stride;
        uint8_t *o0 = out + (size_t)y * row_bytes;
        uint8_t *o1 = o0 + row_bytes;
        for (int x = 0; x < width; x += 2) {
            o0[0] = *u++; o0[1] = y0[x]; o0[2] = y0[x + 1]; o0 += 3;
            o1[0] = *v++; o1[1] = y1[x]; o1[2] = y1[x + 1]; o1 += 3;
        }
    }
}

void yuv420_packed_to_rgb565(const uint8_t *in, uint16_t *rgb_out, int width, int height)
{
    if (!s_lut_ready) build_luts();
    const int row_bytes = width * 3 / 2;
    for (int y = 0; y < height; y += 2) {
        const uint8_t *r0 = in + (size_t)y * row_bytes;
        const uint8_t *r1 = r0 + row_bytes;
        uint16_t *o0 = rgb_out + (size_t)y * width;
        uint16_t *o1 = o0 + width;
        for (int x = 0; x < width; x += 2, r0 += 3, r1 += 3) {
            int D = r0[0] - 128, E = r1[0] - 128;
            CHROMA(D, E)
            o0[x]     = PIX(s_y_lut[r0[1]]);
            o0[x + 1] = PIX(s_y_lut[r0[2]]);
            o1[x]     = PIX(s_y_lut[r1[1]]);
            o1[x + 1] = PIX(s_y_lut[r1[2]]);
        }
    }
}

