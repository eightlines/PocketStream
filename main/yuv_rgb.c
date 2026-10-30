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

// Clamp-and-pack tables, indexed by (value + 256). Covers every value the
// BT.601 terms below can produce (Y 0..255 plus chroma offsets -227..+227).
#define LUT_OFS  256
#define LUT_SIZE 768
static uint16_t s_r_lut[LUT_SIZE], s_g_lut[LUT_SIZE], s_b_lut[LUT_SIZE];
static bool     s_lut_ready = false;

static void build_luts(void)
{
    for (int i = 0; i < LUT_SIZE; i++) {
        uint8_t c = clamp_uint8(i - LUT_OFS);
        s_r_lut[i] = (uint16_t)((c & 0xF8) << 8);
        s_g_lut[i] = (uint16_t)((c & 0xFC) << 3);
        s_b_lut[i] = (uint16_t)(c >> 3);
    }
    s_lut_ready = true;
}

#define PIX(Y) (uint16_t)(s_r_lut[(Y) + rd] | s_g_lut[(Y) - gd] | s_b_lut[(Y) + bd])

void yuv420_to_rgb565_fast(const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane,
                           int y_stride, int uv_stride,
                           uint16_t *rgb_out, int width, int height)
{
    if (!s_lut_ready) build_luts();

    // Odd sizes or unaligned output: plain per-pixel loop with the same math.
    if ((width & 1) || (height & 1) || ((uintptr_t)rgb_out & 3)) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int Y = y_plane[y * y_stride + x] + LUT_OFS;
                int D = u_plane[(y / 2) * uv_stride + x / 2] - 128;
                int E = v_plane[(y / 2) * uv_stride + x / 2] - 128;
                int rd = (359 * E) >> 8, gd = (88 * D + 183 * E) >> 8, bd = (454 * D) >> 8;
                rgb_out[y * width + x] = PIX(Y);
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
            int rd = (359 * E) >> 8;
            int gd = (88 * D + 183 * E) >> 8;
            int bd = (454 * D) >> 8;
            int a = y0[x] + LUT_OFS, b = y0[x + 1] + LUT_OFS;
            int c = y1[x] + LUT_OFS, d = y1[x + 1] + LUT_OFS;
            *o0++ = (uint32_t)PIX(a) | ((uint32_t)PIX(b) << 16);   // little-endian: left pixel low
            *o1++ = (uint32_t)PIX(c) | ((uint32_t)PIX(d) << 16);
        }
    }
}

