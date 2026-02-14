#include "yuv_rgb.h"

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
