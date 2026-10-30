#ifndef YUV_RGB_H
#define YUV_RGB_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Convert I420 (YUV420 planar) to RGB565
 * 
 * RGB565 is packed as 2 bytes per pixel: RRRRRGGG GGGBBBBB (big endian)
 * 
 * @param y_plane Pointer to Y plane data
 * @param u_plane Pointer to U plane data
 * @param v_plane Pointer to V plane data
 * @param rgb_out Pointer to output RGB565 buffer (must be width * height * 2 bytes)
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 */
void yuv420_to_rgb565(const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane,
                      uint16_t *rgb_out, uint16_t width, uint16_t height);

/**
 * @brief Convert I420 frame to RGB565 (single buffer input)
 * 
 * Assumes standard I420 layout: Y plane followed by U plane followed by V plane.
 * 
 * @param i420_data Pointer to I420 frame data
 * @param rgb_out Pointer to output RGB565 buffer (must be width * height * 2 bytes)
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 */
void i420_to_rgb565(const uint8_t *i420_data, uint16_t *rgb_out, uint16_t width, uint16_t height);

/**
 * @brief Fast I420 → RGB565 with plane strides (bit-identical to yuv420_to_rgb565)
 *
 * Works on 2×2 blocks so each U/V sample is read and its colour terms
 * computed once for four pixels, clamps through lookup tables, and writes two
 * pixels per 32-bit store. Strides allow converting a cropped window of a
 * larger decoded frame in place.
 *
 * @param y_plane   First visible Y sample
 * @param u_plane   First visible U sample
 * @param v_plane   First visible V sample
 * @param y_stride  Bytes per Y row
 * @param uv_stride Bytes per U/V row
 * @param rgb_out   Output, width × height, compact (2 bytes per pixel)
 * @param width     Visible width in pixels
 * @param height    Visible height in pixels
 */
void yuv420_to_rgb565_fast(const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane,
                           int y_stride, int uv_stride,
                           uint16_t *rgb_out, int width, int height);

/**
 * @brief Repack planar I420 into the P4's packed YUV420 layout
 *
 * The packed layout (ESP_H264_RAW_FMT_O_UYY_E_VYY, what the PPA calls
 * YUV420) stores each row as width × 1.5 bytes:
 *   even rows (0, 2, …):  U Y Y  U Y Y …
 *   odd rows  (1, 3, …):  V Y Y  V Y Y …
 * where each U/V byte covers a 2×2 block. Width and height must be even.
 *
 * @param out width × height × 3/2 bytes
 */
void i420_to_yuv420_packed(const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane,
                           int y_stride, int uv_stride,
                           uint8_t *out, int width, int height);

/**
 * @brief CPU conversion of packed YUV420 (see above) to RGB565
 *
 * Same BT.601 maths as yuv420_to_rgb565. Only used as a fallback when the
 * PPA can't do the conversion.
 */
void yuv420_packed_to_rgb565(const uint8_t *in, uint16_t *rgb_out, int width, int height);

/**
 * @brief Calculate RGB565 buffer size needed for given dimensions
 * 
 * @param width Frame width
 * @param height Frame height
 * @return Size in bytes needed for RGB565 buffer
 */
static inline size_t rgb565_buffer_size(uint16_t width, uint16_t height)
{
    return (size_t)width * height * 2;
}

/**
 * @brief Calculate I420 buffer size for given dimensions
 * 
 * @param width Frame width
 * @param height Frame height
 * @return Size in bytes for I420 frame
 */
static inline size_t i420_buffer_size(uint16_t width, uint16_t height)
{
    return (size_t)width * height * 3 / 2;
}

#endif // YUV_RGB_H
