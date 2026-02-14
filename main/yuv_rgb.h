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
