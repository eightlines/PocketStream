/**
 * @file my_jpeg_decoder.h
 * @brief JPEG decoder wrapper for ESP32-P4 (wraps esp_jpeg component)
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief JPEG decoder output format
 */
typedef enum {
    MY_JPEG_OUT_RGB565,
    MY_JPEG_OUT_RGB888,
} my_jpeg_output_format_t;

/**
 * @brief JPEG decoder configuration
 */
typedef struct {
    my_jpeg_output_format_t output_format;  ///< Output pixel format
    uint16_t max_width;                      ///< Maximum expected image width
    uint16_t max_height;                     ///< Maximum expected image height
} my_jpeg_decoder_config_t;

/**
 * @brief Initialize JPEG decoder
 * 
 * @param config Decoder configuration
 * @return ESP_OK on success
 */
esp_err_t my_jpeg_decoder_init(const my_jpeg_decoder_config_t *config);

/**
 * @brief Decode a JPEG image
 * 
 * @param jpeg_data Pointer to JPEG data
 * @param jpeg_size Size of JPEG data in bytes
 * @param output_buffer Output buffer for decoded pixels (must be pre-allocated)
 * @param output_size Size of output buffer
 * @param out_width Pointer to receive decoded image width
 * @param out_height Pointer to receive decoded image height
 * @return ESP_OK on success
 */
esp_err_t my_jpeg_decoder_decode(const uint8_t *jpeg_data, size_t jpeg_size,
                                 uint8_t *output_buffer, size_t output_size,
                                 uint16_t *out_width, uint16_t *out_height);

/**
 * @brief Deinitialize JPEG decoder
 */
void my_jpeg_decoder_deinit(void);

#ifdef __cplusplus
}
#endif
