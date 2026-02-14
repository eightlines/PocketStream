#ifndef H264_H
#define H264_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Decoded frame callback function type
 * 
 * @param frame_data Pointer to decoded YUV420 (I420) frame data
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 * @param size Frame data size in bytes
 * @param user_ctx User context passed during init
 */
typedef void (*h264_frame_callback_t)(uint8_t *frame_data, uint16_t width, uint16_t height, uint32_t size, void *user_ctx);

/**
 * @brief H.264 decoder configuration
 */
typedef struct {
    h264_frame_callback_t on_frame_decoded;  /**< Callback when frame is decoded */
    void *user_ctx;                           /**< User context for callback */
} h264_decoder_config_t;

/**
 * @brief Initialize the H.264 decoder
 * 
 * @param config Decoder configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h264_decoder_init(const h264_decoder_config_t *config);

/**
 * @brief Decode H.264 NAL unit data
 * 
 * Feeds H.264 encoded data to the decoder. When a complete frame is decoded,
 * the on_frame_decoded callback will be invoked.
 * 
 * @param data Pointer to H.264 encoded data (NAL units)
 * @param size Size of the data in bytes
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t h264_decoder_decode(const uint8_t *data, uint32_t size);

/**
 * @brief Deinitialize and clean up the H.264 decoder
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h264_decoder_deinit(void);

/**
 * @brief Check if decoder is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool h264_decoder_is_initialized(void);

#endif // H264_H
