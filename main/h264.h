#ifndef H264_H
#define H264_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief A decoded I420 frame.
 *
 * The decoder works in 16-pixel macroblocks, so the buffer holds the coded
 * size (e.g. 368×368 for a 360×360 stream). The visible picture is the
 * width×height rectangle at (x, y), taken from the SPS cropping fields.
 * Plane strides are coded_w (Y) and coded_w / 2 (U, V).
 */
typedef struct {
    uint8_t  *data;      /**< I420: Y plane, then U, then V (coded size) */
    uint32_t  size;      /**< Bytes in data */
    uint16_t  coded_w;   /**< Buffer width in pixels (0 if unknown) */
    uint16_t  coded_h;   /**< Buffer height in pixels (0 if unknown) */
    uint16_t  x, y;      /**< Top-left of the visible picture */
    uint16_t  width;     /**< Visible width (0 if unknown) */
    uint16_t  height;    /**< Visible height (0 if unknown) */
} h264_frame_t;

/**
 * @brief Decoded frame callback function type
 * 
 * @param frame Decoded frame (see h264_frame_t)
 * @param user_ctx User context passed during init
 *
 * Called synchronously from h264_decoder_decode(); frame->data is only valid
 * for the duration of the call.
 */
typedef void (*h264_frame_callback_t)(const h264_frame_t *frame, void *user_ctx);

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

/**
 * @brief Picture geometry from a sequence parameter set.
 */
typedef struct {
    uint16_t coded_w, coded_h;   /**< Macroblock-aligned size */
    uint16_t x, y;               /**< Crop offset (left, top) */
    uint16_t width, height;      /**< Visible size after cropping */
} h264_sps_info_t;

/**
 * @brief Parse picture size and cropping from an SPS NAL unit.
 *
 * @param nal SPS NAL unit starting at the NAL header byte (no start code)
 * @param len Length in bytes
 * @param out Parsed geometry
 * @return true on success
 */
bool h264_parse_sps(const uint8_t *nal, int len, h264_sps_info_t *out);

#endif // H264_H
