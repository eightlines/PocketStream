#include "h264.h"
#include "esp_log.h"
#include "esp_h264_dec.h"
#include "esp_h264_dec_sw.h"
#include "esp_h264_types.h"

static const char *TAG = "H264_DEC";

static esp_h264_dec_handle_t s_decoder = NULL;
static h264_frame_callback_t s_frame_callback = NULL;
static void *s_user_ctx = NULL;

esp_err_t h264_decoder_init(const h264_decoder_config_t *config)
{
    if (s_decoder != NULL) {
        ESP_LOGW(TAG, "Decoder already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Save callback
    s_frame_callback = config->on_frame_decoded;
    s_user_ctx = config->user_ctx;

    // Configure decoder for I420 output (YUV420 planar)
    esp_h264_dec_cfg_sw_t dec_cfg = {
        .pic_type = ESP_H264_RAW_FMT_I420,
    };

    // Create decoder
    esp_h264_err_t ret = esp_h264_dec_sw_new(&dec_cfg, &s_decoder);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to create decoder: %d", ret);
        return ESP_FAIL;
    }

    // Open decoder
    ret = esp_h264_dec_open(s_decoder);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to open decoder: %d", ret);
        esp_h264_dec_del(s_decoder);
        s_decoder = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "H.264 decoder initialized");
    return ESP_OK;
}

esp_err_t h264_decoder_decode(const uint8_t *data, uint32_t size)
{
    if (s_decoder == NULL) {
        ESP_LOGE(TAG, "Decoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid input data");
        return ESP_ERR_INVALID_ARG;
    }

    esp_h264_dec_in_frame_t in_frame = {
        .raw_data.buffer = (uint8_t *)data,
        .raw_data.len = size,
    };
    esp_h264_dec_out_frame_t out_frame = {0};

    // Process all NAL units in the buffer
    while (in_frame.raw_data.len > 0) {
        esp_h264_err_t ret = esp_h264_dec_process(s_decoder, &in_frame, &out_frame);
        if (ret != ESP_H264_ERR_OK) {
            // Decode error - skip this data and wait for next keyframe
            // This is expected with UDP packet loss
            break;
        }

        // Update buffer pointer for next iteration
        in_frame.raw_data.buffer += in_frame.consume;
        in_frame.raw_data.len -= in_frame.consume;

        // If we got a decoded frame, invoke callback
        if (out_frame.out_size > 0 && s_frame_callback != NULL) {
            // Note: Frame dimensions need to be tracked from SPS parsing
            // For now, we pass the raw output; caller should know expected dimensions
            s_frame_callback(out_frame.outbuf, 0, 0, out_frame.out_size, s_user_ctx);
        }
    }

    return ESP_OK;
}

esp_err_t h264_decoder_deinit(void)
{
    if (s_decoder == NULL) {
        ESP_LOGW(TAG, "Decoder not initialized");
        return ESP_OK;
    }

    esp_h264_dec_close(s_decoder);
    esp_h264_dec_del(s_decoder);
    s_decoder = NULL;
    s_frame_callback = NULL;
    s_user_ctx = NULL;

    ESP_LOGI(TAG, "H.264 decoder deinitialized");
    return ESP_OK;
}

bool h264_decoder_is_initialized(void)
{
    return s_decoder != NULL;
}
