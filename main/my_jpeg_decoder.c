/**
 * @file jpeg_decoder.c
 * @brief JPEG decoder wrapper using esp_jpeg component
 */

#include "my_jpeg_decoder.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

// esp_jpeg component header
#include "jpeg_decoder.h"

static const char *TAG = "JPEG_DEC";

static esp_jpeg_image_format_t s_output_format = JPEG_IMAGE_FORMAT_RGB565;
static bool s_initialized = false;

esp_err_t my_jpeg_decoder_init(const my_jpeg_decoder_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "JPEG decoder already initialized");
        return ESP_OK;
    }

    s_output_format = (config->output_format == MY_JPEG_OUT_RGB565) 
                      ? JPEG_IMAGE_FORMAT_RGB565 
                      : JPEG_IMAGE_FORMAT_RGB888;

    s_initialized = true;
    ESP_LOGI(TAG, "JPEG software decoder initialized (format: %s)",
             config->output_format == MY_JPEG_OUT_RGB565 ? "RGB565" : "RGB888");
    
    return ESP_OK;
}

esp_err_t my_jpeg_decoder_decode(const uint8_t *jpeg_data, size_t jpeg_size,
                                 uint8_t *output_buffer, size_t output_size,
                                 uint16_t *out_width, uint16_t *out_height)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "JPEG decoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (jpeg_data == NULL || output_buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure decode operation
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)jpeg_data,
        .indata_size = jpeg_size,
        .outbuf = output_buffer,
        .outbuf_size = output_size,
        .out_format = s_output_format,
        .out_scale = JPEG_IMAGE_SCALE_0,  // No scaling
        .flags = {
            .swap_color_bytes = 1,  // For display compatibility (BGR vs RGB)
        },
        .advanced = {
            .working_buffer = NULL,  // Let decoder allocate
            .working_buffer_size = 0,
        },
    };

    esp_jpeg_image_output_t outimg = {0};
    
    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "JPEG decode failed: %s (size=%u)", esp_err_to_name(ret), (unsigned)jpeg_size);
        return ret;
    }

    ESP_LOGD(TAG, "Decoded %ux%u", outimg.width, outimg.height);

    if (out_width) *out_width = outimg.width;
    if (out_height) *out_height = outimg.height;

    return ESP_OK;
}

void my_jpeg_decoder_deinit(void)
{
    s_initialized = false;
    ESP_LOGI(TAG, "JPEG decoder deinitialized");
}
