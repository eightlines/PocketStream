#pragma once

#include "esp_err.h"
#include "h264.h"

/**
 * @brief Start the RTSP/RTP client.
 *
 * Allocates the NAL reassembly buffer, initializes the H.264 decoder,
 * connects to the RTSP server, performs the RTSP handshake, and spawns:
 *   - rtp_rx:     reads interleaved RTP from the TCP connection
 *   - rtsp_super: monitors for inactivity and reconnects automatically
 *
 * The decoder_cfg is copied internally and reused on every reconnect when
 * the decoder is re-initialized.
 *
 * @return ESP_OK on success, ESP_FAIL on any allocation or connection error.
 */
esp_err_t rtsp_client_start(const h264_decoder_config_t *decoder_cfg);

/**
 * @brief Stop the RTSP client.
 *
 * Signals tasks to exit, closes the TCP socket, waits for the receiver
 * task, frees the NAL buffer, and deinitializes the H.264 decoder.
 */
void rtsp_client_stop(void);
