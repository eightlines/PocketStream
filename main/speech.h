#pragma once

#include "esp_err.h"

/**
 * @file speech.h
 * @brief Wake-word detection + audio streaming to host for Whisper transcription.
 *
 * Architecture (on device):
 *   I2S (ES8311 mic) → ESP-SR AFE (WakeNet9 "Hi ESP" + VAD) → ring buffer
 *
 * When the wake word is detected the module buffers audio.
 * Once VAD reports >SPEECH_SILENCE_MS of silence it connects a TCP socket
 * to SPEECH_HOST:SPEECH_PORT and streams raw PCM16 @ 16 kHz mono preceded
 * by a 16-byte header.  The host (e.g. whisper_host.py) transcribes locally.
 *
 * Hardware (Waveshare ESP32-P4-86-Panel-ETH-2RO):
 *   ES8311 I2C:  I2C_NUM_0, SDA=GPIO7,  SCL=GPIO8   ← verify vs board schematic
 *   I2S (audio): MCLK=GPIO13, BCLK=GPIO12, LRCK=GPIO10, DIN=GPIO11 (ES7210 SDOUT)
 *
 * Prerequisites:
 *   1. Add to main/idf_component.yml:  espressif/esp-sr: "*"
 *   2. idf.py menuconfig → Component config → ESP Speech Recognition
 *        - Enable WakeNet, select "Hi ESP" (wn9_hiesp)
 *        - Enable VAD
 *   3. Or add to sdkconfig.defaults:
 *        CONFIG_SR_WN_WN9_HIESP=y
 *        CONFIG_ESP_GMOUSE_GESTURES=n  (avoid model name collisions)
 */

/**
 * @brief Initialize the speech module.
 *
 * Initialises the I2C bus, configures the ES8311 codec for 16 kHz mono
 * microphone input, sets up the I2S receive channel, and creates the
 * ESP-SR AFE instance (WakeNet9 + VAD).
 *
 * Call once from app_main after network is up.
 *
 * @return ESP_OK on success, ESP_FAIL on any hardware or allocation error.
 */
esp_err_t speech_init(void);

/**
 * @brief Start the speech capture task.
 *
 * Spawns a FreeRTOS task pinned to core 0 that:
 *   1. Listens for the wake word "Hi ESP" via WakeNet9.
 *   2. Records audio into a PSRAM ring buffer.
 *   3. Stops recording after SPEECH_SILENCE_MS of silence (VAD).
 *   4. Connects a TCP socket to SPEECH_HOST:SPEECH_PORT and sends the
 *      captured PCM with a 16-byte header, then closes the socket.
 *   5. Returns to listening.
 */
void speech_start(void);

/**
 * @brief Stop the speech capture task and release resources.
 */
void speech_stop(void);
