/**
 * @file speech.c
 * @brief Wake-word detection (WakeNet9 "Hi ESP") + PCM streaming to host.
 *
 * Pipeline:
 *   I2S (ES7210 4-ch ADC, MIC1/MIC2) → ESP-SR AFE (WakeNet + VAD) → PSRAM buffer
 *   → TCP socket → host Whisper (commands/whisper_host.py)
 *
 * GPIO map (Waveshare ESP32-P4-86-Panel-ETH-2RO):
 *   I2C SDA=GPIO7  SCL=GPIO8  (shared ES8311 DAC + ES7210 ADC)
 *   I2S MCLK   : GPIO13 (4.096 MHz = 256 × 16 kHz, master out)
 *   I2S BCLK   : GPIO12
 *   I2S LRCK   : GPIO10
 *   I2S DIN    : GPIO11 (ES7210 SDOUT → mic samples, CH1=left=MIC1)
 *
 * ESP-SR v2.3.1 API used:
 *   esp_srmodel_init("model")     — loads models from flash "model" partition
 *   afe_config_init(...)           — builds AFE config
 *   esp_afe_handle_from_config()   — selects the right AFE implementation
 *   afe_handle->create_from_config() — creates AFE data instance
 *   afe_handle->feed()             — push raw PCM to AFE ring buffer
 *   afe_handle->fetch()            — pull processed result (blocks)
 *   result->wakeup_state           — WAKENET_DETECTED when wake word heard
 *   result->vad_state              — VAD_SILENCE after vad_min_noise_ms silence
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ESP-SR v2.3.1 headers */
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"  /* esp_afe_handle_from_config() */
#include "esp_afe_config.h"     /* afe_config_init(), afe_config_free() */
#include "esp_wn_models.h"      /* ESP_WN_PREFIX */
#include "model_path.h"         /* esp_srmodel_init(), esp_srmodel_filter() */

#include "speech.h"

static const char *TAG = "SPEECH";

// ─── Configuration ────────────────────────────────────────────────────────────

/* Host that receives the PCM stream (same machine as RTSP server). */
#define SPEECH_HOST         "10.0.0.1"
#define SPEECH_PORT         9000

/* Recording limits */
#define SPEECH_SAMPLE_RATE  16000
#define SPEECH_SILENCE_MS   1500    /* passed to vad_min_noise_ms; AFE waits this long before reporting VAD_SILENCE */
#define SPEECH_MAX_SECONDS  30
#define SPEECH_MAX_SAMPLES  (SPEECH_SAMPLE_RATE * SPEECH_MAX_SECONDS)
#define SPEECH_MAX_BYTES    (SPEECH_MAX_SAMPLES * sizeof(int16_t))

/* WakeNet9 model — must match CONFIG_SR_WN_WN9_HIESP=y (or wn9s_hiesp for small) */
#define WAKENET_MODEL_NAME  "wn9_hiesp"

/* UDP port for real-time audio monitoring (continuous raw-PCM stream).
 * ffmpeg 8.x: -ac is removed as input option; use -ch_layout mono instead.
 * Host: ffplay -f s16le -ar 16000 -ch_layout mono -nodisp udp://0.0.0.0:9001
 *       ffmpeg -f s16le -ar 16000 -ch_layout mono -i udp://0.0.0.0:9001 mic.wav
 * Streams the mono CH1 feed (pre-software-gain) so audio level matches
 * what the hardware actually captures, not the boosted WakeNet input. */
#define SPEECH_MONITOR_PORT 9001

/* I2C for ES8311 codec (verify GPIO nums against your board schematic) */
#define ES8311_I2C_NUM      I2C_NUM_0
#define ES8311_I2C_SDA      GPIO_NUM_7
#define ES8311_I2C_SCL      GPIO_NUM_8
#define ES8311_I2C_ADDR     0x18        /* 7-bit; ADDR pin = GND → 0x18 */
#define ES8311_I2C_SPEED_HZ 100000

/* I2S (ESP32-P4 as master; ES8311 is slave) */
#define I2S_NUM             I2S_NUM_1   /* keep 0 free for other uses */
#define I2S_MCLK_GPIO       GPIO_NUM_13
#define I2S_BCLK_GPIO       GPIO_NUM_12
#define I2S_LRCK_GPIO       GPIO_NUM_10
#define I2S_DIN_GPIO        GPIO_NUM_11 /* ES7210 SDOUT → ESP32-P4 I2S DIN (mic data) */
/* GPIO9 is ES8311 ASDIN (speaker output from ESP32) — not used here */

/* ES7210 — 4-channel ADC (microphone input on this board) */
#define ES7210_I2C_ADDR     0x40

/* Software gain applied to I2S samples before AFE feed.
 * ES7210 max safe hardware gain is 33dB (code 0x0B); codes above 0x0B are invalid.
 * At 33dB, normal speech at ~30cm gives ~500–700 LSB peaks (-34 dBFS).
 * WakeNet9 needs roughly -25 dBFS for reliable detection.
 * ×4 (+12dB) boosts 500→2000 LSB. Clips only at >32767/4 ≈ 8192 LSB raw.
 * NOTE: 3752 LSB raw was observed while shouting — loud speech already near clip
 * boundary with ×4. Clipped audio distorts MFCC features; reduce to ×2 if needed. */
#define SPEECH_SW_GAIN      4

// ─── ES8311 register addresses ────────────────────────────────────────────────

#define ES8311_REG_RESET   0x00
#define ES8311_REG_CLK01   0x01
#define ES8311_REG_CLK02   0x02
#define ES8311_REG_CLK03   0x03
#define ES8311_REG_CLK04   0x04
#define ES8311_REG_CLK05   0x05
#define ES8311_REG_CLK06   0x06
#define ES8311_REG_CLK07   0x07
#define ES8311_REG_CLK08   0x08
#define ES8311_REG_CLK09   0x09
#define ES8311_REG_CLK0A   0x0A
#define ES8311_REG_SYS0C   0x0C
#define ES8311_REG_SYS0D   0x0D
#define ES8311_REG_SYS0E   0x0E
#define ES8311_REG_SYS0F   0x0F
#define ES8311_REG_SYS10   0x10
#define ES8311_REG_SYS11   0x11
#define ES8311_REG_SYS13   0x13
#define ES8311_REG_SYS14   0x14
#define ES8311_REG_SDP15   0x15
#define ES8311_REG_SDP16   0x16
#define ES8311_REG_ADC17   0x17
#define ES8311_REG_ADC1C   0x1C
#define ES8311_REG_DAC31   0x31
#define ES8311_REG_DAC32   0x32
#define ES8311_REG_DAC37   0x37

// ─── ES8311 initialisation sequence ──────────────────────────────────────────
/*
 * I2S slave mode, 16 kHz, 16-bit, mono ADC (mic on LINPUT1).
 * MCLK = 256 × Fs = 4.096 MHz driven by ESP32-P4 I2S master output.
 * PGA gain ≈ +24 dB (reg 0x1C = 0x6A; adjust 0x00–0x7F ≈ 0–+36 dB).
 */
static const uint8_t s_es8311_init[][2] = {
    /* index 0 is the reset; written with a delay in es8311_init_codec() */
    {ES8311_REG_RESET,  0x1F},  /* software reset */
    {ES8311_REG_RESET,  0x00},  /* de-assert reset */
    /* Clock manager */
    {ES8311_REG_CLK01,  0x30},  /* MCLK from ext pin, no div, NFS=0 */
    {ES8311_REG_CLK02,  0x00},  /* CLK_SRC = MCLK, no pre-multiply */
    {ES8311_REG_CLK03,  0x10},  /* ADC OSR for 16 kHz */
    {ES8311_REG_CLK04,  0x00},  /* ADC LRCK_H (slave: not driven) */
    {ES8311_REG_CLK05,  0x00},  /* ADC LRCK_L (slave: not driven) */
    {ES8311_REG_CLK06,  0x03},  /* DAC OSR (not used) */
    {ES8311_REG_CLK07,  0x00},  /* DAC LRCK_H (slave) */
    {ES8311_REG_CLK08,  0x00},  /* DAC LRCK_L (slave) */
    {ES8311_REG_CLK09,  0x00},  /* BCLK div = 0 (slave) */
    {ES8311_REG_CLK0A,  0x00},  /* MSC / BCLK (slave) */
    /* System */
    {ES8311_REG_SYS0C,  0x00},  /* LP mode off */
    {ES8311_REG_SYS0D,  0x01},  /* VSEL = 3.3 V, analog ref enable */
    {ES8311_REG_SYS0E,  0x02},  /* power: ADC on, DAC off */
    {ES8311_REG_SYS0F,  0x44},  /* VMID / bias */
    {ES8311_REG_SYS10,  0x1F},  /* ADC clocks + MIC bias on */
    {ES8311_REG_SYS11,  0x7F},  /* DAC off */
    {ES8311_REG_SYS13,  0x10},  /* MIC boost +18 dB */
    {ES8311_REG_SYS14,  0x0A},  /* input mux: LINPUT1 single-ended */
    /* Serial data format */
    {ES8311_REG_SDP15,  0x00},  /* ADC: I2S standard, 16-bit, slave */
    {ES8311_REG_SDP16,  0x00},  /* DAC: same (unused) */
    /* ADC */
    {ES8311_REG_ADC17,  0xBF},  /* HPF on, ADC volume 0 dB */
    {ES8311_REG_ADC1C,  0x6A},  /* PGA ≈ +24 dB (range 0x00–0x7F) */
    /* DAC (muted, not used) */
    {ES8311_REG_DAC31,  0x60},
    {ES8311_REG_DAC32,  0x00},
    {ES8311_REG_DAC37,  0x48},
};

// ─── Module state ─────────────────────────────────────────────────────────────

static i2c_master_bus_handle_t       s_i2c_bus    = NULL;
static i2c_master_dev_handle_t       s_es8311_dev = NULL;
static i2c_master_dev_handle_t       s_es7210_dev = NULL;
static i2s_chan_handle_t             s_i2s_rx     = NULL;

static srmodel_list_t               *s_models     = NULL;  /* flash models */
static afe_config_t                 *s_afe_cfg    = NULL;
static const esp_afe_sr_iface_t     *s_afe_handle = NULL;
static esp_afe_sr_data_t            *s_afe_data   = NULL;

static int16_t *s_audio_buf      = NULL;
static volatile bool s_task_exit = false;
static TaskHandle_t  s_task_handle = NULL;

// ─── PCM send header ──────────────────────────────────────────────────────────

typedef struct __attribute__((packed)) {
    uint32_t magic;        /* 0x41554449 "AUDI" */
    uint32_t sample_rate;  /* 16000 */
    uint16_t channels;     /* 1 */
    uint16_t bits;         /* 16 */
    uint32_t num_samples;  /* count of int16_t samples */
} speech_pcm_header_t;

// ─── ES8311 helpers ───────────────────────────────────────────────────────────

static esp_err_t es8311_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_es8311_dev, buf, sizeof(buf), 100);
}

static esp_err_t es8311_init_codec(void)
{
    esp_err_t r = es8311_write(ES8311_REG_RESET, 0x1F);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 reset write failed (err=%d) — check I2C pins GPIO%d/GPIO%d addr=0x%02X",
                 r, ES8311_I2C_SDA, ES8311_I2C_SCL, ES8311_I2C_ADDR);
        return r;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Write remaining registers (skip index 0: reset already done) */
    int n = sizeof(s_es8311_init) / sizeof(s_es8311_init[0]);
    for (int i = 1; i < n; i++) {
        r = es8311_write(s_es8311_init[i][0], s_es8311_init[i][1]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "ES8311 reg 0x%02X write failed: %d",
                     s_es8311_init[i][0], r);
            return r;
        }
    }
    ESP_LOGI(TAG, "ES8311 initialized (16 kHz, 16-bit, mono ADC, PGA ~24 dB)");
    return ESP_OK;
}

// ─── ES7210 helpers ───────────────────────────────────────────────────────────
/*
 * ES7210 — 4-channel ADC used for microphone input on the Waveshare ESP32-P4-86-Panel.
 * Microphone data is output on ES7210 SDOUT → GPIO11 (I2S DIN).
 * Configured for: 16 kHz, 16-bit, I2S standard, 4.096 MHz MCLK (256×Fs).
 *
 * Register values derived from ESHome es7210 component and ESP-ADF reference.
 * For 16 kHz with MCLK=4.096 MHz: ADC_div=0 (÷1), OSR=0x20, LRCK=0x0100.
 */

static esp_err_t es7210_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_es7210_dev, buf, sizeof(buf), 100);
}

static uint8_t es7210_read(uint8_t reg)
{
    uint8_t val = 0xFF;
    i2c_master_transmit_receive(s_es7210_dev, &reg, 1, &val, 1, 100);
    return val;
}

static esp_err_t es7210_init_codec(void)
{
    esp_err_t r;

    /* Full reset */
    if ((r = es7210_write(0x00, 0xFF)) != ESP_OK) {
        ESP_LOGE(TAG, "ES7210 not found at I2C addr 0x%02X (err=%d)", ES7210_I2C_ADDR, r);
        return r;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Log REG01 reset default before writing it.
     * If it reads 0x00 → bit=1 means clock-OFF (our 0x3F was disabling clocks).
     * If it reads 0x3F → bit=1 means clock-ON (ESP-ADF interpretation correct). */
    {
        uint8_t reg01_default = es7210_read(0x01);
        ESP_LOGI(TAG, "ES7210 REG01 reset default: 0x%02X (0x00=clocks-on-by-def, 0x3F=clocks-off-by-def)", reg01_default);
    }

    /* Release reset to normal operation (matches ESP-ADF es7210_adc_init sequence) */
    es7210_write(0x00, 0x41);

    /* REG01: clock control.  The register is named "CLOCK_OFF_REG01" suggesting
     * bit=1 means that specific clock is DISABLED (off).
     * 0x3F = 0b00111111 = all 6 ADC clock bits set = ALL ADC CLOCKS DISABLED.
     * 0x00 = all bits clear = all ADC clocks enabled.
     *
     * The ESP-ADF driver writes 0x3F with comment "Enable clocks" which contradicts
     * the register name.  Our old code accidentally wrote 0x00 at the end
     * ("re-enable clocks") which may have been the accidental fix that allowed peak=1.
     * Write 0x00 here to keep all ADC clocks enabled throughout configuration. */
    es7210_write(0x01, 0x00);

    /* Timing: chip initial state period & power-up state period */
    es7210_write(0x09, 0x30);
    es7210_write(0x0A, 0x30);

    /* High-pass filter (order matches ESP-ADF driver) */
    es7210_write(0x23, 0x2A);  /* ADC12_HPF2 */
    es7210_write(0x22, 0x0A);  /* ADC12_HPF1 */
    es7210_write(0x20, 0x0A);  /* ADC34_HPF2 */
    es7210_write(0x21, 0x2A);  /* ADC34_HPF1 */

    /* REG08: master/slave + channel count.
     * bit0=0 → SLAVE mode (ESP32-P4 is I2S master; ES7210 must be slave).
     * Writing 0x01 here was the root cause of silence: it set ES7210 as I2S
     * master, conflicting with the ESP32-P4 master BCLK/LRCK output. */
    es7210_write(0x08, 0x00);

    /* Analog power: 0xC3 (bit7=1 adds extra reference/bias path vs 0x43).
     * ESP-ADF driver uses 0x43, but 0xC3 was previously giving non-zero
     * output on this board. Bit 7 may control VMID or LDO on this variant. */
    es7210_write(0x40, 0xC3);

    /* MIC bias = 2.87 V for CH1/CH2 and CH3/CH4 */
    es7210_write(0x41, 0x70);
    es7210_write(0x42, 0x70);

    /* Initial OSR write before MCLK divider setup */
    es7210_write(0x07, 0x20);

    /* REG02 (MAINCLK): MCLK=4.096 MHz, Fs=16 kHz.
     * From ESP-ADF coeff_div[] for {mclk=4096000, lrck=16000}:
     *   adc_div=0x01, doubler=1 (bit6), dll=1 (bit7)
     *   → 0x01 | 0x40 | 0x80 = 0xC1
     * (our previous 0x00 gave no MCLK division and disabled DLL — broken) */
    es7210_write(0x02, 0xC1);
    /* Give the DLL time to lock onto MCLK before writing further registers */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* OSR = 32 (after MCLK setup) */
    es7210_write(0x07, 0x20);

    /* LRCK divider = 256 → MCLK(4.096 MHz) / 256 = 16 kHz */
    es7210_write(0x04, 0x01);  /* LRCK_H */
    es7210_write(0x05, 0x00);  /* LRCK_L */

    /* REG11 (SDP_INTERFACE1): serial data port format.
     * bits[7:5]=011=0x60 → 16-bit word length.
     * bits[1:0]=00       → standard I2S (Philips) format.
     * This register was NEVER written before; ES7210 stayed at reset default
     * (likely 24-bit or some other format), causing no usable I2S output. */
    es7210_write(0x11, 0x60);

    /* REG12 (SDP_INTERFACE2): TDM mode.
     * 0x00 = TDM disabled → standard 2-channel I2S output (CH1=L, CH2=R).
     * 0x02 = TDM enabled  → needed only when 3 or 4 mics are selected. */
    es7210_write(0x12, 0x00);

    /* MIC gain (registers 0x43–0x46).
     * Format: bit4=1 (enable MIC) | gain_code.
     * Valid codes (3dB/step): 0x00=0dB, 0x04=12dB, 0x08=24dB, 0x09=27dB,
     *   0x0A=30dB, 0x0B=33dB (max confirmed valid on this board).
     * WARNING: codes 0x0C–0x0F appear INVALID on this ES7210 variant —
     *   writing 0x1F (code 0x0F) caused the ADC output to drop to ~2 LSB
     *   (completely dead), even though 0x0F=39dB was expected to be valid.
     *   Always use 0x0B (33dB) as the maximum safe gain code. */
    es7210_write(0x43, 0x1B);  /* MIC1: enable + 33 dB */
    es7210_write(0x44, 0x1B);  /* MIC2: enable + 33 dB */
    es7210_write(0x45, 0x1B);  /* MIC3: enable + 33 dB */
    es7210_write(0x46, 0x1B);  /* MIC4: enable + 33 dB */

    /* Individual MIC power-on (0x00 = all power stages on) */
    es7210_write(0x47, 0x00);  /* MIC1 power */
    es7210_write(0x48, 0x00);  /* MIC2 power */
    es7210_write(0x49, 0x00);  /* MIC3 power */
    es7210_write(0x4A, 0x00);  /* MIC4 power */

    /* Combined MIC power registers (MIC12 at 0x4B, MIC34 at 0x4C) */
    es7210_write(0x4B, 0x00);  /* MIC1+MIC2 bias/ADC/PGA power on */
    es7210_write(0x4C, 0x00);  /* MIC3+MIC4 bias/ADC/PGA power on */

    /* REG13: ADC automute control.
     * Reset default may have automute enabled; 0x00 disables it so the ADC
     * always outputs data regardless of input level. */
    es7210_write(0x13, 0x00);

    /* Final enable: 0x71 powers up analog (VMID + bias); wait for analog
     * to settle; 0x41 starts normal ADC operation.
     * (Skipping 0x71 and going straight to 0x41 left the analog path cold,
     * causing all-zero output despite correct digital configuration.) */
    es7210_write(0x00, 0x71);
    vTaskDelay(pdMS_TO_TICKS(50));
    es7210_write(0x00, 0x41);

    /* Read back key registers to confirm I2C writes landed.
     * If reads return 0xFF or 0x00 for writable fields, the chip may not be
     * at address 0x40, or the writes are being NAK-ed silently. */
    ESP_LOGI(TAG, "ES7210 reg dump:");
    ESP_LOGI(TAG, "  REG00=0x%02X (expect 0x41)", es7210_read(0x00));
    ESP_LOGI(TAG, "  REG01=0x%02X (wrote 0x00 = all ADC clocks enabled)", es7210_read(0x01));
    ESP_LOGI(TAG, "  REG02=0x%02X (expect 0xC1)", es7210_read(0x02));
    ESP_LOGI(TAG, "  REG08=0x%02X (expect 0x00 slave)", es7210_read(0x08));
    ESP_LOGI(TAG, "  REG11=0x%02X (expect 0x60 16-bit I2S)", es7210_read(0x11));
    ESP_LOGI(TAG, "  REG40=0x%02X (expect 0xC3)", es7210_read(0x40));
    ESP_LOGI(TAG, "  REG43=0x%02X (expect 0x1B gain+enable)", es7210_read(0x43));
    ESP_LOGI(TAG, "  REG47=0x%02X (expect 0x00 power on)", es7210_read(0x47));
    ESP_LOGI(TAG, "  REG4B=0x%02X (expect 0x00 power on)", es7210_read(0x4B));
    ESP_LOGI(TAG, "ES7210 initialized (16 kHz, 16-bit I2S slave, 33 dB mic gain)");
    return ESP_OK;
}

// ─── I2C init ─────────────────────────────────────────────────────────────────

static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = ES8311_I2C_NUM,
        .sda_io_num          = ES8311_I2C_SDA,
        .scl_io_num          = ES8311_I2C_SCL,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t r = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (r != ESP_OK) { ESP_LOGE(TAG, "I2C bus init fail: %d", r); return r; }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ES8311_I2C_ADDR,
        .scl_speed_hz    = ES8311_I2C_SPEED_HZ,
    };
    r = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_es8311_dev);
    if (r != ESP_OK) { ESP_LOGE(TAG, "ES8311 I2C device add fail: %d", r); return r; }

    i2c_device_config_t es7210_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ES7210_I2C_ADDR,
        .scl_speed_hz    = ES8311_I2C_SPEED_HZ,
    };
    r = i2c_master_bus_add_device(s_i2c_bus, &es7210_cfg, &s_es7210_dev);
    if (r != ESP_OK) { ESP_LOGE(TAG, "ES7210 I2C device add fail: %d", r); return r; }

    ESP_LOGI(TAG, "I2C ready (SDA=GPIO%d SCL=GPIO%d)", ES8311_I2C_SDA, ES8311_I2C_SCL);
    return ESP_OK;
}

// ─── I2S init ─────────────────────────────────────────────────────────────────

static esp_err_t i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    esp_err_t r = i2s_new_channel(&chan_cfg, NULL, &s_i2s_rx);
    if (r != ESP_OK) { ESP_LOGE(TAG, "I2S channel create fail: %d", r); return r; }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SPEECH_SAMPLE_RATE),
        /* ES7210 outputs standard I2S (Philips format: MSB 1 BCLK after LRCK edge).
         * Stereo mode to check both CH1 (left) and CH2 (right) — mic may be on CH2. */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_GPIO,
            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_LRCK_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN_GPIO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    r = i2s_channel_init_std_mode(s_i2s_rx, &std_cfg);
    if (r != ESP_OK) { ESP_LOGE(TAG, "I2S std mode init fail: %d", r); return r; }

    r = i2s_channel_enable(s_i2s_rx);
    if (r != ESP_OK) { ESP_LOGE(TAG, "I2S channel enable fail: %d", r); return r; }

    ESP_LOGI(TAG, "I2S ready (MCLK=GPIO%d BCLK=GPIO%d WS=GPIO%d DIN=GPIO%d @ %d Hz)",
             I2S_MCLK_GPIO, I2S_BCLK_GPIO, I2S_LRCK_GPIO, I2S_DIN_GPIO,
             SPEECH_SAMPLE_RATE);
    return ESP_OK;
}

// ─── AFE init (WakeNet9 + VAD) — ESP-SR v2.3.1 API ───────────────────────────

static esp_err_t afe_init(void)
{
    /* 1. Load models from the "model" flash partition (packed by build system). */
    s_models = esp_srmodel_init("model");
    if (!s_models) {
        ESP_LOGE(TAG, "esp_srmodel_init failed — is 'model' partition flashed?");
        return ESP_FAIL;
    }

    /* 2. Find the WakeNet model by name. */
    char *wn_name = esp_srmodel_filter(s_models, ESP_WN_PREFIX, WAKENET_MODEL_NAME);
    if (!wn_name) {
        ESP_LOGE(TAG, "WakeNet model '%s' not found — is CONFIG_SR_WN_WN9_HIESP=y?",
                 WAKENET_MODEL_NAME);
        esp_srmodel_deinit(s_models);
        s_models = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WakeNet model: %s", wn_name);

    /* 3. Build AFE config.
     *    "M" = mono microphone, no playback reference (no AEC needed).
     *    AFE_TYPE_SR = speech recognition (includes WakeNet + VAD).
     *    AFE_MODE_HIGH_PERF: WakeNet never detected in LOW_COST even at -7 dBFS input;
     *    switching to HIGH_PERF for better internal processing / feature extraction. */
    s_afe_cfg = afe_config_init("M", s_models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (!s_afe_cfg) {
        ESP_LOGE(TAG, "afe_config_init failed");
        esp_srmodel_deinit(s_models);
        s_models = NULL;
        return ESP_FAIL;
    }

    /* 4. Configure WakeNet. */
    s_afe_cfg->wakenet_init       = true;
    s_afe_cfg->wakenet_model_name = wn_name;
    /* DET_MODE_95: "probability of being wake words is increased with increasing mode,
     * as a consequence also the false alarm rate goes up" — i.e. more sensitive.
     * DET_MODE_90 (default) didn't detect even at -7 dBFS; trying DET_MODE_95. */
    s_afe_cfg->wakenet_mode       = DET_MODE_95;

    /* 5. Configure VAD.
     *    vad_model_name = NULL → WebRTC VAD (no model data needed).
     *    vad_min_noise_ms: AFE reports VAD_SILENCE only after this many ms of silence.
     *    This replaces our manual silence timer.
     *    VAD_MODE_0 = Normal (most permissive). Pipeline is VAD→WakeNet, so aggressive
     *    modes (VAD_MODE_3 = "Very Very Aggressive") suppress speech before WakeNet
     *    sees it, preventing wake word detection. */
    s_afe_cfg->vad_init          = true;
    s_afe_cfg->vad_mode          = VAD_MODE_0;
    s_afe_cfg->vad_model_name    = NULL;           /* WebRTC VAD */
    s_afe_cfg->vad_min_noise_ms  = SPEECH_SILENCE_MS;

    /* 6. Prefer PSRAM for AFE buffers. */
    s_afe_cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;

    /* 7. Select AFE implementation based on config. */
    s_afe_handle = esp_afe_handle_from_config(s_afe_cfg);
    if (!s_afe_handle) {
        ESP_LOGE(TAG, "esp_afe_handle_from_config failed");
        afe_config_free(s_afe_cfg);
        s_afe_cfg = NULL;
        esp_srmodel_deinit(s_models);
        s_models = NULL;
        return ESP_FAIL;
    }

    /* 8. Create AFE data instance (allocates DSP buffers, starts internal tasks). */
    s_afe_data = s_afe_handle->create_from_config(s_afe_cfg);
    if (!s_afe_data) {
        ESP_LOGE(TAG, "AFE create_from_config failed");
        afe_config_free(s_afe_cfg);
        s_afe_cfg = NULL;
        esp_srmodel_deinit(s_models);
        s_models = NULL;
        return ESP_FAIL;
    }

    /* 9. Lower WakeNet threshold for better sensitivity.
     *    Default thresholds are ~0.63/0.635 (high confidence required).
     *    Lowering to 0.5 improves detection when mic signal is -28 to -34 dBFS.
     *    Range: 0.4 (most sensitive) to 0.9999 (least sensitive). Index starts at 1. */
    int thr_ret = s_afe_handle->set_wakenet_threshold(s_afe_data, 1, 0.4f);
    ESP_LOGI(TAG, "set_wakenet_threshold(idx=1, 0.4) returned %d (1=ok -1=fail)", thr_ret);

    ESP_LOGI(TAG, "AFE ready: WakeNet=%s det_mode=DET_MODE_95 threshold=0.4 VAD=WebRTC mode%d silence=%d ms",
             wn_name, s_afe_cfg->vad_mode, SPEECH_SILENCE_MS);
    return ESP_OK;
}

// ─── TCP audio send ───────────────────────────────────────────────────────────

static void send_audio_to_host(const int16_t *samples, uint32_t count)
{
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(SPEECH_PORT),
    };
    inet_pton(AF_INET, SPEECH_HOST, &addr.sin_addr);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) { ESP_LOGE(TAG, "Socket create failed"); return; }

    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Connect to %s:%d failed", SPEECH_HOST, SPEECH_PORT);
        close(sock);
        return;
    }

    speech_pcm_header_t hdr = {
        .magic       = 0x41554449,
        .sample_rate = SPEECH_SAMPLE_RATE,
        .channels    = 1,
        .bits        = 16,
        .num_samples = count,
    };
    send(sock, &hdr, sizeof(hdr), 0);

    const uint8_t *p   = (const uint8_t *)samples;
    uint32_t       rem = count * sizeof(int16_t);
    while (rem > 0) {
        int n = send(sock, p, rem, 0);
        if (n <= 0) { ESP_LOGE(TAG, "TCP send error"); break; }
        p   += n;
        rem -= n;
    }
    close(sock);

    ESP_LOGI(TAG, "Sent %lu samples (%.1f s) to %s:%d",
             (unsigned long)count, (float)count / SPEECH_SAMPLE_RATE,
             SPEECH_HOST, SPEECH_PORT);
}

// ─── Speech task ──────────────────────────────────────────────────────────────

typedef enum { STATE_LISTENING, STATE_RECORDING, STATE_SENDING } speech_state_t;

/* 1 = log raw mic samples, per-second mic peaks and AFE status (VAD, volume,
 * wake word state). Off by default; wake/recording events are always logged. */
#ifndef SPEECH_DEBUG
#define SPEECH_DEBUG 0
#endif

static void speech_task(void *arg)
{
    int feed_chunksize  = s_afe_handle->get_feed_chunksize(s_afe_data);
    int stereo_bytes    = feed_chunksize * 2 * sizeof(int16_t); /* stereo: L+R interleaved */

    int16_t *stereo_buf = malloc(stereo_bytes);
    int16_t *feed_buf   = malloc(feed_chunksize * sizeof(int16_t)); /* mono CH1 for AFE */
    if (!stereo_buf || !feed_buf) {
        ESP_LOGE(TAG, "Failed to allocate I2S buffers");
        free(stereo_buf); free(feed_buf);
        vTaskDelete(NULL);
        return;
    }

    speech_state_t state       = STATE_LISTENING;
    uint32_t       rec_samples = 0;

    /* UDP monitoring socket — streams raw mono CH1 (pre-gain) to host for live listening */
    int udp_mon_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in mon_addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(SPEECH_MONITOR_PORT),
    };
    inet_pton(AF_INET, SPEECH_HOST, &mon_addr.sin_addr);
    ESP_LOGI(TAG, "Audio monitor on %s:%d — on host:", SPEECH_HOST, SPEECH_MONITOR_PORT);
    ESP_LOGI(TAG, "  ffplay -f s16le -ar 16000 -ch_layout mono -nodisp udp://0.0.0.0:%d", SPEECH_MONITOR_PORT);
    ESP_LOGI(TAG, "  ffmpeg -f s16le -ar 16000 -ch_layout mono -i udp://0.0.0.0:%d mic.wav", SPEECH_MONITOR_PORT);

    ESP_LOGI(TAG, "Speech task started (feed_chunksize=%d samples, stereo I2S)", feed_chunksize);
    ESP_LOGI(TAG, "Say \"Hi ESP\" to activate");

    uint32_t dbg_chunk = 0;
    while (!s_task_exit) {
        /* ── Read stereo chunk from I2S (CH1=left, CH2=right interleaved) ── */
        size_t bytes_read = 0;
        esp_err_t r = i2s_channel_read(s_i2s_rx, stereo_buf, stereo_bytes,
                                       &bytes_read, pdMS_TO_TICKS(200));
        if (r != ESP_OK || bytes_read < (size_t)stereo_bytes) continue;

        /* De-interleave: extract CH1 (even = left) into feed_buf for AFE */
        for (int i = 0; i < feed_chunksize; i++) {
            feed_buf[i] = stereo_buf[i * 2];
        }

        /* ── Debug: dump first interleaved samples (CH1=even, CH2=odd) ── */
        if (SPEECH_DEBUG && dbg_chunk == 0) {
            ESP_LOGI(TAG, "Stereo raw [L R L R]: %d %d %d %d  (hex: %04x %04x %04x %04x)",
                     stereo_buf[0], stereo_buf[1], stereo_buf[2], stereo_buf[3],
                     (uint16_t)stereo_buf[0], (uint16_t)stereo_buf[1],
                     (uint16_t)stereo_buf[2], (uint16_t)stereo_buf[3]);
        }

        /* ── Debug: log CH1 and CH2 peaks separately every ~1 s ── */
        bool print_sec = (++dbg_chunk % (SPEECH_SAMPLE_RATE / feed_chunksize) == 0);
        bool dbg_sec   = SPEECH_DEBUG && print_sec;
        if (dbg_sec) {
            int16_t peak_ch1 = 0, peak_ch2 = 0;
            for (int i = 0; i < feed_chunksize; i++) {
                int16_t s1 = stereo_buf[i*2]   < 0 ? -stereo_buf[i*2]   : stereo_buf[i*2];
                int16_t s2 = stereo_buf[i*2+1] < 0 ? -stereo_buf[i*2+1] : stereo_buf[i*2+1];
                if (s1 > peak_ch1) peak_ch1 = s1;
                if (s2 > peak_ch2) peak_ch2 = s2;
            }
            ESP_LOGI(TAG, "MIC CH1(left)=%d  CH2(right)=%d", peak_ch1, peak_ch2);
        }

        /* ── Monitor: stream raw mono CH1 to host via UDP (pre-gain, natural level) ── */
        if (udp_mon_sock >= 0) {
            sendto(udp_mon_sock, feed_buf, feed_chunksize * sizeof(int16_t), 0,
                   (struct sockaddr *)&mon_addr, sizeof(mon_addr));
        }

        /* ── Software gain: boost feed_buf before AFE to lift signal into WakeNet range ── */
#if SPEECH_SW_GAIN != 1
        for (int i = 0; i < feed_chunksize; i++) {
            int32_t v = (int32_t)feed_buf[i] * SPEECH_SW_GAIN;
            if (v > 32767)       v = 32767;
            else if (v < -32768) v = -32768;
            feed_buf[i] = (int16_t)v;
        }
#endif

        /* ── Feed CH1 mono to AFE ring buffer ── */
        s_afe_handle->feed(s_afe_data, feed_buf);

        /* ── Fetch processed result (blocks until AFE produces one frame) ── */
        afe_fetch_result_t *res = s_afe_handle->fetch(s_afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            if (print_sec)
                ESP_LOGW(TAG, "AFE fetch failed (res=%p ret=%d)", res, res ? res->ret_value : -1);
            continue;
        }
        /* Log AFE-reported volume and ring buffer status every ~1 s */
        if (dbg_sec) {
            ESP_LOGI(TAG, "AFE vol=%.1f dB  rbuf_free=%.2f  vad=%d  wakeup=%d  ww_idx=%d",
                     res->data_volume, res->ringbuff_free_pct,
                     res->vad_state, res->wakeup_state, res->wake_word_index);
        }

        switch (state) {

        case STATE_LISTENING:
            if (res->wakeup_state == WAKENET_DETECTED) {
                ESP_LOGI(TAG, "Wake word detected! Recording...");
                rec_samples = 0;
                state = STATE_RECORDING;
            }
            break;

        case STATE_RECORDING:
            /* Buffer AFE-processed audio (noise-suppressed output). */
            if (res->data && res->data_size > 0) {
                int samples = res->data_size / (int)sizeof(int16_t);
                if (rec_samples + (uint32_t)samples <= SPEECH_MAX_SAMPLES) {
                    memcpy(s_audio_buf + rec_samples, res->data, res->data_size);
                    rec_samples += samples;
                } else {
                    /* Hard cap reached — send now. */
                    ESP_LOGW(TAG, "Max recording length (%d s) reached", SPEECH_MAX_SECONDS);
                    state = STATE_SENDING;
                    break;
                }
            }

            /* AFE reports VAD_SILENCE only after vad_min_noise_ms (=SPEECH_SILENCE_MS) ms. */
            if (res->vad_state == VAD_SILENCE) {
                ESP_LOGI(TAG, "Speech ended after %lu ms silence; %.1f s recorded",
                         (unsigned long)SPEECH_SILENCE_MS,
                         (float)rec_samples / SPEECH_SAMPLE_RATE);
                state = STATE_SENDING;
            }
            break;

        case STATE_SENDING:
            if (rec_samples > 0) {
                send_audio_to_host(s_audio_buf, rec_samples);
            }
            rec_samples = 0;
            state = STATE_LISTENING;
            ESP_LOGI(TAG, "Ready — say wake word again");
            break;
        }
    }

    free(stereo_buf);
    free(feed_buf);
    if (udp_mon_sock >= 0) close(udp_mon_sock);
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

// ─── Public API ───────────────────────────────────────────────────────────────

esp_err_t speech_init(void)
{
    s_audio_buf = heap_caps_malloc(SPEECH_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_audio_buf) s_audio_buf = malloc(SPEECH_MAX_BYTES);
    if (!s_audio_buf) {
        ESP_LOGE(TAG, "Failed to allocate %u byte audio buffer", SPEECH_MAX_BYTES);
        return ESP_FAIL;
    }

    esp_err_t r;
    if ((r = i2c_init())          != ESP_OK) return r;
    if ((r = es8311_init_codec()) != ESP_OK) return r;
    /* Start I2S (and therefore MCLK on GPIO13) BEFORE configuring ES7210.
     * ES7210's DLL (REG02 bit7) requires MCLK to be present to lock.
     * If MCLK is absent when the DLL config is written, the ADC clock
     * never starts and the output stays at zero. */
    if ((r = i2s_init())          != ESP_OK) return r;
    if ((r = es7210_init_codec()) != ESP_OK) return r;
    if ((r = afe_init())          != ESP_OK) return r;

    ESP_LOGI(TAG, "Speech module ready. Buffer: %u bytes (~%d s)",
             SPEECH_MAX_BYTES, SPEECH_MAX_SECONDS);
    return ESP_OK;
}

void speech_start(void)
{
    if (s_task_handle) return;
    s_task_exit = false;
    xTaskCreatePinnedToCore(speech_task, "speech", 8192, NULL, 5,
                            &s_task_handle, 0);
    ESP_LOGI(TAG, "Speech task started (core 0)");
}

void speech_stop(void)
{
    s_task_exit = true;
    for (int i = 0; s_task_handle && i < 200; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_afe_data && s_afe_handle) {
        s_afe_handle->destroy(s_afe_data);
        s_afe_data = NULL;
    }
    if (s_afe_cfg) {
        afe_config_free(s_afe_cfg);
        s_afe_cfg = NULL;
    }
    if (s_models) {
        esp_srmodel_deinit(s_models);
        s_models = NULL;
    }
    if (s_i2s_rx) {
        i2s_channel_disable(s_i2s_rx);
        i2s_del_channel(s_i2s_rx);
        s_i2s_rx = NULL;
    }
    if (s_es7210_dev) {
        i2c_master_bus_rm_device(s_es7210_dev);
        s_es7210_dev = NULL;
    }
    if (s_es8311_dev) {
        i2c_master_bus_rm_device(s_es8311_dev);
        s_es8311_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    free(s_audio_buf);
    s_audio_buf = NULL;

    ESP_LOGI(TAG, "Speech module stopped");
}
