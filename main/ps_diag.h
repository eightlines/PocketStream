/**
 * @file ps_diag.h
 * @brief PocketStream diagnostics: verbose-log switch and per-stage timing.
 *
 * PS_DEBUG  (default 0) — 1 re-enables the per-frame / per-NAL logs and the
 *                         frame checksums. They cost several ms per line at
 *                         115200 baud, so leave this off when judging speed.
 * PS_PERF   (default 1) — 1 prints one timing summary line per second.
 *
 * Override either from main/CMakeLists.txt, e.g.
 *   target_compile_definitions(${COMPONENT_LIB} PRIVATE PS_DEBUG=1)
 *
 * All timing is accumulated on the rtp_rx task (network read, depacketize,
 * decode and render all run there today), so no locking is needed.
 * The state lives in main.c.
 */
#pragma once

#include <stdint.h>
#include "esp_log.h"
#include "esp_timer.h"

#ifndef PS_DEBUG
#define PS_DEBUG 0
#endif

#ifndef PS_PERF
#define PS_PERF 1
#endif

// Verbose log: compiled out entirely unless PS_DEBUG=1.
#define PS_LOGV(tag, fmt, ...) \
    do { if (PS_DEBUG) ESP_LOGI(tag, fmt, ##__VA_ARGS__); } while (0)

typedef enum {
    PS_STAGE_NET = 0,   // blocked in recv() waiting for RTP bytes
    PS_STAGE_DECODE,    // inside esp_h264_dec_process()
    PS_STAGE_CONVERT,   // I420 -> RGB565
    PS_STAGE_SCALE,     // RGB565 scale to 720x720
    PS_STAGE_DISPLAY,   // draw_bitmap + wait for the DMA copy
    PS_STAGE_COUNT
} ps_stage_t;

typedef struct {
    uint32_t stage_us[PS_STAGE_COUNT];  // accumulated this window
    uint32_t decode_max_us;             // worst single decode call this window
    uint32_t frames;                    // frames shown this window
    uint32_t bytes;                     // RTP bytes received this window
    uint32_t rtp_gaps;                  // sequence gaps this window
    uint32_t rtp_lost;                  // packets lost this window
    int64_t  window_start_us;
} ps_perf_t;

extern ps_perf_t g_ps_perf;

static inline int64_t ps_now_us(void) { return esp_timer_get_time(); }

static inline void ps_perf_add(ps_stage_t stage, int64_t start_us)
{
#if PS_PERF
    uint32_t dt = (uint32_t)(esp_timer_get_time() - start_us);
    g_ps_perf.stage_us[stage] += dt;
    if (stage == PS_STAGE_DECODE && dt > g_ps_perf.decode_max_us)
        g_ps_perf.decode_max_us = dt;
#else
    (void)stage; (void)start_us;
#endif
}

static inline void ps_perf_bytes(uint32_t n)        { if (PS_PERF) g_ps_perf.bytes += n; }
static inline void ps_perf_gap(uint32_t lost)       { if (PS_PERF) { g_ps_perf.rtp_gaps++; g_ps_perf.rtp_lost += lost; } }

// Call once per displayed frame; prints and resets the window every second.
void ps_perf_frame_done(void);
