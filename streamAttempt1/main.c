#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_h264_dec_sw.h"
#include "lwip/sockets.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_st7703.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "driver/ppa.h"

// Defined by Waveshare spec for 4-inch display
#define LCD_H_RES 720
#define LCD_V_RES 720
#define LCD_BIT_PER_PIXEL 18 // 18-bit RGB666 (Waveshare demo uses this!)
#define PIN_NUM_LCD_RST 27
#define PIN_NUM_BK_LIGHT 26

#define VIDEO_WIDTH 160
#define VIDEO_HEIGHT 120
#define LCD_BK_LIGHT_ON_LEVEL 0 // Active Low
#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

#if LCD_BIT_PER_PIXEL == 24
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB888)
#elif LCD_BIT_PER_PIXEL == 18
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB666)
#elif LCD_BIT_PER_PIXEL == 16
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB565)
#endif

#define ST7703_720_720_PANEL_60HZ_DPI_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 30,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 720,                               \
            .v_size = 720,                               \
            .hsync_back_porch = 40,                      \
            .hsync_pulse_width = 4,                      \
            .hsync_front_porch = 40,                     \
            .vsync_back_porch = 12,                      \
            .vsync_pulse_width = 4,                      \
            .vsync_front_porch = 16,                     \
        },                                               \
        .flags.use_dma2d = false,                        \
    }

#define PORT 3333

// Ethernet Config
#define ETH_PHY_ADDR        1
#define ETH_PHY_RST_GPIO    51
#define ETH_MDC_GPIO        31
#define ETH_MDIO_GPIO       52
#define ETH_REF_CLK_GPIO    50

static const char *TAG = "P4_VIDEO";
static EventGroupHandle_t s_eth_event_group;
static const int GOT_IP_BIT = BIT0;
static SemaphoreHandle_t refresh_finish = NULL;

// 1. Decoder Setup
esp_h264_dec_handle_t dec_handle = NULL;

void init_decoder() {
    esp_h264_dec_cfg_sw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_I420,
    };
    esp_h264_dec_sw_new(&cfg, &dec_handle);
    esp_h264_dec_open(dec_handle);
}

/*
void yuv420_to_rgb565(uint8_t *yuv, uint16_t *rgb, int width, int height, int stride_pixels) {
    // ... removed sw implementation
}
*/


#define CLAMP(x)        ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))
#define SWAP_ENDIAN(x)  ((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))

// Helper to convert YUV to RGB888 (for RGB666 mode)
static inline void yuv_to_rgb888(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    int c = y - 16;
    int d = u - 128;  // Cb
    int e = v - 128;  // Cr
    
    // Standard BT.601 coefficients
    int R = (298 * c + 409 * e + 128) >> 8;
    int G = (298 * c - 100 * d - 208 * e + 128) >> 8;
    int B = (298 * c + 516 * d + 128) >> 8;
    
    *r = CLAMP(R);
    *g = CLAMP(G);
    *b = CLAMP(B);
}

// 2. Network Task
void udp_video_task(void *pvParameters) {
    // Delay to ensure main logs are flushed and system is stable
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "UDP Task Starting...");

    // Print Current IP Address for Debugging
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("ETH_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(netif, &ip_info);
        ESP_LOGI(TAG, "Current IP: " IPSTR, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGW(TAG, "Could not get Ethernet Netif Handle");
    }

    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)pvParameters;
    
    // Setup UDP Socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    // Set Receive Timeout (2 seconds)
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ESP_LOGE(TAG, "Failed to set SO_RCVTIMEO: errno %d", errno);
    }

    int rcvbuf = 128 * 1024;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        ESP_LOGE(TAG, "Failed to set SO_RCVBUF: errno %d", errno);
    }
    
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Socket bound to port %d", PORT);

    // Allocate RGB buffer for video output (RGB666 uses 3 bytes per pixel stored as RGB888)
    // We'll draw to this buffer then use draw_bitmap to send to display
    uint8_t *rgb_buffer = heap_caps_aligned_alloc(64, LCD_H_RES * LCD_V_RES * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!rgb_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "UDP Task RGB buffer at %p (size=%d bytes)", rgb_buffer, LCD_H_RES * LCD_V_RES * 3);

    // Allocate Output Frame struct
    esp_h264_dec_out_frame_t out_frame = {0};
    esp_h264_dec_in_frame_t in_frame = {0};

    int frame_count = 0;
    int write_pos = 0;
    int read_pos = 0;

    // --- BLUE STARTUP (REMOVED to show Bars) ---
    // for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) rgb_buffer[i] = 0x001F; // Blue
    // esp_cache_msync(rgb_buffer, LCD_H_RES * LCD_V_RES * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    // ----------------------------------------
    
    // PPA Hardware often requires output stride to be cache-line aligned (64 bytes in PSRAM)
    // 720 pixels * 2 bytes = 1440 bytes. 1440 % 64 = 32. Not aligned.
    // We align the stride to 1472 bytes (736 pixels).
    // Buffer size = 736 * 720 * 2
    int ppa_out_w = ALIGN_UP(LCD_H_RES * 2, 64) / 2;
    int ppa_out_h = LCD_V_RES;
    size_t ppa_buffer_size = ppa_out_w * ppa_out_h * 2;
    uint16_t *ppa_out_buffer = heap_caps_aligned_alloc(64, ppa_buffer_size, MALLOC_CAP_SPIRAM);
    
    // Allocate a separate PPA Input Buffer in PSRAM (Safe Buffer)
    // Use larger buffer for flexibility (720x720 YUV420 = 777600 bytes)
    size_t ppa_in_size = 720 * 720 * 3 / 2;
    uint8_t *ppa_in_buffer = heap_caps_aligned_alloc(64, ppa_in_size, MALLOC_CAP_SPIRAM);

    if (!ppa_out_buffer || !ppa_in_buffer) {
        ESP_LOGE(TAG, "Failed to allocate PPA buffers");
        return;
    }
    ESP_LOGI(TAG, "PPA Buffers: IN@%p (size=%d), OUT@%p (size=%d)", 
             ppa_in_buffer, ppa_in_size, ppa_out_buffer, ppa_buffer_size);
    ESP_LOGI(TAG, "RGB buffer at: %p (size=%d)", rgb_buffer, LCD_H_RES * LCD_V_RES * 2);

    // --- PPA Initialization ---
    ppa_client_handle_t ppa_client = NULL;
    ppa_client_config_t ppa_client_cfg = {
        .oper_type = PPA_OPERATION_SRM,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_client_cfg, &ppa_client));
    ESP_LOGI(TAG, "PPA Client Registered");
    // --------------------------

    ESP_LOGI(TAG, "UDP Task Started, listening on port %d", PORT);

    //int heartbeat_counter = 0;
    int udp_packet_counter = 0;
    int nal_loop_counter = 0;
    
    // Decoding state flags (moved outside loop to persist, but reset on timeout)
    bool has_sps = false;
    bool has_pps = false;
    
    // Buffer to accumulate SPS+PPS+IDR for combined decoding
    // TinyH264 may need all headers + IDR in one buffer
    uint8_t *combined_buffer = heap_caps_aligned_alloc(64, 64 * 1024, MALLOC_CAP_SPIRAM);
    int combined_len = 0;
    uint8_t *sps_data = NULL;
    int sps_len = 0;
    uint8_t *pps_data = NULL; 
    int pps_len = 0;
    // Allocate Input Buffer for H264 data (in PSRAM)
    const int BUFFER_SIZE = 1200 * 1024; // Increase to 1.2MB 
    // CRITICAL FIX: Add 64 bytes of padding at the end of the buffer
    // The decoder (TinyH264) often reads a few bytes past the end of the NAL unit during emulation prevention byte checks.
    // If the NAL unit ends exactly at the end of a memory block (or worse, the ring buffer wraps), it can crash or error.
    uint8_t *rx_buffer = heap_caps_aligned_alloc(64, BUFFER_SIZE + 64, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    memset(rx_buffer, 0, BUFFER_SIZE + 64); // Zero out for safety
    
    while (1) {
        // Compact buffer if needed (if running out of space or read pointer is far ahead)
        if (read_pos > 0 && (BUFFER_SIZE - write_pos < 4096 || read_pos > BUFFER_SIZE / 2)) {
            int remaining = write_pos - read_pos;
            if (remaining > 0) {
                memmove(rx_buffer, rx_buffer + read_pos, remaining);
            }
            write_pos = remaining;
            read_pos = 0;
        }

        // Receive Raw H.264 data from FFmpeg
        int len = recv(sock, rx_buffer + write_pos, BUFFER_SIZE - write_pos, 0);
        if (len > 0) {
            udp_packet_counter++;
            if (udp_packet_counter % 100 == 0) {
                ESP_LOGI(TAG, "UDP: Received %d packets (last len=%d)", udp_packet_counter, len);
            }
        }
        
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // If we time out (no data for 2 seconds), reset the stream state
                // This handles the "drop connection and reconnect" scenario
                if (frame_count > 0 || has_sps || has_pps) {
                    ESP_LOGW(TAG, "Stream Timeout - Resetting Decoder State...");
                    has_sps = false;
                    has_pps = false;
                    frame_count = 0;
                    
                    // Optional: Flush decoder (close/open) to ensure clean state
                    // esp_h264_dec_close(dec_handle);
                    // esp_h264_dec_open(dec_handle);
                }
                
                ESP_LOGI(TAG, "Waiting for UDP packets... (read_pos=%d, write_pos=%d)", read_pos, write_pos);
                continue;
            }
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Log the first 16 bytes of every UDP packet received
        if (len > 0) {
            ESP_LOG_BUFFER_HEX("UDP_PKT", rx_buffer + write_pos, len > 16 ? 16 : len);
        }

        if (len > 0) {
            static bool first_packet = true;
            if (first_packet) {
                ESP_LOGI(TAG, "First UDP packet received! (%d bytes)", len);
                first_packet = false;
            }
            write_pos += len;

            // Loop to process all complete NAL units in the buffer
            while (write_pos - read_pos > 4) {
                
                // If the buffer is more than 50% full, force a shift instantly to keep memory contiguous
                if (write_pos > BUFFER_SIZE * 0.5 && read_pos > 0) {
                   // Ensure we don't overlapping copy if not necessary, though memmove handles it.
                   int data_len = write_pos - read_pos;
                   memmove(rx_buffer, rx_buffer + read_pos, data_len);
                   write_pos = data_len;
                   read_pos = 0;
                }

                int available_data = write_pos - read_pos;
                uint8_t *p = rx_buffer + read_pos;
                
                nal_loop_counter++;
                
                // --- Robust NAL reassembly: handle NAL units split across UDP packets ---
                // 1. Ensure we are at a start code
                int sc_len = 0;
                if (p[0] == 0 && p[1] == 0 && p[2] == 1) sc_len = 3;
                else if (p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1) sc_len = 4;
                if (sc_len == 0) {
                    // No start code at current read_pos, scan for next start code
                    int offset = -1;
                    for (int i = 0; i < available_data - 3; i++) {
                        if (p[i] == 0 && p[i+1] == 0 && p[i+2] == 1) { offset = i; break; }
                        if (p[i] == 0 && p[i+1] == 0 && p[i+2] == 0 && p[i+3] == 1) { offset = i; break; }
                    }
                    if (offset != -1) {
                        read_pos += offset;
                        continue; // Found a start code, re-enter loop
                    } else {
                        // Not enough data for a start code, keep last 3 bytes
                        // Improve efficiency: if buffer is FULL and no start code, drain it
                        if (available_data > BUFFER_SIZE / 2 && offset == -1) {
                            ESP_LOGW(TAG, "Draining garbage from buffer (%d bytes)", available_data);
                            read_pos = write_pos; // Dump all
                        } else if (available_data > 3) {
                            read_pos += (available_data - 3);
                        }
                        break; // Wait for more data
                    }
                }
                // 2. Find END of this NAL (Start of NEXT NAL)
                int nal_len = -1;
                // Optimization: Start searching after the current start code + header
                // NAL must have at least 1 byte payload, so checking from sc_len + 1 is safe
                int search_start = sc_len + 1;
                // Limit search for next start code to avoid tiny NALs or false positives? No, just scan.
                // NOTE: sc_len is 3 or 4.
                for (int i = search_start; i < available_data - 3; i++) {
                    if ((p[i] == 0 && p[i+1] == 0 && p[i+2] == 1) ||
                        (p[i] == 0 && p[i+1] == 0 && p[i+2] == 0 && p[i+3] == 1)) {
                        nal_len = i;
                        break;
                    }
                }
                
                // CRITICAL: If no next start code is found, but the buffer is huge, we might have a full frame.
                // However, without the next start code, we don't know if the frame is complete.
                // BUT, if we are at the end of a received UDP packet, maybe we can assume end of NAL?
                // H264 over UDP (raw) usually puts one NAL per packet (if small), or fragments it.
                // If it fragments, we just wait.
                
                if (nal_len == -1) {
                    // No end marker found: NAL is possibly incomplete.
                    // However, if the buffer is nearly full, we risk deadlocking if we never find the next start code
                    // (e.g. if the next packet is dropped).
                    // Robustness: If available_data is huge (e.g. > 100KB), assume the frame ended and we missed the start code of the next one.
                    if (available_data > 100 * 1024) {
                        ESP_LOGW(TAG, "NAL too large (%d) or missing next Start Code. Forcing decode.", available_data);
                        nal_len = available_data; 
                    } else {
                        break; // Wait for more data
                    }
                }

                // 3. Identify NAL Type
                uint8_t nal_type = p[sc_len] & 0x1F;
                
                // Store SPS/PPS and feed to decoder separately to initialize
                if (nal_type == 7) { 
                    // SPS - feed to decoder to initialize
                    if (!has_sps) {
                        in_frame.raw_data.buffer = p;
                        in_frame.raw_data.len = nal_len;
                        in_frame.consume = 0;
                        int ret = esp_h264_dec_process(dec_handle, &in_frame, &out_frame);
                        if (ret == ESP_H264_ERR_OK) {
                            has_sps = true;
                            ESP_LOGI(TAG, "SPS accepted (len=%d)", nal_len);
                        } else {
                            ESP_LOGW(TAG, "SPS rejected (len=%d, ret=%d)", nal_len, ret);
                        }
                    }
                    read_pos += nal_len;
                    continue;
                }
                if (nal_type == 8) { 
                    // PPS - feed to decoder to initialize
                    if (!has_pps && has_sps) {
                        in_frame.raw_data.buffer = p;
                        in_frame.raw_data.len = nal_len;
                        in_frame.consume = 0;
                        int ret = esp_h264_dec_process(dec_handle, &in_frame, &out_frame);
                        if (ret == ESP_H264_ERR_OK) {
                            has_pps = true;
                            ESP_LOGI(TAG, "PPS accepted (len=%d)", nal_len);
                        } else {
                            ESP_LOGW(TAG, "PPS rejected (len=%d, ret=%d)", nal_len, ret);
                        }
                    }
                    read_pos += nal_len;
                    continue;
                }

                // For IDR (type 5) or slice (type 1), decode directly
                bool should_skip = false;

                if (!has_sps || !has_pps) {
                    should_skip = true;
                }

                if (should_skip) {
                    static int suppress = 0;
                    if (suppress++ % 30 == 0) {
                        ESP_LOGW(TAG, "Skipping NAL %d - Waiting for Sync (SPS=%d, PPS=%d)", nal_type, has_sps, has_pps);
                    }
                    read_pos += nal_len;
                    continue;
                }

                if (frame_count < 5) {
                    ESP_LOGI(TAG, "Decoding NAL type=%d, len=%d", nal_type, nal_len);
                }

                // 4. Decode the frame directly (SPS/PPS already fed to decoder)
                in_frame.raw_data.buffer = p;
                in_frame.raw_data.len = nal_len;
                in_frame.consume = 0;
                
                int ret = esp_h264_dec_process(dec_handle, &in_frame, &out_frame);
                if (ret != ESP_H264_ERR_OK) {
                    // Only log occasionally to reduce spam
                    static int err_count = 0;
                    if (err_count++ % 10 == 0) {
                        ESP_LOGW(TAG, "Decoder error: ret=%d, NAL type=%d, len=%d (error #%d)", ret, nal_type, nal_len, err_count);
                    }
                    read_pos += nal_len; // Skip corrupted NAL
                    // DON'T reset SPS/PPS state - allow P-frames to continue attempting decode
                } else {
                    // Update read_pos based on consumption
                    if (in_frame.consume > 0) {
                        read_pos += in_frame.consume;
                    } else {
                        // If decoder reports 0 consume (or bug), advance manually by NAL length
                        read_pos += nal_len; 
                    }
                        
                    // Decoder pads to 16-pixel macroblock boundaries
                    // 160x120 -> 160x128 = 20480 pixels, YUV420 = 30720 bytes
                    size_t min_frame_size = (VIDEO_WIDTH * ((VIDEO_HEIGHT + 15) & ~15)) * 3 / 2;
                    if (out_frame.outbuf && out_frame.out_size > 0 && out_frame.out_size >= min_frame_size) {


                            if (frame_count < 5 || frame_count % 30 == 0) {
                                ESP_LOGI(TAG, "Decoded Frame %d: Size %lu", frame_count + 1, (unsigned long)out_frame.out_size);
                            }
                            frame_count++;


                            // Get Resolution dynamically
                            esp_h264_dec_param_sw_handle_t param_handle = NULL;
                            esp_h264_dec_sw_get_param_hd(dec_handle, &param_handle);
                            
                            esp_h264_resolution_t res = {0};
                            esp_h264_dec_get_resolution(param_handle, &res);
                            
                            int width = res.width;
                            int height = res.height;
                            
                            // Decoder pads height to macroblock boundary (multiple of 16)
                            if (width == 0 || height == 0) {
                                width = VIDEO_WIDTH;
                                height = (VIDEO_HEIGHT + 15) & ~15; // Round up to 16
                            }
                            
                            // Log actual decoded dimensions on first few frames
                            if (frame_count <= 3) {
                                ESP_LOGI(TAG, "Decoder resolution: %dx%d (expected %dx%d)", 
                                    res.width, res.height, VIDEO_WIDTH, VIDEO_HEIGHT);
                            }

                            // Log detailed frame info every 30 frames
                            if (frame_count % 30 == 0) {
                                ESP_LOGI(TAG, "Frame Info: W=%d H=%d Size=%lu", width, height, (unsigned long)out_frame.out_size);
                                ESP_LOG_BUFFER_HEX("Y-Plane Head", out_frame.outbuf, 16);
                                // Also dump UV plane starts
                                int dbg_y_size = width * height;
                                ESP_LOGI(TAG, "Y size=%d, checking offsets...", dbg_y_size);
                                ESP_LOG_BUFFER_HEX("At Y+YSize (U?)", out_frame.outbuf + dbg_y_size, 16);
                                ESP_LOG_BUFFER_HEX("At Y+YSize*5/4 (V?)", out_frame.outbuf + dbg_y_size * 5/4, 16);
                            }

                            // 4. Decode - read DIRECTLY from decoder output
                            // SOFTWARE FALLBACK: YUV420 to RGB565
                            // TinyH264 outputs I420 (planar YUV420P)
                            
                            // Decoder reports height as padded (128), we use that for plane offsets
                            int frame_w = width;    // 160
                            int frame_h = height;   // 128 (padded)
                            
                            // 1. Map Input Buffers - I420: Y, then U, then V
                            // Read directly from decoder output, not from copy
                            uint8_t *y_plane = out_frame.outbuf;
                            // U plane starts at height * width
                            uint8_t *u_plane = y_plane + (frame_h * frame_w);
                            // V plane starts at height * width * 5/4
                            uint8_t *v_plane = y_plane + (frame_h * frame_w * 5 / 4);
                            
                            // 2. Clear LCD Buffer (Blue for Debug) - DISABLED to preserve bars/prev frame
                            /*
                            for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                                rgb_buffer[i] = 0x001F; // Blue
                            }
                            */

                            // 3. Convert Loop
                            // We only convert the visible area (min of video and screen)
                            // Use VIDEO_HEIGHT (120) not padded height (128) for rendering
                            int render_w = (VIDEO_WIDTH < LCD_H_RES) ? VIDEO_WIDTH : LCD_H_RES;
                            int render_h = (VIDEO_HEIGHT < LCD_V_RES) ? VIDEO_HEIGHT : LCD_V_RES;
                            
                            // Center the image
                            int offset_x = (LCD_H_RES - render_w) / 2;
                            int offset_y = (LCD_V_RES - render_h) / 2;

                            if (frame_count % 30 == 0) {
                                ESP_LOGI(TAG, "Frame Info: Res: %dx%d. Render: %dx%d @ (%d, %d)", 
                                    frame_w, frame_h, render_w, render_h, offset_x, offset_y);
                            }

                            // Debug: Log UV plane values at different positions
                            if (frame_count % 30 == 0) {
                                // Sample UV at different X positions (bars are vertical)
                                int uv_y = 30;  // middle of frame
                                ESP_LOGI(TAG, "UV @ y=%d: x=10:U=%d,V=%d x=30:U=%d,V=%d x=50:U=%d,V=%d x=70:U=%d,V=%d",
                                    uv_y,
                                    u_plane[uv_y * (frame_w/2) + 5], v_plane[uv_y * (frame_w/2) + 5],
                                    u_plane[uv_y * (frame_w/2) + 15], v_plane[uv_y * (frame_w/2) + 15],
                                    u_plane[uv_y * (frame_w/2) + 25], v_plane[uv_y * (frame_w/2) + 25],
                                    u_plane[uv_y * (frame_w/2) + 35], v_plane[uv_y * (frame_w/2) + 35]);
                            }

                            // Sample pixels at different positions to verify actual video data
                            // SMPTE bars: left=white, then yellow, cyan, green, magenta, red, blue, black
                            if (frame_count % 10 == 0) {
                                // Sample 4 horizontal positions: 10%, 30%, 50%, 70% 
                                int x_pos[4] = {frame_w/10, frame_w*3/10, frame_w*5/10, frame_w*7/10};
                                int y_pos = VIDEO_HEIGHT / 4; // Top quarter (where main bars are)
                                ESP_LOGI(TAG, "Y samples at y=%d: x=%d:%d x=%d:%d x=%d:%d x=%d:%d",
                                    y_pos,
                                    x_pos[0], y_plane[y_pos * frame_w + x_pos[0]],
                                    x_pos[1], y_plane[y_pos * frame_w + x_pos[1]],
                                    x_pos[2], y_plane[y_pos * frame_w + x_pos[2]],
                                    x_pos[3], y_plane[y_pos * frame_w + x_pos[3]]);
                            }

                            // Render video frame - RGB888 (for RGB666 mode)
                            for (int y = 0; y < render_h; y++) {
                                for (int x = 0; x < render_w; x++) {
                                    int Y_val = y_plane[y * frame_w + x];
                                    int uv_y = y >> 1;
                                    int uv_x = x >> 1;
                                    int U = u_plane[uv_y * (frame_w / 2) + uv_x];
                                    int V = v_plane[uv_y * (frame_w / 2) + uv_x];
                                    uint8_t r, g, b;
                                    yuv_to_rgb888(Y_val, U, V, &r, &g, &b);
                                    int pix_idx = ((offset_y + y) * LCD_H_RES + (offset_x + x)) * 3;
                                    rgb_buffer[pix_idx + 0] = r;
                                    rgb_buffer[pix_idx + 1] = g;
                                    rgb_buffer[pix_idx + 2] = b;
                                }
                            }
                            
                            // Add moving indicator to verify display updates (white)
                            int indicator_x = (frame_count * 5) % (LCD_H_RES - 20);
                            for (int dy = 0; dy < 10; dy++) {
                                for (int dx = 0; dx < 20; dx++) {
                                    int pix_idx = ((10 + dy) * LCD_H_RES + indicator_x + dx) * 3;
                                    rgb_buffer[pix_idx + 0] = 0xFF;  // R
                                    rgb_buffer[pix_idx + 1] = 0xFF;  // G
                                    rgb_buffer[pix_idx + 2] = 0xFF;  // B (White)
                                }
                            }
                            
                            // Send to display using draw_bitmap
                            esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, rgb_buffer);
                            
                            ESP_LOGI(TAG, "Frame %d rendered", frame_count);

                            /*
                            ppa_srm_oper_config_t srm_config = {
                                .in.buffer = ppa_in_buffer,
                                .in.pic_w = width,
                                .in.pic_h = height,
                                .in.block_w = width,
                                .in.block_h = height,
                                .in.srm_cm = PPA_SRM_COLOR_MODE_YUV420,
                                
                                .out.buffer = ppa_out_buffer,
                                .out.buffer_size = ppa_buffer_size, // Configured for aligned buffer
                                .out.pic_w = ppa_out_w,             // Stride-aligned width (736)
                                .out.pic_h = ppa_out_h,
                                .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
                                
                                .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                                .scale_x = scale_factor_x, 
                                .scale_y = scale_factor_y,
                                .mode = PPA_TRANS_MODE_BLOCKING,
                            };
                            
                            ESP_LOGI(TAG, "Starting PPA... (In: %dx%d, Out: %dx%d)", width, height, ppa_out_w, ppa_out_h);
                            esp_err_t ppa_ret = ppa_do_scale_rotate_mirror(ppa_client, &srm_config);
                            if (ppa_ret != ESP_OK) {
                                ESP_LOGE(TAG, "PPA Failed: %d", ppa_ret);
                            } else {
                                ESP_LOGI(TAG, "PPA Done. Copying to LCD...");
                                // 3. Copy Valid PPA Output to LCD Buffer (Strided Copy)
                                esp_cache_msync(ppa_out_buffer, ppa_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
                                
                                for (int y = 0; y < LCD_V_RES; y++) {
                                    // Copy 720 pixels (1440 bytes) from 736-pixel stride buffer
                                    uint16_t *src_line = ppa_out_buffer + (y * ppa_out_w);
                                    uint16_t *dst_line = rgb_buffer + (y * LCD_H_RES);
                                    memcpy(dst_line, src_line, LCD_H_RES * 2);
                                }
                                
                                // Flush LCD Buffer
                                esp_cache_msync(rgb_buffer, LCD_H_RES * LCD_V_RES * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                            }
                            */
                            
                            // 4. Yield explicitly to ensure watchdog didn't bite us
                            vTaskDelay(1);
                        }
                }
            }
        }
    }
}

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        xEventGroupClearBits(s_eth_event_group, GOT_IP_BIT);
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    xEventGroupSetBits(s_eth_event_group, GOT_IP_BIT);
}

static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;

IRAM_ATTR static bool notify_refresh_ready(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    BaseType_t need_yield = pdFALSE;
    xSemaphoreGiveFromISR(sem, &need_yield);
    return (need_yield == pdTRUE);
}

static void init_display(void)
{
    refresh_finish = xSemaphoreCreateBinary();
    
#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));
#endif

    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
#ifdef MIPI_DSI_PHY_PWR_LDO_CHAN
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
#endif

    ESP_LOGI(TAG, "Initialize MIPI DSI bus");
    esp_lcd_dsi_bus_config_t bus_config = ST7703_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_dbi_io_config_t dbi_config = ST7703_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));

    ESP_LOGI(TAG, "Install LCD driver of st7703");
    esp_lcd_dpi_panel_config_t dpi_config = ST7703_720_720_PANEL_60HZ_DPI_CONFIG(MIPI_DPI_PX_FORMAT);
    st7703_vendor_config_t vendor_config = {
        .flags = {
            .use_mipi_interface = 1,
        },
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // Waveshare demo uses RGB
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7703(mipi_dbi_io, &panel_config, &panel_handle));
    
    ESP_LOGI(TAG, "Panel created");
    
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_LOGI(TAG, "Panel reset");
    
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_LOGI(TAG, "Panel initialized");
    
    // Register refresh callback - CRITICAL for DPI panel to stay active
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = notify_refresh_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, refresh_finish));
    ESP_LOGI(TAG, "Refresh callback registered");
    
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "Panel display on");

    // Turn on Backlight
    ESP_LOGI(TAG, "Backlight ON");
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    // --- TEST HARDWARE PATTERN FIRST ---
    ESP_LOGI(TAG, "Testing Hardware Color Bar Pattern...");
    esp_lcd_dpi_panel_set_pattern(panel_handle, MIPI_DSI_PATTERN_BAR_VERTICAL);
    vTaskDelay(pdMS_TO_TICKS(3000));  // Show for 3 seconds
    esp_lcd_dpi_panel_set_pattern(panel_handle, MIPI_DSI_PATTERN_NONE);
    ESP_LOGI(TAG, "Hardware Pattern Test Complete");
    // ----------------------------------------
}

void init_ethernet() {
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create new default instance of esp-netif for Ethernet
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;
    
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp32_emac_config.clock_config.rmii.clock_gpio = ETH_REF_CLK_GPIO;
    
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    
    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Set Static IP to match stream_video.ps1
    // Updated to match Host Ethernet IP (10.0.0.1 / 255.0.0.0)
    ESP_LOGI(TAG, "Configuring Static IP: 10.0.0.2...");
    esp_netif_dhcpc_stop(eth_netif);
    esp_netif_ip_info_t ip_vals;
    IP4_ADDR(&ip_vals.ip, 10, 0, 0, 2);
    IP4_ADDR(&ip_vals.gw, 10, 0, 0, 1);
    IP4_ADDR(&ip_vals.netmask, 255, 0, 0, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_vals));

    // Register Event Handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

void app_main() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_eth_event_group = xEventGroupCreate();

    // A. Initialize Ethernet
    init_ethernet();
    
    // Wait for IP
    ESP_LOGI(TAG, "Waiting for IP...");
    xEventGroupWaitBits(s_eth_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // B. Initialize MIPI DSI Display (720x720)
    init_display();
    
    // C. Init H264
    init_decoder();

    // D. Start Task
    xTaskCreatePinnedToCore(udp_video_task, "video_task", 32 * 1024, panel_handle, 5, NULL, 1);
}