/**
 * @file main.c
 * @brief High-performance MJPEG video streaming receiver for ESP32-P4
 * 
 * Features:
 * - TCP transport with length-prefixed framing (reliable delivery)
 * - Hardware JPEG decoder (fast decode)
 * - Double buffering (decode while displaying)
 * - Direct RGB565 output to display
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "ethernet.h"
#include "display.h"
#include "my_jpeg_decoder.h"

static const char *TAG = "MJPEG_RX";

// Stream parameters
#define VIDEO_WIDTH     320
#define VIDEO_HEIGHT    320
#define DISPLAY_WIDTH   720
#define DISPLAY_HEIGHT  720
#define TCP_PORT        3333
#define MAX_JPEG_SIZE   (256 * 1024)  // 256KB max per frame

// Double buffer management
typedef struct {
    uint16_t *buffer;
    volatile bool ready;
} frame_buffer_t;

static frame_buffer_t s_frame_buffers[2];
static volatile int s_decode_idx = 0;  // Buffer being decoded into
static volatile int s_display_idx = 1; // Buffer being displayed
static SemaphoreHandle_t s_buffer_mutex = NULL;
static SemaphoreHandle_t s_frame_ready_sem = NULL;

// Statistics
static uint32_t s_frame_count = 0;
static uint32_t s_decode_errors = 0;
static int64_t s_last_stats_time = 0;

// Receive exactly n bytes from socket
static int recv_exact(int sock, uint8_t *buf, size_t len)
{
    size_t received = 0;
    while (received < len) {
        int r = recv(sock, buf + received, len - received, 0);
        if (r <= 0) return r;
        received += r;
    }
    return received;
}

// Display task - renders frames from double buffer
static void display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started");
    
    while (1) {
        // Wait for a frame to be ready
        if (xSemaphoreTake(s_frame_ready_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }
        
        // Get the buffer that's ready for display
        xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
        int idx = s_display_idx;
        frame_buffer_t *fb = &s_frame_buffers[idx];
        xSemaphoreGive(s_buffer_mutex);
        
        if (fb->ready && fb->buffer != NULL) {
            // Draw to display
            esp_lcd_panel_draw_bitmap(get_panel_handle(), 0, 0, VIDEO_WIDTH, VIDEO_HEIGHT, fb->buffer);
            display_wait_refresh_done();
            
            // Mark buffer as available
            fb->ready = false;
        }
    }
}

// TCP receiver task - receives MJPEG frames and decodes them
static void tcp_receiver_task(void *pvParameters)
{
    // Allocate JPEG receive buffer (needs to be large for 720p JPEG)
    uint8_t *jpeg_buffer = heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (jpeg_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "TCP receiver task started, waiting for connections on port %d", TCP_PORT);
    
    while (1) {
        // Create TCP server socket
        int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (server_sock < 0) {
            ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Allow socket reuse
        int opt = 1;
        setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // Bind
        struct sockaddr_in bind_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(TCP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };
        
        if (bind(server_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            ESP_LOGE(TAG, "Failed to bind: errno %d", errno);
            close(server_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Listen
        if (listen(server_sock, 1) < 0) {
            ESP_LOGE(TAG, "Failed to listen: errno %d", errno);
            close(server_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        ESP_LOGI(TAG, "Listening on port %d...", TCP_PORT);
        
        // Accept connection
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);
        
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Accept failed: errno %d", errno);
            close(server_sock);
            continue;
        }
        
        ESP_LOGI(TAG, "Client connected from %s", inet_ntoa(client_addr.sin_addr));
        
        // Set socket options for low latency
        int nodelay = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        
        // Receive frames
        // Protocol: [4-byte length (big-endian)][JPEG data]
        s_last_stats_time = esp_timer_get_time();
        
        while (1) {
            // Read frame length (4 bytes, big-endian)
            uint8_t len_buf[4];
            int r = recv_exact(client_sock, len_buf, 4);
            if (r <= 0) {
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }
            
            uint32_t frame_len = (len_buf[0] << 24) | (len_buf[1] << 16) | 
                                 (len_buf[2] << 8) | len_buf[3];
            
            if (frame_len == 0 || frame_len > MAX_JPEG_SIZE) {
                ESP_LOGE(TAG, "Invalid frame length: %lu", (unsigned long)frame_len);
                break;
            }
            
            // Read JPEG data
            r = recv_exact(client_sock, jpeg_buffer, frame_len);
            if (r <= 0) {
                ESP_LOGI(TAG, "Failed to receive frame data");
                break;
            }
            
            // Get buffer to decode into
            xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
            int idx = s_decode_idx;
            frame_buffer_t *fb = &s_frame_buffers[idx];
            xSemaphoreGive(s_buffer_mutex);
            
            // Decode JPEG
            uint16_t width, height;
            size_t buf_size = VIDEO_WIDTH * VIDEO_HEIGHT * 2;
            esp_err_t ret = my_jpeg_decoder_decode(jpeg_buffer, frame_len, 
                                                   (uint8_t *)fb->buffer, buf_size,
                                                   &width, &height);
            
            if (ret == ESP_OK) {
                s_frame_count++;
                
                // Swap buffers
                xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
                fb->ready = true;
                s_decode_idx = s_display_idx;
                s_display_idx = idx;
                xSemaphoreGive(s_buffer_mutex);
                
                // Signal display task
                xSemaphoreGive(s_frame_ready_sem);
                
                // Print stats every second
                int64_t now = esp_timer_get_time();
                if (now - s_last_stats_time >= 1000000) {
                    ESP_LOGI(TAG, "FPS: %lu, errors: %lu", 
                             (unsigned long)s_frame_count, (unsigned long)s_decode_errors);
                    s_frame_count = 0;
                    s_decode_errors = 0;
                    s_last_stats_time = now;
                }
            } else {
                s_decode_errors++;
            }
        }
        
        close(client_sock);
        close(server_sock);
        ESP_LOGI(TAG, "Connection closed, restarting server...");
    }
    
    free(jpeg_buffer);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Ethernet
    init_ethernet();
    ESP_LOGI(TAG, "Waiting for IP...");
    xEventGroupWaitBits(s_eth_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Initialize display
    ESP_LOGI(TAG, "Initializing display...");
    init_display();
    
    // Allocate double frame buffers in PSRAM (720x720 x 2 bytes x 2 buffers = ~2MB)
    size_t frame_size = VIDEO_WIDTH * VIDEO_HEIGHT * sizeof(uint16_t);
    ESP_LOGI(TAG, "Allocating double frame buffers: %u bytes each", (unsigned)frame_size);
    
    for (int i = 0; i < 2; i++) {
        s_frame_buffers[i].buffer = heap_caps_aligned_calloc(64, 1, frame_size, 
                                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_frame_buffers[i].buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            return;
        }
        s_frame_buffers[i].ready = false;
    }
    ESP_LOGI(TAG, "Frame buffers allocated");
    
    // Create synchronization primitives
    s_buffer_mutex = xSemaphoreCreateMutex();
    s_frame_ready_sem = xSemaphoreCreateBinary();
    
    // Initialize JPEG decoder
    my_jpeg_decoder_config_t jpeg_cfg = {
        .output_format = MY_JPEG_OUT_RGB565,
        .max_width = VIDEO_WIDTH,
        .max_height = VIDEO_HEIGHT,
    };
    ret = my_jpeg_decoder_init(&jpeg_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize JPEG decoder");
        return;
    }
    
    // Start tasks
    xTaskCreatePinnedToCore(display_task, "display", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(tcp_receiver_task, "tcp_rx", 8192, NULL, 5, NULL, 0);
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "MJPEG receiver ready!");
    ESP_LOGI(TAG, "Stream command (from PC):");
    ESP_LOGI(TAG, "  .\\stream_mjpeg.ps1");
    ESP_LOGI(TAG, "===========================================");
    
    // Main loop - just idle
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
