#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_st7703.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "unity.h"

static const char *TAG = "DISPLAY";

static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
static SemaphoreHandle_t refresh_finish = NULL;

// Double buffering: the panel scans out s_fbs[s_front] while the next frame is
// written into the other one, then display_present() swaps them (no copy).
static void    *s_fbs[2] = {0};
static int      s_num_fbs   = 0;
static int      s_front     = 0;     // driver starts scanning fb 0
static bool     s_presented = false;
static uint32_t s_present_vsync = 0;
static volatile uint32_t s_vsync_count = 0;
static SemaphoreHandle_t s_vsync_sem  = NULL;

IRAM_ATTR static bool notify_refresh_ready(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    BaseType_t need_yield = pdFALSE;
    xSemaphoreGiveFromISR(sem, &need_yield);
    return (need_yield == pdTRUE);
}

// Fires once per refresh. On this chip revision it is raised when the DPI DMA
// restarts on the current frame buffer, i.e. right after a swap takes effect.
IRAM_ATTR static bool on_vsync(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    s_vsync_count++;
    BaseType_t need_yield = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync_sem, &need_yield);
    return (need_yield == pdTRUE);
}

void init_display(void)
{
    refresh_finish = xSemaphoreCreateBinary();
    s_vsync_sem    = xSemaphoreCreateBinary();
    
#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));
#endif

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
    // Use the library's tested DPI config - includes correct timing and DMA2D enabled
    esp_lcd_dpi_panel_config_t dpi_config = ST7703_720_720_PANEL_60HZ_DPI_CONFIG(MIPI_DPI_PX_FORMAT);
    dpi_config.num_fbs = DISPLAY_NUM_FBS;  // driver allocates these in PSRAM
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
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7703(mipi_dbi_io, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Register refresh callback
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = notify_refresh_ready,
        .on_refresh_done     = on_vsync,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, refresh_finish));
    ESP_LOGI(TAG, "Refresh callback registered");

#if DISPLAY_NUM_FBS >= 2
    if (esp_lcd_dpi_panel_get_frame_buffer(panel_handle, 2, &s_fbs[0], &s_fbs[1]) == ESP_OK &&
            s_fbs[0] && s_fbs[1]) {
        s_num_fbs = 2;
        ESP_LOGI(TAG, "Double buffered: fb0=%p fb1=%p", s_fbs[0], s_fbs[1]);
    } else {
        ESP_LOGW(TAG, "Double buffering unavailable; using copy-to-framebuffer path");
    }
#endif
}

bool display_has_double_buffer(void)
{
    return s_num_fbs >= 2;
}

size_t display_frame_buffer_size(void)
{
    return (size_t)LCD_H_RES * LCD_V_RES * (LCD_BIT_PER_PIXEL / 8);
}

uint16_t *display_acquire_back_buffer(void)
{
    if (s_num_fbs < 2) return NULL;
    // The previous swap only takes effect at the next refresh; until then the
    // panel may still be reading this buffer. Usually already past by now.
    while (s_presented && (uint32_t)(s_vsync_count - s_present_vsync) < 1) {
        if (xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "No vsync for 100 ms");
            break;
        }
    }
    return (uint16_t *)s_fbs[s_front ^ 1];
}

void display_present(uint16_t *fb)
{
    int idx = (fb == s_fbs[1]) ? 1 : 0;
    // A pointer inside a driver frame buffer makes draw_bitmap switch to it
    // (cache write-back only, no copy).
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, fb);
    s_front         = idx;
    s_present_vsync = s_vsync_count;
    s_presented     = true;
    // draw_bitmap signalled "transfer done" synchronously; consume it so the
    // copy path's display_wait_refresh_done() stays in step.
    xSemaphoreTake(refresh_finish, 0);
}

esp_lcd_panel_handle_t get_panel_handle(void)
{
    return panel_handle;
}

void display_wait_refresh_done(void)
{
    xSemaphoreTake(refresh_finish, portMAX_DELAY);
}

void display_draw_color_bar(void)
{
    uint8_t byte_per_pixel = (LCD_BIT_PER_PIXEL + 7) / 8;
    uint16_t row_line = LCD_V_RES / byte_per_pixel / 8;
    uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * LCD_H_RES * byte_per_pixel, MALLOC_CAP_DMA);

    for (int j = 0; j < byte_per_pixel * 8; j++)
    {
        for (int i = 0; i < row_line * LCD_H_RES; i++)
        {
            for (int k = 0; k < byte_per_pixel; k++)
            {
                color[i * byte_per_pixel + k] = (BIT(j) >> (k * 8)) & 0xff;
            }
        }
        TEST_ESP_OK(esp_lcd_panel_draw_bitmap(panel_handle, 0, j * row_line, LCD_H_RES, (j + 1) * row_line, color));
        display_wait_refresh_done();
    }

    uint16_t color_line = row_line * byte_per_pixel * 8;
    uint16_t res_line = LCD_V_RES - color_line;
    if (res_line)
    {
        for (int i = 0; i < res_line * LCD_H_RES; i++)
        {
            for (int k = 0; k < byte_per_pixel; k++)
            {
                color[i * byte_per_pixel + k] = 0xff;
            }
        }
        TEST_ESP_OK(esp_lcd_panel_draw_bitmap(panel_handle, 0, color_line, LCD_H_RES, LCD_V_RES, color));
        display_wait_refresh_done();
    }

    free(color);
}
