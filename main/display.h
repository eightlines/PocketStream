#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"

#define LCD_H_RES 720
#define LCD_V_RES 720
#define LCD_BIT_PER_PIXEL 16  // RGB565

#define PIN_NUM_LCD_RST (27)
#define PIN_NUM_BK_LIGHT (26)
#define LCD_BK_LIGHT_ON_LEVEL (0)
#define LCD_BK_LIGHT_OFF_LEVEL (!LCD_BK_LIGHT_ON_LEVEL)

#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500

#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB565)

/**
 * @brief Initialize the MIPI DSI display
 * 
 * Configures the ST7703 LCD panel driver with 720x720 resolution,
 * RGB666 pixel format, and enables the backlight.
 */
void init_display(void);

/**
 * @brief Get the LCD panel handle
 * 
 * @return esp_lcd_panel_handle_t The panel handle for drawing operations
 */
esp_lcd_panel_handle_t get_panel_handle(void);

/**
 * @brief Wait for the display refresh to complete
 */
void display_wait_refresh_done(void);

/**
 * @brief Draw color bar test pattern on the display
 */
void display_draw_color_bar(void);

#endif // DISPLAY_H
