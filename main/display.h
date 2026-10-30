#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

// Number of driver-allocated frame buffers. 2 = copy-free, tear-free swaps.
#ifndef DISPLAY_NUM_FBS
#define DISPLAY_NUM_FBS 2
#endif

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
 * @brief True if the panel has two frame buffers (copy-free presentation).
 */
bool display_has_double_buffer(void);

/**
 * @brief Size in bytes of one frame buffer (LCD_H_RES x LCD_V_RES RGB565).
 */
size_t display_frame_buffer_size(void);

/**
 * @brief Get the frame buffer that is not on screen.
 *
 * Waits (rarely) until the panel has stopped scanning it after the last swap.
 * Write the next frame into it, then call display_present().
 *
 * @return Back buffer, or NULL if double buffering is unavailable.
 */
uint16_t *display_acquire_back_buffer(void);

/**
 * @brief Show a buffer returned by display_acquire_back_buffer().
 *
 * No copy: the panel switches to it at the next refresh.
 */
void display_present(uint16_t *fb);

/**
 * @brief Draw color bar test pattern on the display
 */
void display_draw_color_bar(void);

#endif // DISPLAY_H
