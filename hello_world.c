#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include "hardware/spi.h"
#include "time.h"
#include <string.h>

#include "src/drivers/display/ili9341/lv_ili9341.h"

#include <lvgl.h>

#define _CASET 0x2a     // Column Address Set
#define _PASET 0x2b     // Page Address Set
#define _RAMWR 0x2c     // Memory Write

#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define DISPLAY_DEPTH  2

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define PIN_DC   8
#define PIN_RST  9

#define SPI_PORT spi0

uint16_t displayBuffer[DISPLAY_WIDTH][DISPLAY_HEIGHT];

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
    lv_draw_sw_rgb565_swap(displayBuffer, (DISPLAY_WIDTH * DISPLAY_HEIGHT));

    _writeBlock(area->x1, area->y1, area->x2, area->y2, px_map, (area->x2 - area->x1) * (area->y2 - area->y1) * 2);

    lv_display_flush_ready(disp);
}

static uint32_t my_tick(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void lv_draw_spinner(void)
{
    /*Create a spinner*/
    lv_obj_t * spinner = lv_spinner_create(lv_screen_active());
    lv_obj_set_size(spinner, 100, 100);
    lv_obj_center(spinner);
    lv_spinner_set_anim_params(spinner, 1000, 200);
}

int32_t my_lcd_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size)
{
    gpio_put(PIN_DC, 0);
    gpio_put(PIN_CS, 0);

    while (!spi_is_writable(SPI_PORT))
    {
        sleep_us(1);
    }

    spi_write_blocking(SPI_PORT, cmd, cmd_size);

    gpio_put(PIN_CS, 1);

    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);

    spi_write_blocking(SPI_PORT, param, param_size);

    gpio_put(PIN_CS, 1);

    return 0;
}

/* Send large array of pixel data to the LCD. If necessary, this function has to do the byte-swapping. This function can do the transfer in the background. */
int32_t my_lcd_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size)
{
    lv_display_flush_ready(disp);

    return 0;
}

void _data(uint8_t *data, size_t dataLen)
{
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);

    spi_write_blocking(SPI_PORT, data, dataLen);

    gpio_put(PIN_CS, 1);
}

void _write(uint8_t cmd, uint8_t *data, size_t dataLen)
{
    gpio_put(PIN_DC, 0);
    gpio_put(PIN_CS, 0);

    // spi write
    uint8_t commandBuffer[1];
    commandBuffer[0] = cmd;

    while (!spi_is_writable(SPI_PORT))
    {
        sleep_us(1);
    }

    spi_write_blocking(SPI_PORT, commandBuffer, 1);

    gpio_put(PIN_CS, 1);

    // do stuff
    if (data != NULL)
    {
        _data(data, dataLen);
    }
}

void _writeBlock(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t *data, size_t dataLen)
{
    uint16_t buffer[2];
    buffer[0] = __builtin_bswap16(x0);
    buffer[1] = __builtin_bswap16(x1);

    _write(_CASET, (uint8_t *)buffer, 4);

    buffer[0] = __builtin_bswap16(y0);
    buffer[1] = __builtin_bswap16(y1);

    _write(_PASET, (uint8_t *)buffer, 4);
    _write(_RAMWR, data, dataLen);
}

LV_IMG_DECLARE(x281) 

int main() 
{
    stdio_init_all();

    spi_init(SPI_PORT, 24 * 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);

    gpio_put(PIN_RST, 0);
    sleep_us(30);
    gpio_put(PIN_RST, 1);
    sleep_us(30);

    lv_init();
    lv_tick_set_cb(my_tick);

    lv_display_t *disp = lv_ili9341_create(DISPLAY_WIDTH, DISPLAY_HEIGHT, LV_LCD_FLAG_NONE, my_lcd_send_cmd, my_lcd_send_color);

    // grön (5), röd(6), blå (5)

    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, displayBuffer, 0, DISPLAY_WIDTH*DISPLAY_HEIGHT*DISPLAY_DEPTH, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_obj_t * icon = lv_img_create(lv_scr_act());
    lv_img_set_src(icon, &x281);

    lv_draw_spinner();

    while(1)
    {
        uint32_t time_till_next = lv_timer_handler();

        sleep_ms(time_till_next);
    }

    return 0;
}