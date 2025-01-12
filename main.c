#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "time.h"

#include "src/drivers/display/ili9341/lv_ili9341.h"
#include "src/widgets/chart/lv_chart.h"
#include "src/widgets/chart/lv_chart_private.h"

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

LV_IMG_DECLARE(x281) 

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
    lv_draw_sw_rgb565_swap(displayBuffer, (DISPLAY_WIDTH * DISPLAY_HEIGHT));

    _writeBlock(area->x1, area->y1, area->x2, area->y2, px_map, (area->x2 - area->x1) * (area->y2 - area->y1) * DISPLAY_DEPTH);

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
    lv_spinner_set_anim_params(spinner, 2000, 300);
}

void lv_example_label_1(void)
{
    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    lv_style_set_text_font(&style_btn, &lv_font_montserrat_24); 

    lv_obj_t * label1 = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label1, LV_LABEL_LONG_MODE_WRAP);     /*Break the long lines*/
    lv_label_set_recolor(label1, true);                      /*Enable re-coloring by commands in the text*/
    lv_label_set_text(label1, "#0000ff Re-color# #ff00ff words# #ff0000 of a# label, align the lines to the center ");
    lv_obj_set_width(label1, 150);  /*Set smaller width to make the lines wrap*/
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_add_style(label1, &style_btn, 0);
    
    lv_obj_t * label2 = lv_label_create(lv_screen_active());
    lv_label_set_long_mode(label2, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);     /*Circular scroll*/
    lv_obj_set_width(label2, 150);
    lv_label_set_text(label2, "It is a circularly scrolling text. ");
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 80);
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

void lv_example_chart_1(void)
{
    /*Create a chart*/
    lv_obj_t * chart;
    chart = lv_chart_create(lv_screen_active());
    lv_obj_set_size(chart, 300, 150);
    lv_obj_center(chart);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_point_count(chart, 20);

    /*Add two data series*/
    lv_chart_series_t * ser1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t * ser2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_SECONDARY_Y);

    uint32_t i;
    for(i = 0; i < 20; i++) {
        /*Set the next points on 'ser1'*/
        lv_chart_set_next_value(chart, ser1, lv_rand(10, 50));

        /*Directly set points on 'ser2'*/
        ser2->y_points[i] = lv_rand(50, 90);
    }

    lv_chart_refresh(chart); /*Required after direct set*/
}

int main() 
{
    stdio_init_all();

    spi_init(SPI_PORT, 48 * 1000 * 1000);
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

    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, displayBuffer, 0, DISPLAY_WIDTH*DISPLAY_HEIGHT*DISPLAY_DEPTH, LV_DISPLAY_RENDER_MODE_PARTIAL);

    //lv_obj_t * icon = lv_img_create(lv_scr_act());
    //lv_img_set_src(icon, &x281);

    //lv_draw_spinner();

    //lv_example_label_1();

    lv_example_chart_1();

    while(1)
    {
        uint32_t time_till_next = lv_timer_handler();

        sleep_ms(time_till_next);
    }

    return 0;
}