/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"

#include "st7789_lcd.pio.h"
#include "raspberry_256x256_rgb565.h"

// Tested with the parts that have the height of 240 and 320
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define IMAGE_SIZE 256
#define LOG_IMAGE_SIZE 8

#define PIN_DIN 17
#define PIN_CLK 16
#define PIN_CS 14
#define PIN_DC 15
// #define PIN_RESET 4
// #define PIN_BL 5

#define SERIAL_CLK_DIV 1.f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// For optimal use of DMA bandwidth we would use an autopull threshold of 32,
// but we are using a threshold of 8 here (consume 1 byte from each FIFO entry
// and discard the remainder) to make things easier for software on the other side

static inline void st7789_lcd_program_init(PIO pio, uint sm, uint offset, uint data_pin, uint clk_pin, float clk_div) {
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clk_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, data_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, clk_pin, 1, true);
    pio_sm_config c = st7789_lcd_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, clk_pin);
    sm_config_set_out_pins(&c, data_pin, 1);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, clk_div);
    sm_config_set_out_shift(&c, false, true, 8);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Making use of the narrow store replication behaviour on RP2040 to get the
// data left-justified (as we are using shift-to-left to get MSB-first serial)

static inline void st7789_lcd_put(PIO pio, uint sm, uint8_t x) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        ;
    *(volatile uint8_t*)&pio->txf[sm] = x;
}

// SM is done when it stalls on an empty FIFO

static inline void st7789_lcd_wait_idle(PIO pio, uint sm) {
    uint32_t sm_stall_mask = 1u << (sm + PIO_FDEBUG_TXSTALL_LSB);
    pio->fdebug = sm_stall_mask;
    while (!(pio->fdebug & sm_stall_mask))
        ;
}

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
static const uint8_t st7789_init_seq[] = {
        1, 20, 0x01,                        // Software reset
        1, 10, 0x11,                        // Exit sleep mode
        2, 2, 0x3a, 0x55,                   // Set colour mode to 16 bit
        2, 0, 0x36, 0x00,                   // Set MADCTL: row then column, refresh is bottom to top ????
        5, 0, 0x2a, 0x00, 0x00, SCREEN_WIDTH >> 8, SCREEN_WIDTH & 0xff,   // CASET: column addresses
        5, 0, 0x2b, 0x00, 0x00, SCREEN_HEIGHT >> 8, SCREEN_HEIGHT & 0xff, // RASET: row addresses
        1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                         // Normal display on, then 10 ms delay
        1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
        0                                   // Terminate list
};

static inline void lcd_set_dc_cs(bool dc, bool cs) {
    sleep_us(1);
    gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count) {
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    st7789_lcd_put(pio, sm, *cmd++);
    if (count >= 2) {
        st7789_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            st7789_lcd_put(pio, sm, *cmd++);
    }
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void st7789_start_pixels(PIO pio, uint sm) {
    uint8_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
}

int main() {
    stdio_init_all();

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &st7789_lcd_program);
    st7789_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

    gpio_init(PIN_CS);
    gpio_init(PIN_DC);
    // gpio_init(PIN_RESET);
    // gpio_init(PIN_BL);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    // gpio_set_dir(PIN_RESET, GPIO_OUT);
    // gpio_set_dir(PIN_BL, GPIO_OUT);

    gpio_put(PIN_CS, 1);
    // gpio_put(PIN_RESET, 1);
    lcd_init(pio, sm, st7789_init_seq);
    // gpio_put(PIN_BL, 1);

    // Other SDKs: static image on screen, lame, boring
    // Raspberry Pi Pico SDK: spinning image on screen, bold, exciting

    // Lane 0 will be u coords (bits 8:1 of addr offset), lane 1 will be v
    // coords (bits 16:9 of addr offset), and we'll represent coords with
    // 16.16 fixed point. ACCUM0,1 will contain current coord, BASE0/1 will
    // contain increment vector, and BASE2 will contain image base pointer
#define UNIT_LSB 16
    interp_config lane0_cfg = interp_default_config();
    interp_config_set_shift(&lane0_cfg, UNIT_LSB - 1); // -1 because 2 bytes per pixel
    interp_config_set_mask(&lane0_cfg, 1, 1 + (LOG_IMAGE_SIZE - 1));
    interp_config_set_add_raw(&lane0_cfg, true); // Add full accumulator to base with each POP
    interp_config lane1_cfg = interp_default_config();
    interp_config_set_shift(&lane1_cfg, UNIT_LSB - (1 + LOG_IMAGE_SIZE));
    interp_config_set_mask(&lane1_cfg, 1 + LOG_IMAGE_SIZE, 1 + (2 * LOG_IMAGE_SIZE - 1));
    interp_config_set_add_raw(&lane1_cfg, true);

    interp_set_config(interp0, 0, &lane0_cfg);
    interp_set_config(interp0, 1, &lane1_cfg);
    interp0->base[2] = (uint32_t) raspberry_256x256;

    float theta = 0.f;
    float theta_max = 2.f * (float) M_PI;
    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());   
    int pixel_index = 0;
    int num_pixel = SCREEN_WIDTH * SCREEN_HEIGHT;

    while (1) {
        uint32_t delta_millis_display = curr_millis_display - prev_millis_display;
        prev_millis_display = curr_millis_display;
        curr_millis_display = to_ms_since_boot(get_absolute_time());
        printf("theta %f, milliseconds %d\n",theta, delta_millis_display);
        theta += 0.0025f;
        if (theta > theta_max)
            theta -= theta_max;
        int32_t rotate[4] = {
                (int32_t) (cosf(theta) * (1 << UNIT_LSB)), (int32_t) (-sinf(theta) * (1 << UNIT_LSB)),
                (int32_t) (sinf(theta) * (1 << UNIT_LSB)), (int32_t) (cosf(theta) * (1 << UNIT_LSB))
        };
        interp0->base[0] = rotate[0];
        interp0->base[1] = rotate[2];
        st7789_start_pixels(pio, sm);
        for (int y = 0; y < SCREEN_HEIGHT; ++y) {
            interp0->accum[0] = rotate[1] * y;
            interp0->accum[1] = rotate[3] * y;
            for (int x = 0; x < SCREEN_WIDTH; ++x) {
                uint16_t colour = *(uint16_t *) (interp0->pop[2]);
                st7789_lcd_put(pio, sm, colour >> 8);
                st7789_lcd_put(pio, sm, colour & 0xff);
            }
        }
    }

}

