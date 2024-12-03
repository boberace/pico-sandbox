/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"


#define NUM_LEDS_PER_STRIP 250
#define NUM_STRIPS 8
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NUM_STRIPS)
#define WS2812_PIN_BASE 6

#define NUM_BYTES_COLOR 3
#define NUM_BITS_COLOR (NUM_BYTES_COLOR * 8)

#define LED_PIN 25

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 0
#define UART_A_RX_PIN 1
#define UART_A_DATA_BITS 8
#define UART_A_STOP_BITS 1
#define UART_A_PARITY    UART_PARITY_NONE

PIO pio = pio0;
int sm = 0;

uint32_t LEDS[NUM_LEDS] = {0};

#define NUM_BYTES 9
uint8_t byte_buffer[NUM_BYTES];
volatile bool recieved = false;

auto_init_mutex(puts_mutex);
int dma_chan_array_byte;
int dma_chan_loop_array;

void putsblocking(char *b){

    mutex_enter_blocking(&puts_mutex);
    puts(b);
    mutex_exit(&puts_mutex);

}

void __isr dma_handler() {
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan_array_byte;
    recieved = true;
    
}

void setup_dma() {
    // Claim two DMA channels.
    dma_chan_array_byte = 0;
    dma_chan_loop_array = 1;

    // Set up the DMA channel configuration.
    dma_channel_config c_array = dma_channel_get_default_config(dma_chan_array_byte);
    dma_channel_config c_loop = dma_channel_get_default_config(dma_chan_loop_array);

    // // Set the transfer request to be triggered by UART RX.
    // channel_config_set_transfer_data_size(&c_array, DMA_SIZE_8);
    // channel_config_set_read_increment(&c_array, false);
    // channel_config_set_write_increment(&c_array, true);
    // channel_config_set_dreq(&c_array, DREQ_UART1_RX);

    // // Configure the DMA channel for circular buffer.
    // dma_channel_configure(
    //     dma_chan_array_byte,
    //     &c_array,
    //     byte_buffer,          // Destination pointer
    //     &uart_get_hw(UART_A_ID)->dr, // Source pointer (UART data register)
    //     NUM_BYTES,            // Number of transfers
    //     true                 // Start immediately
    // );

    // dma_channel_set_irq0_enabled(dma_chan_array_byte, true);
    // irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    // irq_set_enabled(DMA_IRQ_0, true);

    channel_config_set_read_increment(&c_loop, false);
    channel_config_set_write_increment(&c_loop, false);
    // channel_config_set_chain_to(&c_loop, dma_chan_array_byte);

    uint8_t * p_byte_buffer = &byte_buffer[0];
    dma_channel_configure(
        dma_chan_loop_array,
        &c_loop,
        &dma_hw->ch[dma_chan_array_byte].al2_write_addr_trig,
        &p_byte_buffer,           
        1,              
        false           
    );


    // Set the transfer request to be triggered by UART RX.
    channel_config_set_read_increment(&c_array, false);
    channel_config_set_write_increment(&c_array, true);
    channel_config_set_dreq(&c_array, DREQ_UART1_RX);
    channel_config_set_chain_to(&c_array, dma_chan_loop_array);
    channel_config_set_transfer_data_size(&c_array, DMA_SIZE_8); 

    // Set the transfer request to be triggered by the loop.
    dma_channel_configure(
        dma_chan_array_byte,
        &c_array,
        byte_buffer,          // Destination pointer
        &uart_get_hw(UART_A_ID)->dr, // Source pointer (UART data register)
        NUM_BYTES,
        false
    );

    // Set up the interrupt handler.
    dma_channel_set_irq0_enabled(dma_chan_array_byte, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);


    // Start the DMA channel.
    dma_start_channel_mask(1u << dma_chan_loop_array);

}

uint setup_uarta() {

    uart_init(UART_A_ID, 2400);
    gpio_set_function(UART_A_TX_PIN, UART_FUNCSEL_NUM(UART_A_ID, UART_A_TX_PIN));
    gpio_set_function(UART_A_RX_PIN, UART_FUNCSEL_NUM(UART_A_ID, UART_A_RX_PIN));
    int actual = uart_set_baudrate(UART_A_ID, UART_A_BAUD_RATE);    
    uart_set_hw_flow(UART_A_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off    
    uart_set_format(UART_A_ID, UART_A_DATA_BITS, UART_A_STOP_BITS, UART_A_PARITY);// Set our data format    
    uart_set_fifo_enabled(UART_A_ID, false);// Turn off FIFO's - we want to do this character by character
    uart_set_irq_enables(UART_A_ID, true, false);
    return actual;
}



static inline void put_bits(uint32_t bit_array) {

    pio_sm_put_blocking(pio, sm, bit_array);

}


static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}


static inline void ws2812_parallel_program_init(PIO pio, uint sm, uint offset, uint pin_base, uint pin_count, float freq) {
    for(uint i=pin_base; i<pin_base+pin_count; i++) {
        pio_gpio_init(pio, i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, pin_count, true);

    pio_sm_config c = ws2812_parallel_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_out_pins(&c, pin_base, pin_count);
    sm_config_set_set_pins(&c, pin_base, pin_count);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = ws2812_parallel_T1 + ws2812_parallel_T2 + ws2812_parallel_T3;
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void send_leds() {
    for(uint l=0; l<NUM_LEDS_PER_STRIP; l++) {
        for(uint b=0; b<NUM_BITS_COLOR; b++) {
            uint32_t bits = 0;
            for(uint s=0; s<NUM_STRIPS; s++) {
                bits = bits | (((LEDS[l + s * NUM_LEDS_PER_STRIP] & (1 << b)) ? 1 : 0) << s);
            }
            put_bits(bits);
        }
    }
}


void core1_entry(){

    char buffer[100];

    uint uart_ret = setup_uarta();
    sprintf(buffer, "UART setup baud rate %d\r\n", uart_ret);
    putsblocking(buffer);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint counter = 0;
    while (true) {       

        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0); 
        sleep_ms(100);
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(700);
        counter++;
        
    }

}

int main() {
    
    stdio_init_all();
    sleep_ms(1000);
    char buffer[100];
    multicore_launch_core1(core1_entry);

    uint offset = pio_add_program(pio, &ws2812_parallel_program);

    ws2812_parallel_program_init(pio, sm, offset, WS2812_PIN_BASE, NUM_STRIPS, 800000);

    for(uint l=0; l<NUM_LEDS; l++) {
        LEDS[l] = urgb_u32(255, 0, 0);
    }

    for(uint l=0; l<NUM_LEDS_PER_STRIP; l++) {
        uint8_t bt = 0;

        LEDS[l + 0 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, 0, 0);
        LEDS[l + 1 * NUM_LEDS_PER_STRIP] = urgb_u32(0, bt, 0);
        LEDS[l + 2 * NUM_LEDS_PER_STRIP] = urgb_u32(0, 0, bt);
        LEDS[l + 3 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, bt, 0);
        LEDS[l + 4 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, 0, bt);
        LEDS[l + 5 * NUM_LEDS_PER_STRIP] = urgb_u32(0, bt, bt);
        LEDS[l + 6 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, bt, bt);
        LEDS[l + 7 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, bt >> 1, bt >> 1);

    }

    send_leds();
    sleep_ms(1000);

    for(uint l=0; l<NUM_LEDS_PER_STRIP; l++) {
        uint8_t bt = 4;

        LEDS[l + 0 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, 0, 0);
        LEDS[l + 1 * NUM_LEDS_PER_STRIP] = urgb_u32(0, bt, 0);
        LEDS[l + 2 * NUM_LEDS_PER_STRIP] = urgb_u32(0, 0, bt);
        LEDS[l + 3 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, bt, 0);
        LEDS[l + 4 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, 0, bt);
        LEDS[l + 5 * NUM_LEDS_PER_STRIP] = urgb_u32(0, bt, bt);
        LEDS[l + 6 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, bt, bt);
        LEDS[l + 7 * NUM_LEDS_PER_STRIP] = urgb_u32(bt, bt >> 1, bt >> 1);

    }

    uint32_t us_prev = to_ms_since_boot(get_absolute_time());
    send_leds();
    uint32_t us_after = to_ms_since_boot(get_absolute_time());

    sprintf (buffer, "Before: %d ms\n After: %d ms\n Delta: %d ms\n", us_prev,us_after,us_after-us_prev);

    putsblocking(buffer);

    mutex_init(&puts_mutex);

    setup_dma();

    while (true){
        if (recieved) {
            char buffer[100];
            sprintf(buffer, "interrupt triggered\n");
            putsblocking(buffer);
            for (int i = 0; i < NUM_BYTES; i++) {
                sprintf(buffer, "%d: %d\n",i, byte_buffer[i]);
                putsblocking(buffer);
            }
            recieved = false;
        }
    }

}

