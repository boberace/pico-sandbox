#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "math.h"

#include "blink.pio.h"
#include "r2r.pio.h"

#define R2R_BITS 6
#define R2R_BASE_PIN 2
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define WAV_SAMPLES 512

#define system_frequency clock_get_hz(clk_sys)

uint32_t sine_wave[WAV_SAMPLES];

int dma_chan_array_byte;
int dma_chan_loop_array;

static inline void put_value(PIO pio, uint sm, uint32_t value) {
    pio_sm_put_blocking(pio, sm, value);
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void gen_waves();
void r2r_forever(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, uint freq);
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins);

int main()
{
    stdio_init_all();
    gen_waves();

    sleep_ms(500);

    PIO pio_blink = pio0;
    uint sm_blink = 0;
    uint offset_blink = pio_add_program(pio_blink, &blink_program);
    printf("Loaded blink program at %d\n", offset_blink);  
    blink_pin_forever(pio_blink, sm_blink, offset_blink, 15, 1);

    PIO pio_r2r = pio1;
    uint sm_r2r = 0;
    uint offset_r2r = pio_add_program(pio_r2r, &r2r_program);
    r2r_forever(pio_r2r, sm_r2r, offset_r2r, R2R_BASE_PIN, R2R_BITS, 1);
    printf("Loaded r2r program at %d\n", offset_r2r);  

    
    for (int i = 0; i < WAV_SAMPLES; i++) {
        uint8_t value = sine_wave[i];  
        printf("%d\n", value);  
    }

    uint count = 0;
    while (true) {
        // printf("Hello, world! %i\n", count);
        // count++;
        // sleep_ms(1000);
        for (int i = 0; i < WAV_SAMPLES; i++) {
            put_value(pio_r2r, sm_r2r, sine_wave[i]);
            // sleep_ms(2);
        }
    }
}

void blink_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = blink_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin, 1);
    pio_sm_init(pio, sm, offset, &c);
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (system_frequency / (2 * freq)) - 3;
}

void gen_waves() { // =ROUND(63*(SIN(A1*2*PI()/256)/2+0.5),0)
    for (int i = 0; i < WAV_SAMPLES; i++) {
        sine_wave[i] = (uint8_t)round(R2R_MAX * (sin(i * 2 * M_PI / WAV_SAMPLES) / 2 + 0.5));
    }
}

void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins) {
    pio_sm_config c = r2r_program_get_default_config(offset);
    sm_config_set_out_pins(&c, base_pin, num_pins); 
    sm_config_set_out_shift(&c, true, true, 32); 
    for(int i = 0; i < num_pins; i++) {
        pio_gpio_init(pio, base_pin + i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, base_pin, num_pins, true);
    pio_sm_init(pio, sm, offset, &c);
}

void r2r_forever(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, uint freq) {
    r2r_program_init(pio, sm, offset, base_pin, num_pins);  
    pio_sm_set_enabled(pio, sm, true);

}