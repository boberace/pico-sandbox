#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "blink.pio.h"
#include "r2r.pio.h"

#define system_frequency clock_get_hz(clk_sys)

#define R2R_BITS 6
#define PIN_R2R_BASE 10 // 11-15 also used
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define NUM_WAV_SAMPLES 125

PIO pio_r2r = pio1;
uint sm_r2r = 0;

volatile uint8_t fun_wave[NUM_WAV_SAMPLES];
volatile uint8_t * p_fun_wave = &fun_wave[0];

int dma_chan_wave_bytes;
int dma_chan_wave_loop;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void setup_waves();
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq);
void setup_r2r(float freq);
void setup_dma_wave();

int main()
{
    stdio_init_all();

    setup_waves();  
    setup_r2r(100.0);

    setup_dma_wave();

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);    

    blink_pin_forever(pio, 0, offset, 25, 2);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}

void blink_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = blink_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin, 1);
    pio_sm_init(pio, sm, offset, &c);
 }

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq)
{
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

void setup_waves() 
{ // =ROUND(63*(SIN(A1*2*PI()/256)/2+0.5),0)
    for (int i = 0; i < NUM_WAV_SAMPLES; i++) {
        fun_wave[i] = (uint8_t)round(R2R_MAX * (sin(i * 2 * M_PI / NUM_WAV_SAMPLES) / 2 + 0.5));
    }
}

void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq) 
{
    pio_sm_config c = r2r_program_get_default_config(offset);
    sm_config_set_out_pins(&c, base_pin, num_pins); 
    sm_config_set_out_shift(&c, true, true, 8); 
    for(int i = 0; i < num_pins; i++) {
        pio_gpio_init(pio, base_pin + i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, base_pin, num_pins, true);
    sm_config_set_clkdiv(&c, system_frequency/NUM_WAV_SAMPLES/freq);
    pio_sm_init(pio, sm, offset, &c);
}

void setup_r2r(float freq) 
{
    uint offset_r2r = pio_add_program(pio_r2r, &r2r_program);
    printf("Loaded r2r program at %d\n", offset_r2r);     
    r2r_program_init(pio_r2r, sm_r2r, offset_r2r, PIN_R2R_BASE, R2R_BITS, freq);  
    pio_sm_set_enabled(pio_r2r, sm_r2r, true);

}


void setup_dma_wave()
{

    dma_chan_wave_bytes = dma_claim_unused_channel(true); 
    dma_chan_wave_loop = dma_claim_unused_channel(true);

    dma_channel_config c_wave = dma_channel_get_default_config(dma_chan_wave_bytes);
    dma_channel_config c_loop = dma_channel_get_default_config(dma_chan_wave_loop);

    uint pio_r2r_dreq = pio_get_dreq(pio_r2r, sm_r2r, true);
      
    channel_config_set_read_increment(&c_wave, true);
    channel_config_set_write_increment(&c_wave, false);
    channel_config_set_dreq(&c_wave, pio_r2r_dreq);
    channel_config_set_chain_to(&c_wave, dma_chan_wave_loop);
    channel_config_set_transfer_data_size(&c_wave, DMA_SIZE_8);
    channel_config_set_irq_quiet(&c_wave, false);
    dma_channel_configure(
        dma_chan_wave_bytes, 
        &c_wave, 
        &pio1_hw->txf[sm_r2r], 
        NULL, // loop channel fills this in
        NUM_WAV_SAMPLES, 
        false
    );

    channel_config_set_read_increment(&c_loop, false);
    channel_config_set_write_increment(&c_loop, false);
    channel_config_set_chain_to(&c_loop, dma_chan_wave_bytes);
    channel_config_set_transfer_data_size(&c_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_loop, false);
    dma_channel_configure(
        dma_chan_wave_loop, 
        &c_loop, 
        &dma_hw->ch[dma_chan_wave_bytes].read_addr, 
        &p_fun_wave,
        1, 
        true
    );
}
