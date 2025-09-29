#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "blink.pio.h"
#include "r2r.pio.h"

#define system_frequency clock_get_hz(clk_sys)

#define CAPTURE_CHANNEL 0 // ADC0 is GPIO26

#define R2R_BITS 10
#define PIN_R2R_BASE 0 
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define NUM_WAV_SAMPLES (1 << R2R_BITS)

PIO pio_r2r = pio1;
uint sm_r2r = 0;

volatile uint16_t fun_wave[NUM_WAV_SAMPLES];
volatile uint16_t * p_fun_wave = &fun_wave[0];

int dma_chan_dac_data, dma_chan_dac_loop, dma_chan_adc_data;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void setup_waves();
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq);
void setup_r2r(float freq);
void setup_dma_adc();
void setup_dma_dac();

int main()
{
    stdio_init_all();

    setup_waves();  
    setup_r2r(20000.0);

    setup_dma_adc();
    setup_dma_dac();

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);    

    blink_pin_forever(pio, 0, offset, 25, 2);
    uint count = 0;

    while (true) {
        printf("%d, system freq %d\n", count, system_frequency);
        count++;
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
    pio->txf[sm] = (system_frequency / (2 * freq)) - 3;
}

void setup_waves() 
{ // =ROUND(63*(SIN(A1*2*PI()/256)/2+0.5),0)
    for (int i = 0; i < NUM_WAV_SAMPLES; i++) {
        // fun_wave[i] = (uint16_t)round(R2R_MAX * (sin(i * 2 * M_PI / NUM_WAV_SAMPLES) / 2 + 0.5));
        fun_wave[i] = 1;
    }
}

void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq) 
{
    pio_sm_config c = r2r_program_get_default_config(offset);
    sm_config_set_out_pins(&c, base_pin, num_pins); 
    sm_config_set_out_shift(&c, true, true, 10); 
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


void setup_dma_adc(){

    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // Not using ERR bit
        false    // Do not Shift each sample to 8 bits when pushing to FIFO
    );

    adc_set_clkdiv(10); // 50khz


    sleep_ms(1000);
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    dma_chan_adc_data = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan_adc_data);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan_adc_data, &cfg,
        fun_wave,    // dst
        &adc_hw->fifo,  // src
        NUM_WAV_SAMPLES,  // transfer count
        true            // start immediately
    );

    printf("Starting capture\n");
    adc_run(true);
}

void setup_dma_dac()
{

    dma_chan_dac_data = dma_claim_unused_channel(true); 
    dma_chan_dac_loop = dma_claim_unused_channel(true);

    dma_channel_config c_data = dma_channel_get_default_config(dma_chan_dac_data);
    dma_channel_config c_loop = dma_channel_get_default_config(dma_chan_dac_loop);

    uint pio_r2r_dreq = pio_get_dreq(pio_r2r, sm_r2r, true);
      
    channel_config_set_read_increment(&c_data, true);
    channel_config_set_write_increment(&c_data, false);
    channel_config_set_dreq(&c_data, pio_r2r_dreq);
    channel_config_set_chain_to(&c_data, dma_chan_dac_loop);
    channel_config_set_transfer_data_size(&c_data, DMA_SIZE_16);
    channel_config_set_irq_quiet(&c_data, false);
    dma_channel_configure(
        dma_chan_dac_data, 
        &c_data, 
        &pio1_hw->txf[sm_r2r], 
        NULL, // loop channel fills this in
        NUM_WAV_SAMPLES, 
        false
    );

    channel_config_set_read_increment(&c_loop, false);
    channel_config_set_write_increment(&c_loop, false);
    channel_config_set_chain_to(&c_loop, dma_chan_dac_data);
    channel_config_set_transfer_data_size(&c_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_loop, false);
    dma_channel_configure(
        dma_chan_dac_loop, 
        &c_loop, 
        &dma_hw->ch[dma_chan_dac_data].read_addr, 
        &p_fun_wave,
        1, 
        true
    );
}
