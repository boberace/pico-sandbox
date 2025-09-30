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
#define NUM_LOOP_SAMPLES (1 << 16)

PIO pio_r2r = pio1;
uint sm_r2r = 0;

volatile uint8_t fun_wave[NUM_LOOP_SAMPLES];
volatile uint8_t * p_fun_wave = &fun_wave[0];

void blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins);
void setup_r2r();
void setup_sampling_loop();

int main()
{
    stdio_init_all();

    setup_r2r();
    setup_sampling_loop();

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


void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins) 
{
    pio_sm_config c = r2r_program_get_default_config(offset);
    sm_config_set_out_pins(&c, base_pin, num_pins); 
    sm_config_set_out_shift(&c, false, true, 8); 
    for(int i = 0; i < num_pins; i++) {
        pio_gpio_init(pio, base_pin + i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, base_pin, num_pins, true);
    pio_sm_init(pio, sm, offset, &c);
}

void setup_r2r() 
{
    uint offset_r2r = pio_add_program(pio_r2r, &r2r_program);
    printf("Loaded r2r program at %d\n", offset_r2r);     
    r2r_program_init(pio_r2r, sm_r2r, offset_r2r, PIN_R2R_BASE, R2R_BITS);  
    pio_sm_set_enabled(pio_r2r, sm_r2r, true);

}


void setup_sampling_loop(){

    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // Not using ERR bit
        true    // Shift each sample to 8 bits when pushing to FIFO
    );

    adc_set_clkdiv(48000000/96/50000); // 50khz

    int dma_chan_adc_data = dma_claim_unused_channel(true); 
    int dma_chan_adc_loop = dma_claim_unused_channel(true);

    dma_channel_config c_adc_data = dma_channel_get_default_config(dma_chan_adc_data);
    dma_channel_config c_adc_loop = dma_channel_get_default_config(dma_chan_adc_loop);

      
    channel_config_set_read_increment(&c_adc_data, false);
    channel_config_set_write_increment(&c_adc_data, true);
    channel_config_set_dreq(&c_adc_data, DREQ_ADC);
    channel_config_set_chain_to(&c_adc_data, dma_chan_adc_loop);
    channel_config_set_transfer_data_size(&c_adc_data, DMA_SIZE_8);
    channel_config_set_irq_quiet(&c_adc_data, false);
    dma_channel_configure(
        dma_chan_adc_data, // channel – DMA channel
        &c_adc_data, // config – Pointer to DMA config structure
        NULL, // write_addr – Initial write address - loop channel fills this in
        &adc_hw->fifo, // read_addr – Initial read address 
        NUM_LOOP_SAMPLES, // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    channel_config_set_read_increment(&c_adc_loop, false);
    channel_config_set_write_increment(&c_adc_loop, false);
    channel_config_set_chain_to(&c_adc_loop, dma_chan_adc_data);
    channel_config_set_transfer_data_size(&c_adc_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_adc_loop, false);
    dma_channel_configure(
        dma_chan_adc_loop, // channel – DMA channel
        &c_adc_loop, // config – Pointer to DMA config structure
        &dma_hw->ch[dma_chan_adc_data].write_addr,  // write_addr – Initial write address
        &p_fun_wave, // read_addr – Initial read address 
        1,  // encoded_transfer_count – The encoded transfer count
        true // trigger – True to start the transfer immediately
    );  
    
    printf("Starting capture\n");
    adc_run(true);

    // sleep_ms(1);

    int dma_chan_dac_data = dma_claim_unused_channel(true); 
    int dma_chan_dac_loop = dma_claim_unused_channel(true);

    dma_channel_config c_dac_data = dma_channel_get_default_config(dma_chan_dac_data);
    dma_channel_config c_dac_loop = dma_channel_get_default_config(dma_chan_dac_loop);
  
      
    channel_config_set_read_increment(&c_dac_data, true);
    channel_config_set_write_increment(&c_dac_data, false);
    channel_config_set_dreq(&c_dac_data, DREQ_ADC); 
    channel_config_set_chain_to(&c_dac_data, dma_chan_dac_loop);
    channel_config_set_transfer_data_size(&c_dac_data, DMA_SIZE_8);
    channel_config_set_irq_quiet(&c_dac_data, false);
    dma_channel_configure(
        dma_chan_dac_data, // channel – DMA channel
        &c_dac_data, // config – Pointer to DMA config structure
        &pio1_hw->txf[sm_r2r],  // write_addr – Initial write address
        NULL, // read_addr – Initial read address - loop channel fills this in
        NUM_LOOP_SAMPLES,  // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    channel_config_set_read_increment(&c_dac_loop, false);
    channel_config_set_write_increment(&c_dac_loop, false);
    channel_config_set_chain_to(&c_dac_loop, dma_chan_dac_data);
    channel_config_set_transfer_data_size(&c_dac_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_dac_loop, false);
    dma_channel_configure(
        dma_chan_dac_loop, // channel – DMA channel
        &c_dac_loop, // config – Pointer to DMA config structure
        &dma_hw->ch[dma_chan_dac_data].read_addr,  // write_addr – Initial write address
        &p_fun_wave, // read_addr – Initial read address 
        1,  // encoded_transfer_count – The encoded transfer count
        true // trigger – True to start the transfer immediately
    );


}
