#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "math.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

#include "arm_math.h"

#include "r2r.pio.h"

#define system_frequency clock_get_hz(clk_sys)

#define CAPTURE_CHANNEL 2 // ADC 0,1,2 is GPIO 26,27,28

#define R2R_BITS 8
#define PIN_R2R_BASE 0 
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define NUM_FRAME_SAMPLES 256
#define NUM_FRAMES 1024
uint frame_index = 0;
#define NUM_LOOP_SAMPLES NUM_FRAME_SAMPLES * NUM_FRAMES
#define ADC_FREQ 24000  

#define PIN_TEST 15
#define TOGGLE_PIN(a) gpio_xor_mask(1u << a)
#define SET_PIN(a) gpio_put(a, 1)
#define CLR_PIN(a) gpio_put(a, 0)

bool dac_dma_started = false;

int dma_chan_adc_data, dma_chan_adc_loop;
int dma_chan_dac_data, dma_chan_dac_loop;

uint8_t frame_wave[NUM_FRAME_SAMPLES];
uint8_t * p_frame_wave = &frame_wave[0];
uint8_t loop_wave[NUM_LOOP_SAMPLES];
uint8_t * p_loop_wave = &loop_wave[0];

PIO pio_r2r = pio1;
uint sm_r2r = 0;

void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins);
void setup_r2r();
void setup_sampling_loop();
void adc_dma_handler();

int main()
{
    stdio_init_all();

    gpio_init(PIN_TEST);
    gpio_set_dir(PIN_TEST, GPIO_OUT);

    setup_r2r();
    setup_sampling_loop();
    uint count = 0;

    while (true) {
        printf("%d, system freq %d\n", count, system_frequency);
        count++;
        sleep_ms(1000);
    }
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

void setup_sampling_loop()
{

    // ADC setup

    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_set_clkdiv(round(48000000 / (float)ADC_FREQ) - 1); // actual tick period - 1 of 48MHz ADC clock
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // Not using ERR bit
        true    // Shift each sample to 8 bits when pushing to FIFO
    );
  
    // ADC DMA setup

    dma_chan_adc_data = dma_claim_unused_channel(true); 
    dma_chan_adc_loop = dma_claim_unused_channel(true);

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
        NUM_FRAME_SAMPLES, // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    dma_channel_set_irq0_enabled(dma_chan_adc_data, true);
    irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    channel_config_set_read_increment(&c_adc_loop, false);
    channel_config_set_write_increment(&c_adc_loop, false);
    channel_config_set_chain_to(&c_adc_loop, dma_chan_adc_data);
    channel_config_set_transfer_data_size(&c_adc_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_adc_loop, true);
    dma_channel_configure(
        dma_chan_adc_loop, // channel – DMA channel
        &c_adc_loop, // config – Pointer to DMA config structure
        &dma_hw->ch[dma_chan_adc_data].write_addr,  // write_addr – Initial write address
        &p_frame_wave, // read_addr – Initial read address 
        1,  // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );  

    
    dma_chan_dac_data = dma_claim_unused_channel(true); 
    dma_chan_dac_loop = dma_claim_unused_channel(true);

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
        &pio_r2r->txf[sm_r2r],  // write_addr – Initial write address
        NULL, // read_addr – Initial read address - loop channel fills this in
        NUM_LOOP_SAMPLES,  // encoded_transfer_count – The encoded transfer count NUM_LOOP_SAMPLES
        false // trigger – True to start the transfer immediately
    );

    channel_config_set_read_increment(&c_dac_loop, false);
    channel_config_set_write_increment(&c_dac_loop, false);
    channel_config_set_chain_to(&c_dac_loop, dma_chan_dac_data);
    channel_config_set_transfer_data_size(&c_dac_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_dac_loop, true);
    dma_channel_configure(
        dma_chan_dac_loop, // channel – DMA channel
        &c_dac_loop, // config – Pointer to DMA config structure
        &dma_hw->ch[dma_chan_dac_data].read_addr,  // write_addr – Initial write address
        &p_loop_wave, // read_addr – Initial read address 
        1,  // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    
    printf("Starting capture\n");
    adc_run(true);
    dma_start_channel_mask(1u << dma_chan_adc_loop); 
    printf("Capture started\n");

}

void adc_dma_handler() {
    // Check if the DMA channel triggered the interrupt
    if (dma_hw->ints0 & (1u << dma_chan_adc_data)) {
        // Acknowledge the interrupt by clearing the flag
        dma_hw->ints0 = (1u << dma_chan_adc_data);
        SET_PIN(PIN_TEST);
        // TOGGLE_PIN(PIN_TEST);
        memcpy(loop_wave + frame_index * NUM_FRAME_SAMPLES, frame_wave, NUM_FRAME_SAMPLES);
        frame_index++;
        frame_index%=NUM_FRAMES;

        //processing must be deterministic so subsequent frames align

        CLR_PIN(PIN_TEST);
        if (!dac_dma_started) {
            dma_start_channel_mask(1u << dma_chan_dac_loop);
            dac_dma_started = true;
        }      
    }

}

