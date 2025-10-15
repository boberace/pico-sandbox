#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "pico/stdlib.h"
// #include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"


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

bool dac_dma_started = false;

int dma_chan_adc_data, dma_chan_adc_loop;

volatile uint8_t frame_wave[NUM_FRAME_SAMPLES];
volatile uint8_t * p_frame_wave = &frame_wave[0];
volatile uint8_t loop_wave[NUM_LOOP_SAMPLES];

void setup_sampling_loop();
void adc_dma_handler();

int main()
{
    stdio_init_all();

    gpio_init(PIN_TEST);
    gpio_set_dir(PIN_TEST, GPIO_OUT);

    setup_sampling_loop();
    uint count = 0;

    while (true) {
        printf("%d, system freq %d\n", count, system_frequency);
        count++;
        sleep_ms(1000);
    }
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
    
    printf("Starting capture\n");
    adc_run(true);
    // todo: run DAC of sync pulse to control latency
    dma_start_channel_mask(1u << dma_chan_adc_loop); 
    sleep_us(1);
    printf("Capture started\n");

}

void adc_dma_handler() {
    // Check if the DMA channel triggered the interrupt
    if (dma_hw->ints0 & (1u << dma_chan_adc_data)) {
        // Acknowledge the interrupt by clearing the flag
        dma_hw->ints0 = (1u << dma_chan_adc_data);

        TOGGLE_PIN(PIN_TEST);
        memcpy(loop_wave[frame_index * NUM_FRAME_SAMPLES], frame_wave[0], NUM_FRAME_SAMPLES);
        frame_index++;
        frame_index%=NUM_FRAMES;

        //processing must be deterministic so subsequent frames align

        if (!dac_dma_started) {
            // start DAC DMA here
            dac_dma_started = true;
        }      
    }
}

