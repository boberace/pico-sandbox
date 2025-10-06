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
#include "adc_sync.pio.h"

#define system_frequency clock_get_hz(clk_sys)

#define CAPTURE_CHANNEL 0 // ADC0 is GPIO26

#define R2R_BITS 8
#define PIN_R2R_BASE 0 
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define NUM_LOOP_SAMPLES (1 << 17)
#define PIN_ADC_SYNC 15
#define ADC_FREQ 24000  // 1MHz ADC sampling rate

int dma_chan_dac_data, dma_chan_dac_loop;
int dma_chan_adc_data, dma_chan_adc_loop;
int dma_chan_sync_data, dma_chan_sync_loop;

PIO pio_adc_sync = pio0;
uint sm_adc_sync = 0;

PIO pio_r2r = pio1;
uint sm_r2r = 0;

volatile uint8_t fun_wave[NUM_LOOP_SAMPLES];
volatile uint8_t * p_fun_wave = &fun_wave[0];

volatile uint8_t toggle[] = {0, 0xff};
volatile uint8_t * p_toggle = &toggle[0];

void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins);
void setup_r2r();
void setup_sampling_loop();
void adc_sync_program_init(PIO pio, uint sm, uint offset, uint pin);
void setup_adc_sync();
// void dma_adc_handler();
// void dma_dac_handler();

// void __not_in_flash_func(adc_fifo_isr)() {
// void adc_fifo_isr() {
//     uint16_t raw_value = adc_fifo_get();

//     latest_adc_reading = raw_value;    

//     pio_r2r->txf[sm_r2r] = raw_value; 

//     gpio_put(PIN_LED_A, 1);
//     asm volatile("nop; nop; nop; nop; nop;nop; nop; nop; nop; nop;");
//     gpio_put(PIN_LED_A, 0); 

//     sample_count++;

// }

int main()
{
    stdio_init_all();

    // gpio_init(PIN_ADC_SYNC);
    // gpio_set_dir(PIN_ADC_SYNC, GPIO_OUT);

    setup_r2r();
    setup_adc_sync();
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

void adc_sync_program_init(PIO pio, uint sm, uint offset, uint pin) 
{
    pio_sm_config c = adc_sync_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pin, 1); 
    sm_config_set_out_shift(&c, false, true, 1); 
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_init(pio, sm, offset, &c);
}

void setup_adc_sync() 
{
    uint offset_adc_sync = pio_add_program(pio_adc_sync, &adc_sync_program);
    printf("Loaded adc_sync program at %d\n", offset_adc_sync);     
    adc_sync_program_init(pio_adc_sync, sm_adc_sync, offset_adc_sync, PIN_ADC_SYNC);  
    pio_sm_set_enabled(pio_adc_sync, sm_adc_sync, true);

}

void setup_sampling_loop()
{

    // ADC setup

    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_set_clkdiv(round(48000000 / (float)ADC_FREQ) - 1); 
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
        NUM_LOOP_SAMPLES, // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    channel_config_set_read_increment(&c_adc_loop, false);
    channel_config_set_write_increment(&c_adc_loop, false);
    channel_config_set_chain_to(&c_adc_loop, dma_chan_adc_data);
    channel_config_set_transfer_data_size(&c_adc_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_adc_loop, true);
    dma_channel_configure(
        dma_chan_adc_loop, // channel – DMA channel
        &c_adc_loop, // config – Pointer to DMA config structure
        &dma_hw->ch[dma_chan_adc_data].write_addr,  // write_addr – Initial write address
        &p_fun_wave, // read_addr – Initial read address 
        1,  // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );  
    
    // DAC DMA setup

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
        &p_fun_wave, // read_addr – Initial read address 
        1,  // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    // sync pulse

    dma_chan_sync_data = dma_claim_unused_channel(true);  
    dma_chan_sync_loop = dma_claim_unused_channel(true);

    dma_channel_config c_sync_data = dma_channel_get_default_config(dma_chan_sync_data);
    dma_channel_config c_sync_loop = dma_channel_get_default_config(dma_chan_sync_loop);
      
    channel_config_set_read_increment(&c_sync_data, true);
    channel_config_set_write_increment(&c_sync_data, false);
    channel_config_set_dreq(&c_sync_data, DREQ_ADC); 
    channel_config_set_chain_to(&c_sync_data, dma_chan_sync_loop);
    channel_config_set_transfer_data_size(&c_sync_data, DMA_SIZE_8);
    channel_config_set_irq_quiet(&c_sync_data, false);
    dma_channel_configure(
        dma_chan_sync_data, // channel – DMA channel
        &c_sync_data, // config – Pointer to DMA config structure
        &pio_adc_sync->txf[sm_adc_sync],  // write_addr – Initial write address
        NULL, // read_addr – Initial read address - loop channel fills this in
        2,  // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    channel_config_set_read_increment(&c_sync_loop, false);
    channel_config_set_write_increment(&c_sync_loop, false);
    channel_config_set_chain_to(&c_sync_loop, dma_chan_sync_data);
    channel_config_set_transfer_data_size(&c_sync_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_sync_loop, true);
    dma_channel_configure(
        dma_chan_sync_loop, // channel – DMA channel
        &c_sync_loop, // config – Pointer to DMA config structure
        &dma_hw->ch[dma_chan_sync_data].read_addr,  // write_addr – Initial write address
        &p_toggle, // read_addr – Initial read address 
        1,  // encoded_transfer_count – The encoded transfer count
        false // trigger – True to start the transfer immediately
    );

    printf("Starting capture\n");
    adc_run(true);

    dma_start_channel_mask(1u << dma_chan_adc_loop);    //dma_chan_dac_loop
    sleep_us(1);
    dma_start_channel_mask(1u << dma_chan_sync_loop);
    sleep_us(90);
    dma_start_channel_mask(1u << dma_chan_dac_loop);   //dma_chan_adc_loop       

}

