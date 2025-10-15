#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "r2r.pio.h"


#define system_frequency clock_get_hz(clk_sys)

#define CAPTURE_CHANNEL 0 // ADC0 is GPIO26

#define R2R_BITS 8
#define PIN_R2R_BASE 0 
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define R2R_MID (1 << (R2R_BITS-1))
#define NUM_LOOP_SAMPLES (1 << 17)
#define PIN_DAC_SYNC 16
#define DAC_FREQ 24000.0

#define NUM_SINE_SAMPLES (1 << 8) // 8 bit so overflow wraps around
int8_t sine_wave[NUM_SINE_SAMPLES];

const float osc_freq = 150.0f; // in Hz
uint32_t osc_phase_accum = 0;
uint32_t osc_phase_incr = ((1ULL << 32) * (osc_freq ) / DAC_FREQ); // phase increment per sample

PIO pio_r2r = pio0;
uint sm_r2r = 0;

// volatile uint16_t fun_wave[NUM_LOOP_SAMPLES] = {0};
volatile uint8_t dac_wave[NUM_LOOP_SAMPLES] = {0};
volatile uint8_t * p_dac_wave = &dac_wave[0];

volatile uint8_t icounter = 0;

void setup_waves();
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, uint sync_pin, float freq);
void setup_r2r();


void pio_irq_handler() {
    
    if (pio_interrupt_get(pio0, 0)) { // Check if IRQ 0 from PIO0 triggered
        osc_phase_accum += osc_phase_incr;
        uint8_t index = (osc_phase_accum >> 24) & 0xFF;
        uint8_t sample = sine_wave[index] + R2R_MID;
        pio_sm_put(pio_r2r, sm_r2r, sample);
        icounter++;
        pio_interrupt_clear(pio0, 0); // Clear the interrupt
    }
}

int main()
{
    setup_waves();
    stdio_init_all();
    setup_r2r();    

    uint count = 0;

    while (true) {
        printf("%d, system freq %d\n", count, system_frequency);
        count++;
        sleep_ms(1000);
    }
}

void setup_waves() 
{ 
    for (int i = 0; i < NUM_SINE_SAMPLES; i++) {
        sine_wave[i] = R2R_MAX * (sin(i * 2 * M_PI / NUM_SINE_SAMPLES) / 2);
    }
}

void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, uint sync_pin, float freq) 
{
    pio_sm_config c = r2r_program_get_default_config(offset);

    pio_gpio_init(pio, sync_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, sync_pin, 1, true); 
    sm_config_set_sideset_pins(&c, sync_pin);

    sm_config_set_out_pins(&c, base_pin, num_pins);        
    for(int i = 0; i < num_pins; i++) {
        pio_gpio_init(pio, base_pin + i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, base_pin, num_pins, true);

    sm_config_set_out_shift(&c, true, true, 8);

    float div = system_frequency / (freq * 2); // factor of 2 because we pull twice per sample (once to load OSR, once to shift out)
    sm_config_set_clkdiv(&c, div);
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true); // 
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio_r2r, sm_r2r, true);
    
}

void setup_r2r() 
{
    uint offset_r2r = pio_add_program(pio_r2r, &r2r_program);
    printf("Loaded r2r program at %d\n", offset_r2r);     
    r2r_program_init(pio_r2r, sm_r2r, offset_r2r, PIN_R2R_BASE, R2R_BITS, PIN_DAC_SYNC, DAC_FREQ);  

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);    
    
    pio_sm_put(pio_r2r, sm_r2r, 0xffffffff);

}


