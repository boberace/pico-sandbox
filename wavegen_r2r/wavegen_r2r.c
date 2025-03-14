#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "math.h"
#include "quadrature_encoder.pio.h"
#include "r2r.pio.h"

#define system_frequency clock_get_hz(clk_sys)

const uint PIN_AB_ENC1 = 2; // 3 also used
const uint PIN_BUT_ENC1 = 4;
const uint PIN_BUT_ENC2 = 5;
const uint PIN_AB_ENC2 = 6; // 7 also used

#define R2R_BITS 6
#define R2R_BASE_PIN 10 // 11-15 also used
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define NUM_WAV_SAMPLES 125

static char event_str[128];

PIO pio_quadenc = pio0;
uint sm_quadenc1 = 0;
uint sm_quadenc2 = 1;

PIO pio_r2r = pio1;
uint sm_r2r = 0;

uint8_t fun_wave[NUM_WAV_SAMPLES];

int dma_chan_wave_bytes;
int dma_chan_wave_loop;

void gen_waves();
void r2r_forever(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq);
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq);
void setup_dma_wave();
void dma_handler_wave();
void set_frequency(float freq);
static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate);
static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm);
void gpio_event_string(char *buf, uint32_t events);
void gpio_callback(uint gpio, uint32_t events);

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    int new_value_enc1, delta_enc1, old_value_enc1 = 0;
    int last_value_enc1 = -1, last_delta_enc1 = -1;
    int new_value_enc2, delta_enc2, old_value_enc2 = 0;
    int last_value_enc2 = -1, last_delta_enc2 = -1;    

    // we don't really need to keep the offset, as this program must be loaded
    // at offset 0
    pio_add_program(pio_quadenc, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_quadenc, sm_quadenc1, PIN_AB_ENC1, 0);
    quadrature_encoder_program_init(pio_quadenc, sm_quadenc2, PIN_AB_ENC2, 0);

    gen_waves();    

    uint offset_r2r = pio_add_program(pio_r2r, &r2r_program);
    r2r_forever(pio_r2r, sm_r2r, offset_r2r, R2R_BASE_PIN, R2R_BITS, 1*440.0f);
    printf("Loaded r2r program at %d\n", offset_r2r);  
    
    for (int i = 0; i < NUM_WAV_SAMPLES; i++) {
        uint8_t value = fun_wave[i];  
        printf("%d\n", value);  
    }

    setup_dma_wave();

    gpio_init(PIN_BUT_ENC1);
    gpio_pull_up(PIN_BUT_ENC1);
    gpio_set_irq_enabled_with_callback(PIN_BUT_ENC1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_init(PIN_BUT_ENC2);
    gpio_pull_up(PIN_BUT_ENC2);
    gpio_set_irq_enabled_with_callback(PIN_BUT_ENC2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    uint count = 0;
    while (true) {
        count++;
        sleep_ms(100);

        new_value_enc1 = quadrature_encoder_get_count(pio_quadenc, sm_quadenc1);
        delta_enc1 = new_value_enc1 - old_value_enc1;
        old_value_enc1 = new_value_enc1;

        new_value_enc2 = quadrature_encoder_get_count(pio_quadenc, sm_quadenc2);
        delta_enc2 = new_value_enc2 - old_value_enc2;
        old_value_enc2 = new_value_enc2;

        if (new_value_enc1 != last_value_enc1 || delta_enc1 != last_delta_enc1 ) {
            printf("enc1 position %8d, delta %6d\n", new_value_enc1, delta_enc1);
            last_value_enc1 = new_value_enc1;
            last_delta_enc1 = delta_enc1;
        }

        if (new_value_enc2 != last_value_enc2 || delta_enc2 != last_delta_enc2 ) {
            printf("enc2 position %8d, delta %6d\n", new_value_enc2, delta_enc2);
            last_value_enc2 = new_value_enc2;
            last_delta_enc2 = delta_enc2;
        }

    }
}

void gen_waves() 
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

void r2r_forever(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq) 
{
    r2r_program_init(pio, sm, offset, base_pin, num_pins, freq);  
    pio_sm_set_enabled(pio, sm, true);

}

void setup_dma_wave()
{

    dma_chan_wave_bytes = 0; 
    dma_chan_wave_loop = 1;

    dma_channel_config c_bytes = dma_channel_get_default_config(dma_chan_wave_bytes);
    dma_channel_config c_loop = dma_channel_get_default_config(dma_chan_wave_loop);

    // uint8_t * p_fun_wave = fun_wave;
    // channel_config_set_read_increment(&c_loop, false);
    // channel_config_set_chain_to(&c_loop, dma_chan_wave_bytes);
    // channel_config_set_irq_quiet(&c_loop, true);
    // dma_channel_configure(
    //     dma_chan_wave_loop, 
    //     &c_loop, 
    //     &dma_hw->ch[dma_chan_wave_bytes].read_addr, 
    //     &p_fun_wave, 
    //     1, 
    //     false
    // );
      
    channel_config_set_dreq(&c_bytes, pio_get_dreq(pio_r2r, sm_r2r, true));
    // channel_config_set_chain_to(&c_bytes, dma_chan_wave_loop);
    channel_config_set_transfer_data_size(&c_bytes, DMA_SIZE_8);
    channel_config_set_irq_quiet(&c_bytes, false);
    dma_channel_configure(
        dma_chan_wave_bytes, 
        &c_bytes, 
        &pio1_hw->txf[sm_r2r], 
        fun_wave,
        NUM_WAV_SAMPLES, 
        false
    );
    dma_channel_set_irq0_enabled(dma_chan_wave_bytes, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler_wave);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_chan_wave_bytes);

}

void dma_handler_wave() 
{
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan_wave_bytes;
    dma_channel_set_read_addr(dma_chan_wave_bytes, &fun_wave[0], true);
    
}

void set_frequency(float freq) 
{
    pio_sm_set_clkdiv_int_frac(pio_r2r, sm_r2r, system_frequency/NUM_WAV_SAMPLES/freq, 0);
}   


// max_step_rate is used to lower the clock of the state machine to save power
// if the application doesn't require a very high sampling rate. Passing zero
// will set the clock to the maximum

static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate)
{
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, false);
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin + 1);

    gpio_pull_up(pin);
    gpio_pull_up(pin + 1);

    pio_sm_config c = quadrature_encoder_program_get_default_config(0);

    sm_config_set_in_pins(&c, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c, pin); // for JMP
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    // passing "0" as the sample frequency,
    if (max_step_rate == 0) {
        sm_config_set_clkdiv(&c, 1.0);
    } else {
        // one state machine loop takes at most 10 cycles
        float div = (float)clock_get_hz(clk_sys) / (10 * max_step_rate);
        sm_config_set_clkdiv(&c, div);
    }

    pio_sm_init(pio, sm, 0, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm)
{
    uint ret;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
    while (n > 0) {
        ret = pio_sm_get_blocking(pio, sm);
        n--;
    }
    return ret;
}

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) 
{
for (uint i = 0; i < 4; i++) {
    uint mask = (1 << i);
    if (events & mask) {
        // Copy this event string into the user string
        const char *event_str = gpio_irq_str[i];
        while (*event_str != '\0') {
            *buf++ = *event_str++;
        }
        events &= ~mask;

        // If more events add ", "
        if (events) {
            *buf++ = ',';
            *buf++ = ' ';
        }
    }
}
*buf++ = '\0';
}

void gpio_callback(uint gpio, uint32_t events) 
{
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
}
