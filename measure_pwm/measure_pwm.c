/**
 * todo : create formula for enum dreq_num_rp2350 so any pio or state machine can be used
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "measure_pwm.pio.h"


#define PIN_CAPTURE 17
#define PIN_PWM_A 20
#define PIN_PWM_B 21

// start DMA configuration
// configure with bits so we can align the buffer
#define PIN_CAP_BUF_BITS 4 // min 2 
#define PIN_CAP_DMA_SIZE 2 // (0 = 8bit/1byte, 1 = 16bit/2byte, 2 = 32bit/4byte) no 64bit/8byte for DMA - enum dma_channel_transfer_size

#define CAPTURE_RING_BITS (PIN_CAP_BUF_BITS + PIN_CAP_DMA_SIZE) // max 15
#define PIN_CAP_BUF_SIZE (1 << PIN_CAP_BUF_BITS) // number of buffer samples
#define PIN_CAP_BYTES_SIZE (1 << CAPTURE_RING_BITS)  // how many bytes the buffer takes up

uint32_t pin_cap_buf[PIN_CAP_BUF_SIZE] __attribute__((aligned(PIN_CAP_BYTES_SIZE))); // must align for circular DMA
// end DMA configuration

uint dma_data_cptr_chan = 0;

void measure_pwm_forever(uint pin, uint32_t *buf);
void measure_pwm_program_init(PIO pio, uint sm, uint offset, uint pin);

int main() {
    
    stdio_init_all();

    gpio_set_function(PIN_PWM_A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_B, GPIO_FUNC_PWM);
    uint16_t wrap = 20000;
    uint slice_num = pwm_gpio_to_slice_num(PIN_PWM_A);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 2000);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 5);
    pwm_set_clkdiv_int_frac (slice_num, 125, 0); // microseconds
    pwm_set_enabled(slice_num, true);   
    
    measure_pwm_forever(PIN_CAPTURE, pin_cap_buf);

    dma_channel_start(dma_data_cptr_chan); 
    sleep_ms(1000);

    while (1) {

        printf("--------------------\n");   

        for(int i = 0; i < PIN_CAP_BUF_SIZE; i++) {
            printf("tics %8d\n", pin_cap_buf[i]);
        }

        sleep_ms(1000);
    }
}

void measure_pwm_forever(uint pin, uint32_t *buf) {

    PIO pio = pio0;   
    uint sm = 0; 

    uint offset = pio_add_program(pio, &measure_pwm_program);

    measure_pwm_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    
    dma_channel_config data_capture_cfg = dma_channel_get_default_config(dma_data_cptr_chan);

    channel_config_set_transfer_data_size(&data_capture_cfg, PIN_CAP_DMA_SIZE);
    channel_config_set_read_increment(&data_capture_cfg, false);
    channel_config_set_write_increment(&data_capture_cfg, true);
    channel_config_set_dreq(&data_capture_cfg, DREQ_PIO0_RX0); // why +sm in SD Card example?
    channel_config_set_ring(&data_capture_cfg, true, CAPTURE_RING_BITS);
    dma_channel_configure(
        dma_data_cptr_chan, 
        &data_capture_cfg, 
        buf,        
        &pio->rxf[sm], 
        0xFFFFFFFF, // maximum amount of data to transfer before resetting 
        false
    );
}

void measure_pwm_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = measure_pwm_program_get_default_config(offset);

    pio_gpio_init(pio, pin);
    sm_config_set_in_pins(&c, pin);
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE); 
    // set up to create microsecond tics
    float div = (float)clock_get_hz(clk_sys) / 8 / 1000000; 
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
