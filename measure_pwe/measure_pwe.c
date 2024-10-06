/**
 * todo : create formula for enum dreq_num_rp2350 so any pio or state machine can be used
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "measure_pwe.pio.h"
#include "ws281x.pio.h"


#define IS_RGBW false
#define NUM_PIXELS 1
#define WS281X_PIN 15

// start DMA configuration -----------------------------------
#define PIN_CAPTURE 17
// configure with bits so we can align the buffer
#define PIN_CAP_BUF_BITS 6 // min 2 
#define PIN_CAP_DMA_SIZE 2 // (0 = 8bit/1byte, 1 = 16bit/2byte, 2 = 32bit/4byte) no 64bit/8byte for DMA - enum dma_channel_transfer_size

#define CAPTURE_RING_BITS (PIN_CAP_BUF_BITS + PIN_CAP_DMA_SIZE) // max 15
#define PIN_CAP_BUF_SIZE (1 << PIN_CAP_BUF_BITS) // number of buffer samples
#define PIN_CAP_BYTES_SIZE (1 << CAPTURE_RING_BITS)  // how many bytes the buffer takes up

uint32_t pin_cap_buf[PIN_CAP_BUF_SIZE] __attribute__((aligned(PIN_CAP_BYTES_SIZE))); // must align for circular DMA
// end DMA configuration -----------------------------------

uint dma_data_cptr_chan = 0;

// headers
void measure_pwe_forever(uint pin, uint32_t *buf);
static inline void measure_pwe_program_init(PIO pio, uint sm, uint offset, uint pin);
static inline void ws281x_program_init(uint pin, float freq, bool rgbw);
static inline void put_pixel(uint32_t pixel_grb);
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);
static inline uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

int main() {
    
    stdio_init_all();
    
    measure_pwe_forever(PIN_CAPTURE, pin_cap_buf);

    dma_channel_start(dma_data_cptr_chan); 
    sleep_ms(1000);

    ws281x_program_init(WS281X_PIN, 400000, IS_RGBW);
    uint count_c = 0;
    uint count_b = 0;
    while (1) {
        uint32_t c = urgb_u32((count_c==1?0xFF:0), (count_c==0?0xFF:0), (count_c==2?0xFF:0));       

        printf("--------------------\n");   
        printf("c = %d \n",c);
        printf("--------------------\n");   

        put_pixel(c);

        for(int i = 0; i < PIN_CAP_BUF_SIZE; i++) {
            uint32_t buf = pin_cap_buf[i];
            printf("%2d tics %10d\n", count_b, buf);
            count_b++;
            if (buf > 25) {
                count_b = 0;
            }
        }

        count_c++;
        count_c%=4;
        sleep_ms(1000);
    }
}

void measure_pwe_forever(uint pin, uint32_t *buf) {

    PIO pio = pio0;   
    uint sm = 0; 

    uint offset = pio_add_program(pio, &measure_pwe_program);

    measure_pwe_program_init(pio, sm, offset, pin);
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

static inline void measure_pwe_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = measure_pwe_program_get_default_config(offset);

    pio_gpio_init(pio, pin);
    sm_config_set_in_pins(&c, pin);
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE); 
    // set up to create microsecond tics
    float div = (float)clock_get_hz(clk_sys) / 8 / 10000000; 
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void ws281x_program_init(uint pin, float freq, bool rgbw) {
    PIO pio = pio1;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws281x_program);

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    pio_sm_config c = ws281x_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, rgbw ? 32 : 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = ws281x_T1 + ws281x_T2 + ws281x_T3;
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio1, 0, pixel_grb << 8u);
}
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}
static inline uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            ((uint32_t) (w) << 24) |
            (uint32_t) (b);
}