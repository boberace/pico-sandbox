#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#include "blink.pio.h"


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define SPI_PORT spi0
// #define PIN_MISO 16
// #define PIN_CS   17
// #define PIN_SCK  18
// #define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define I2C_PORT i2c0
// #define I2C_SDA 8
// #define I2C_SCL 9

// Data will be copied from src to dst
// const char src[] = "Hello, world! (from DMA)";
// char dst[count_of(src)];


#define PRINTFLAG 123
#define WATCHED_PINS_MASK 0x01FFFFFF // Mask for GPIO pins 0 through 24
#define NUM_STATES 1024 // Number of states recorded

static int core0_rx_val = 0, core1_rx_val = 0;

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

void core0_sio_irq() {
    // Just record the latest entry
    while (multicore_fifo_rvalid())
        core0_rx_val = multicore_fifo_pop_blocking();

    multicore_fifo_clear_irq();
}
void core1_sio_irq() {
    // Just record the latest entry
    while (multicore_fifo_rvalid())
        core1_rx_val = multicore_fifo_pop_blocking();

    multicore_fifo_clear_irq();
}

void core1_entry() {
    sleep_ms(100); // let core 0 interrupt setup finish
    printf("Core 1 started\n");
    // multicore_fifo_clear_irq();
    // irq_set_exclusive_handler(SIO_FIFO_IRQ_NUM(1), core1_sio_irq);
    // irq_set_enabled(SIO_FIFO_IRQ_NUM(1), true);
    
    // Initialize GPIO pins 0 through 24
    for (int pin = 0; pin <= 24; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN); // Set as input
        gpio_pull_up(pin);         // Enable pull-up resistor
    }
    
    uint32_t gpio_state_prev = 0; // Previous GPIO state
    uint state_index = 0; // Current index in the binary states array
    int gpio_states[NUM_STATES] = {0}; // Array to store binary states
    while (true) {
        uint32_t gpio_state = gpio_get_all() & WATCHED_PINS_MASK;
        if (gpio_state != gpio_state_prev) {
            gpio_state_prev = gpio_state; // Update previous state
            // Store the current GPIO state in the array
            gpio_states[state_index++] = gpio_state;
            if (state_index == (NUM_STATES - 1)) {
                state_index = 0; // Reset index if it exceeds the array size
                multicore_fifo_push_blocking(PRINTFLAG);
            }      
        }
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    // SPI initialisation. This example will use SPI at 1MHz.
    // spi_init(SPI_PORT, 1000*1000);
    // gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    // gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    // gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    // gpio_set_dir(PIN_CS, GPIO_OUT);
    // gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 400Khz.
    // i2c_init(I2C_PORT, 400*1000);
    
    // gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // Get a free channel, panic() if there are none
    // int chan = dma_claim_unused_channel(true);
    
    // 8 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.
    
    // dma_channel_config c = dma_channel_get_default_config(chan);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, true);
    
    // dma_channel_configure(
    //     chan,          // Channel to be configured
    //     &c,            // The configuration we just created
    //     dst,           // The initial write address
    //     src,           // The initial read address
    //     count_of(src), // Number of transfers; in this case each is 1 byte.
    //     true           // Start immediately.
    // );
    
    // We could choose to go and do something else whilst the DMA is doing its
    // thing. In this case the processor has nothing else to do, so we just
    // wait for the DMA to finish.
    // dma_channel_wait_for_finish_blocking(chan);
    
    // The DMA has now copied our text from the transmit buffer (src) to the
    // receive buffer (dst), so we can print it out from there.
    // puts(dst);


    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);   
    blink_pin_forever(pio, 0, offset, 25, 5); 

    // multicore_reset_core1();
    // sleep_ms(1000);
    printf("launch core 1\n");
    multicore_launch_core1(core1_entry);

    irq_set_exclusive_handler(SIO_FIFO_IRQ_NUM(0), core0_sio_irq);
    irq_set_enabled(SIO_FIFO_IRQ_NUM(0), true);
    sleep_ms(10);
    
    printf("Starting core0 loop\n");
    while (true) {
        // printf(".");
        // sleep_ms(1000);
        if (core0_rx_val == PRINTFLAG) {
            core0_rx_val = 0;
            printf("Core 0 received print flag\n");
            // for (int i = 0; i < NUM_STATES; i++) {
            //     for (int j = 24; j >= 0; j--) {
            //         printf("%d,", (gpio_states[i] >> j) & 1);
            //     }
            //     printf("\n");
            //     gpio_states[i] = 0; // Reset the state after printing
            // }
        }
    }
}
