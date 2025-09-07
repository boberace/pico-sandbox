#include <stdio.h>
#include "pico/stdlib.h"

#define WATCHED_PINS_MASK 0x01FFFFFF // Mask for GPIO pins 0 through 24
#define NUM_STATES 128
// void gpio_callback(uint gpio, uint32_t events) {
//     // Read the state of GPIO pins 0 through 24
//     // uint32_t gpio_states = gpio_get_all() & WATCHED_PINS_MASK;

//     // Print the state of the GPIO pins
//     // printf("GPIO state changed! GPIO[0-24]: 0x%08X\n", gpio_states);
//     // printf("GPIO %d changed! Events: %u\n", gpio, events);
//     printf("%d \n", gpio);
// }

int main()
{
    stdio_init_all();
    sleep_ms(1000); // Wait for the serial port to initialize
   // Initialize GPIO pins 0 through 24
    for (int pin = 0; pin <= 24; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN); // Set as input
        gpio_pull_up(pin);         // Enable pull-up resistor
    }

    // Set up interrupts for GPIO pins 0 through 24
    // for (int pin = 0; pin <= 24; pin++) {
    //     gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE , true, &gpio_callback); //GPIO_IRQ_EDGE_FALL
    // }
    printf("Hello, world!\n");
    uint32_t gpio_state_prev =0;
    uint32_t gpio_states[NUM_STATES] = {0};
    uint32_t gpio_states_index = 0;
    while (true) {
        // printf("Hello, world!\n");
        // sleep_ms(1000);`
        // tight_loop_contents();
        uint32_t gpio_state = gpio_get_all() & WATCHED_PINS_MASK;
        if (gpio_state != gpio_state_prev) {
            gpio_states[gpio_states_index++] = gpio_state;
            gpio_state_prev = gpio_state;   
            if (gpio_states_index == NUM_STATES) {
                printf("--------------------------------------------------\n");
                gpio_states_index = 0; // Reset index if it exceeds the array size
                for(int i = 0; i < NUM_STATES; i++) {
                    for(int j = 0; j < 25; j++) {
                        printf("%d", (gpio_states[i] >> j) & 1);                        
                    }
                    gpio_states[i] = 0;
                    printf("\n");
                }
            }            
        }
    }
}
