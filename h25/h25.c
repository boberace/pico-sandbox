#include <stdio.h>
#include "pico/stdlib.h"

#define WATCHED_PINS_MASK 0x01FFFFFF // Mask for GPIO pins 0 through 24
#define NUM_STATES 64
// void gpio_callback(uint gpio, uint32_t events) {
//     // Read the state of GPIO pins 0 through 24
//     // uint32_t gpio_states = gpio_get_all() & WATCHED_PINS_MASK;

//     // Print the state of the GPIO pins
//     // printf("GPIO state changed! GPIO[0-24]: 0x%08X\n", gpio_states);
//     // printf("GPIO %d changed! Events: %u\n", gpio, events);
//     printf("%d \n", gpio);
// }

int num_positions = 50;
uint32_t positions[]={11184811,	11184809,	11184813,	11184805,	11184821,	11184789,	11184853,	11184725,	11184981,	11184469,	11185493,	11183445,	11187541,	11179349,	11195733,	11162965,	11228501,	11097429,	11359573,	10835285,	11883861,	9786709,	13981013,	5592405,	22369621,	22369620,	22369622,	22369618,	22369626,	22369610,	22369642,	22369578,	22369706,	22369450,	22369962,	22368938,	22370986,	22366890,	22375082,	22358698,	22391466,	22325930,	22457002,	22194858,	22719146,	21670570,	23767722,	19573418,	27962026,	11184810};

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

    printf("Hello, world!\n");
    uint32_t gpio_state_prev =0;
    uint32_t gpio_states[NUM_STATES] = {0};
    uint32_t gpio_states_index = 0;
    int position;
    int postion_prev = -1;
    while (true) {

        uint32_t gpio_state = gpio_get_all() & WATCHED_PINS_MASK;
        
        if (gpio_state != gpio_state_prev) {
            gpio_states[gpio_states_index++] = gpio_state;
            gpio_state_prev = gpio_state;   
            uint8_t pop_count;
            uint8_t pop_count_lowest = 0xff;
            for(int p=0; p<num_positions; p++) {
                uint32_t pop_test = gpio_state ^ positions[p];
                pop_count = __builtin_popcount(pop_test);
                if(pop_count < pop_count_lowest) {
                    pop_count_lowest = pop_count;
                    position = p;
                }
            }
            printf("%d %d\n", pop_count_lowest, position);
    
        }
    }
}
