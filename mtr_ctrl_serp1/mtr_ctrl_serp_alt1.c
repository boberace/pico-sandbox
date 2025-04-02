#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "blink.pio.h"
#include "pin_watch.pio.h"
#include "coil.pio.h"

#define PIN_SPD_SEN 16
#define PIN_SPD_LED 17

#define PIN_CA1 19
#define PIN_CA2 18
#define PIN_MTR_ENA 22

#define PIN_HALL_LCEN 21 
#define PIN_HALL_RBTW 20 

#define PIN_LED_LCEN 2
#define PIN_LED_RBTW 3
#define PIN_LED 25

#define NUM_STATOR_POLES 12

#define FSYS clock_get_hz(clk_sys)

PIO pio_pin_watch = pio1;
uint sm_pin_watch = 0;

// PIO pio_blink = pio0;
// uint sm_blink = 1;

PIO pio_pin_tic_counter = pio0;   
uint sm_hall_lcen_tic_counter = 0; 
uint sm_hall_rbtw_tic_counter = 2;

uint hall_lcen_tic_counter_rise = 1;
uint hall_lcen_tic_counter_fall = 2;
uint hall_rbtw_tic_counter_rise = 3;
uint hall_rbtw_tic_counter_fall = 4;

volatile bool hall_flag = false;

bool clockwise = true;
bool hall_lcen_initial = false;
bool hall_rbtw_initial = false;
bool hall_lcen = false;
bool hall_rbtw = false;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin, float freq);
void setup_motor();
void coil_A(bool a, bool b);
void pin_watch_program_init(PIO pio, uint sm, uint offset, uint pin_watch, uint pin_led);

void gpio_callback(uint gpio, uint32_t events) {
    if(gpio == PIN_HALL_LCEN) {
        if(events & GPIO_IRQ_EDGE_RISE) {
            // hall_lcen_tic_counter_rise = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_lcen_tic_counter);
            hall_flag = true;
            gpio_put(PIN_LED_LCEN, 1);
        } else if(events & GPIO_IRQ_EDGE_FALL) {
            // hall_lcen_tic_counter_fall = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_lcen_tic_counter);
            hall_flag = false;
            gpio_put(PIN_LED_LCEN, 0);
        }
    } else if(gpio == PIN_HALL_RBTW) {
        if(events & GPIO_IRQ_EDGE_RISE) {
            // hall_rbtw_tic_counter_rise = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_rbtw_tic_counter);
            hall_flag = true;
            gpio_put(PIN_LED_RBTW, 1);
            coil_A(!clockwise, clockwise);
        } else if(events & GPIO_IRQ_EDGE_FALL) {
            // hall_rbtw_tic_counter_fall = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_rbtw_tic_counter);
            hall_flag = false;
            gpio_put(PIN_LED_RBTW, 0);
            coil_A(clockwise, !clockwise);            
        }
    }
}


int main()
{
    stdio_init_all();

    sleep_ms(1000); 

   // offset not used as we have to load the program at address 0
   uint offset = pio_add_program(pio_pin_watch, &pin_watch_program);
   pin_watch_program_init(pio_pin_watch, sm_pin_watch, 0, PIN_SPD_SEN, PIN_SPD_LED); 

 
    // uint offset = pio_add_program(pio_blink, &blink_program);
    // blink_program_init(pio_blink, sm_blink, offset, PIN_LED, 1.0f); // 1Hz    

    setup_motor();           

    gpio_init(PIN_HALL_LCEN);
    gpio_set_irq_enabled_with_callback(PIN_HALL_LCEN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(PIN_HALL_RBTW);
    gpio_set_irq_enabled_with_callback(PIN_HALL_RBTW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);


    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());   

    while (true) {

        curr_millis_display = to_ms_since_boot(get_absolute_time());
        if( curr_millis_display - prev_millis_display > 1000){
            prev_millis_display = curr_millis_display;            
            printf("lcen: rise %d, fall %d <> rbtw: rise %d, fall %d\n", hall_lcen_tic_counter_rise, hall_lcen_tic_counter_fall, hall_rbtw_tic_counter_rise, hall_rbtw_tic_counter_fall);
        }
    }

    pio_sm_exec(pio_pin_watch, sm_pin_watch, pio_encode_jmp(0)); // stop the program

}

void blink_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) 
{
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = blink_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin, 1);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
 }


void setup_motor() {
    gpio_init(PIN_CA1);
    gpio_set_dir(PIN_CA1, GPIO_OUT);
    gpio_put(PIN_CA1, 0);

    gpio_init(PIN_CA2);
    gpio_set_dir(PIN_CA2, GPIO_OUT);
    gpio_put(PIN_CA2, 0);

    gpio_init(PIN_LED_LCEN);
    gpio_set_dir(PIN_LED_LCEN, GPIO_OUT);
    gpio_put(PIN_LED_LCEN, 0);

    gpio_init(PIN_LED_RBTW);
    gpio_set_dir(PIN_LED_RBTW, GPIO_OUT);
    gpio_put(PIN_LED_RBTW, 0);

    gpio_init(PIN_MTR_ENA);
    gpio_set_dir(PIN_MTR_ENA, GPIO_OUT);
    gpio_put(PIN_MTR_ENA, 1);

}
void coil_A(bool a, bool b){
    uint32_t mask = (1 << PIN_CA1) | (1 << PIN_CA2); // Mask for PIN_CA1 and PIN_CA2
    uint32_t value = (a << PIN_CA1) | (b << PIN_CA2); // Set values for PIN_CA1 and PIN_CA2
    gpio_put_masked(mask, value);
}

void pin_watch_program_init(PIO pio, uint sm, uint offset, uint pin_watch, uint pin_led) 
{
    // the code must be loaded at address 0, because it uses computed jumps 

    pio_gpio_init(pio, pin_watch);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_watch, 1, false);
    gpio_pull_up(pin_watch);

    pio_gpio_init(pio, pin_led);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_led, 1, true);  
    
    pio_sm_config c = pin_watch_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin_watch);
    sm_config_set_set_pins(&c, pin_led, 1);
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE); 

    float clkdiv = ((float)FSYS / 1000000.0); 
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    
}

void coil_program_init(PIO pio, uint sm, uint offset, float freq, uint base_pin) {
    pio_sm_config c = coil_program_get_default_config(offset);
 
    sm_config_set_out_pins(&c, base_pin , 2);
 
    pio_sm_set_consecutive_pindirs(pio, sm, base_pin, 2, false);
 
    pio_gpio_init(pio, base_pin);
    pio_gpio_init(pio, base_pin + 1);
 
    sm_config_set_out_shift(&c, false, true, 32); 
 
    float clkdiv = ((float)FSYS / 1000000.0); 
    sm_config_set_clkdiv(&c, clkdiv);
 
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
 }