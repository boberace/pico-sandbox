#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "blink.pio.h"
#include "pin_tic_counter.pio.h"
#include "pin_watch.pio.h"

#define PIN_SPD_SEN 16
#define PIN_SPD_LED 17

#define PIN_CA1 19
#define PIN_CA2 18
#define PIN_MTR_ENA 22

#define PIN_HALL_LCEN 20 // 15 // on left
#define PIN_HALL_RBTW 21 // 14 // on right

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
uint sm_hall_rbtw_tic_counter = 1;

bool clockwise = true;
bool hall_lcen_initial = false;
bool hall_rbtw_initial = false;
bool hall_lcen = false;
bool hall_rbtw = false;
bool state_A = false;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin, float freq);
void setup_motor();
void coil_A(bool a, bool b);
void pin_tic_counter_program_init(PIO pio, uint sm, uint mon_pin, uint fb_pin);
static inline int32_t pin_tic_counter_get_tics(PIO pio, uint sm);
void pin_watch_program_init(PIO pio, uint sm, uint offset, uint pin_watch, uint pin_led);

int main()
{
    stdio_init_all();

    sleep_ms(1000); 

   // offset not used as we have to load the program at address 0
   uint offset = pio_add_program(pio_pin_watch, &pin_watch_program);
   pin_watch_program_init(pio_pin_watch, sm_pin_watch, 0, PIN_SPD_SEN, PIN_SPD_LED); 


    pio_add_program(pio_pin_tic_counter, &pin_tic_counter_program);    
    pin_tic_counter_program_init(pio_pin_tic_counter, sm_hall_lcen_tic_counter, PIN_HALL_LCEN, PIN_LED_LCEN);
    pin_tic_counter_program_init(pio_pin_tic_counter, sm_hall_rbtw_tic_counter, PIN_HALL_RBTW, PIN_LED_RBTW);

 
    // uint offset = pio_add_program(pio_blink, &blink_program);
    // blink_program_init(pio_blink, sm_blink, offset, PIN_LED, 1.0f); // 1Hz    

    setup_motor();    
        
    hall_lcen = gpio_get(PIN_LED_LCEN);
    hall_rbtw = gpio_get(PIN_LED_RBTW);

    uint hall_lcen_tic_counter = 0; 
    uint hall_rbtw_tic_counter = 0; 

    bool hall_flag = false;

    state_A = hall_rbtw_initial ? !hall_lcen_initial : hall_lcen_initial;

    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());   

    while (true) {
        hall_lcen = gpio_get(PIN_LED_LCEN);
        hall_rbtw = gpio_get(PIN_LED_RBTW);
        if (!clockwise) {
            if (hall_lcen != state_A) {
                state_A = hall_lcen;
                coil_A(!state_A, state_A);
                hall_flag = true;     
            }
        } else { // counter clockwise
            if (hall_lcen == state_A) {
                state_A = !hall_lcen;
                coil_A(state_A, !state_A);
                hall_flag = true;
            }
        }
        if (hall_flag) {
            hall_flag = false;
            hall_lcen_tic_counter = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_lcen_tic_counter);
            hall_rbtw_tic_counter = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_rbtw_tic_counter);                

        }

        curr_millis_display = to_ms_since_boot(get_absolute_time());
        if( curr_millis_display - prev_millis_display > 1000){
            prev_millis_display = curr_millis_display;            
            printf("Hall lcen tics: %d, Hall rbtw tics: %d\n", hall_lcen_tic_counter, hall_rbtw_tic_counter);
        }
    }

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

    gpio_init(PIN_MTR_ENA);
    gpio_set_dir(PIN_MTR_ENA, GPIO_OUT);
    gpio_put(PIN_MTR_ENA, 1);

    gpio_init(PIN_HALL_LCEN);
    gpio_set_dir(PIN_HALL_LCEN, GPIO_IN);

    gpio_init(PIN_HALL_RBTW);
    gpio_set_dir(PIN_HALL_RBTW, GPIO_IN);

    coil_A(0, 0);
    hall_lcen = gpio_get(PIN_HALL_LCEN);
    hall_rbtw = gpio_get(PIN_HALL_RBTW);

}
void coil_A(bool a, bool b){
    gpio_put(PIN_CA1, a);
    gpio_put(PIN_CA2, b);
}


void pin_tic_counter_program_init(PIO pio, uint sm, uint mon_pin, uint fb_pin) {
    // the code must be loaded at address 0, because it uses computed jumps      

    pio_gpio_init(pio, fb_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, fb_pin, 1, true);      
        
    pio_gpio_init(pio, mon_pin);
    gpio_pull_up(mon_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, mon_pin, 1, false);    
   
    pio_sm_config c = pin_tic_counter_program_get_default_config(0);
    sm_config_set_sideset_pins(&c, fb_pin); 

    sm_config_set_in_pins(&c, mon_pin);
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE); 

    // sm_config_set_clkdiv_int_frac8(&c, 1, 0);
    float clkdiv = ((float)FSYS / 1000000.0) / 6.0; // 1 MHz - 6 cycles per loop in pio
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_init(pio, sm, 0, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline int32_t pin_tic_counter_get_tics(PIO pio, uint sm)
{
    uint ret;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
    while (n > 0) {
        ret = pio_sm_get(pio, sm);
        n--;
    }
    return ret;
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

    // pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    // pio_set_irq1_source_enabled(pio, pis_interrupt1, true);

    float clkdiv = ((float)FSYS / 1000000.0); 
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    
}

