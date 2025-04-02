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

#define PIN_HALL_LCEN 21 
#define PIN_HALL_RBTW 20 

#define PIN_LED_LCEN 2
#define PIN_LED_RBTW 3
#define PIN_LED 25

#define NUM_STATOR_POLES 12

#define FSYS clock_get_hz(clk_sys)

PIO pio_pin_watch = pio1;
uint sm_pin_watch = 0;

PIO pio_blink = pio1;
uint sm_blink = 1;

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
void pin_tic_counter_program_init(PIO pio, uint sm, uint mon_pin, uint fb_pin);
static inline int32_t pin_tic_counter_get_tics(PIO pio, uint sm);
void pin_watch_program_init(PIO pio, uint sm, uint offset, uint pin_watch, uint pin_led);

void pin_tic_counter_isr()
{   
    uint16_t irq_status = (pio_pin_tic_counter->irq);
    
    if (irq_status & (1 << 0)) // SM0 IRQ0 (0 + 0) % 4 = 0
    {
        
        hall_lcen_tic_counter_rise = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_lcen_tic_counter);
    }   
    if (irq_status & (1 << 1)) // SM0 IRQ1 (0 + 1) % 4 = 1
    {
        
        hall_lcen_tic_counter_fall = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_lcen_tic_counter);
    }     
    if (irq_status & (1 << 2)) // SM2 IRQ0 (2 + 0) % 4 = 2
    { 
        coil_A(!clockwise, clockwise);
        hall_rbtw_tic_counter_rise = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_rbtw_tic_counter);
    }
    if (irq_status & (1 << 3)) // SM2 IRQ1 (2 + 1) % 4 = 3
    { 
        coil_A(clockwise, !clockwise);
        hall_rbtw_tic_counter_fall = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_rbtw_tic_counter);
    }
    // printf("irq_status: %d\n", irq_status);
    // Clear the IRQs
    pio_pin_tic_counter->irq = irq_status;
}

int main()
{
    stdio_init_all();

    sleep_ms(1000); 

    // offset not used as the program is loaded at address 0
    pio_add_program(pio_pin_watch, &pin_watch_program);
    pin_watch_program_init(pio_pin_watch, sm_pin_watch, 0, PIN_SPD_SEN, PIN_SPD_LED); 

    // offset not used as the program is loaded at address 0
    pio_add_program(pio_pin_tic_counter, &pin_tic_counter_program);    
    pin_tic_counter_program_init(pio_pin_tic_counter, sm_hall_lcen_tic_counter, PIN_HALL_LCEN, PIN_LED_LCEN);
    pin_tic_counter_program_init(pio_pin_tic_counter, sm_hall_rbtw_tic_counter, PIN_HALL_RBTW, PIN_LED_RBTW);
 
    uint offset = pio_add_program(pio_blink, &blink_program);
    blink_program_init(pio_blink, sm_blink, offset, PIN_LED, 1.0f); // 1Hz    

    setup_motor();           

    irq_set_exclusive_handler(PIO0_IRQ_0, pin_tic_counter_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    irq_set_exclusive_handler(PIO0_IRQ_1, pin_tic_counter_isr);
    irq_set_enabled(PIO0_IRQ_1, true);

    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());   

    while (true) {

        curr_millis_display = to_ms_since_boot(get_absolute_time());
        if( curr_millis_display - prev_millis_display > 1000){
            prev_millis_display = curr_millis_display;            
            printf("lcen: rise ~%d~ fall ~%d~ <> rbtw: rise ~%d~, fall ~%d~\n", hall_lcen_tic_counter_rise, hall_lcen_tic_counter_fall, hall_rbtw_tic_counter_rise, hall_rbtw_tic_counter_fall);
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

    gpio_init(PIN_MTR_ENA);
    gpio_set_dir(PIN_MTR_ENA, GPIO_OUT);
    gpio_put(PIN_MTR_ENA, 1);  

}
void coil_A(bool a, bool b){
    uint32_t mask = (1 << PIN_CA1) | (1 << PIN_CA2); // Mask for PIN_CA1 and PIN_CA2
    uint32_t value = (a << PIN_CA1) | (b << PIN_CA2); // Set values for PIN_CA1 and PIN_CA2
    gpio_put_masked(mask, value);
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
    float clkdiv = ((float)FSYS / 1000000.0) / 7.0; // 1 MHz - 7 cycles per loop in pio
    sm_config_set_clkdiv(&c, clkdiv);

    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt1, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt2, true);
    pio_set_irq1_source_enabled(pio, pis_interrupt3, true);

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

    float clkdiv = ((float)FSYS / 1000000.0); 
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    
}

