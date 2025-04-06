#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "math.h"

#include "blink.pio.h"
#include "pin_watch.pio.h"
#include "coil.pio.h"
#include "pid.h"

#define PIN_SPD_SEN 16
#define PIN_SPD_LED 17

#define PIN_CA1 19
#define PIN_CA2 18
#define PIN_MTR_ENA 22

#define PIN_HALL_LCEN 21 
#define PIN_HALL_RBTW 20 

#define PIN_LED_LCEN 2
#define PIN_LED_RBTW 3
#define PIN_LED_A 4
#define PIN_LED_B 5
#define PIN_LED_BI 25

#define NUM_STATOR_POLE_PAIRS 12
#define NUM_ROTOR_POLE_PAIRS 12

#define FSYS clock_get_hz(clk_sys)

uint coil_A_slice_num = 0;
uint16_t coil_A_wrap = 100;

float motor_pwm_sp = 0.0f;
float motor_curr_rps = 0.0f;
float motor_max_rps = 30.0f;
float motor_target_speed = -0.5f;

PIO pio_pin_watch = pio1;
uint sm_pin_watch = 0;

// PIO pio_blink = pio0;
// uint sm_blink = 1;

PIO pio_coil = pio0;   
uint sm_coil = 0; 

uint hall_cw_seq[] = {2,0,3,1};
uint hall_ccw_seq[] = {1,3,0,2};

uint hall_lcen_tic_counter_rise = 1;
uint hall_lcen_tic_counter_fall = 2;
uint hall_rbtw_tic_counter_rise = 3;
uint hall_rbtw_tic_counter_fall = 4;

uint hall_bin_curr = 0;
uint hall_bin_prev = 0;

volatile bool hall_flag = false;

bool clockwise = true;
bool hall_lcen_initial = false;
bool hall_rbtw_initial = false;
bool hall_lcen = false;
bool hall_rbtw = false;

uint64_t prev_micros_lcen = 0;
uint64_t curr_micros_lcen = 0;
uint64_t prev_micros_rbtw = 0;
uint64_t curr_micros_rbtw = 0;

PID motor_pid;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin, float freq);
// void setup_motor();
// void coil_A(bool a, bool b);
void pin_watch_program_init(PIO pio, uint sm, uint offset, uint pin_watch, uint pin_led);
void coil_program_init(PIO pio, uint sm, uint offset, uint base_coil_pin, uint base_led_pin);
void setup_coil_A_pwm();
void coil_A_pwm(float value);

void gpio_callback(uint gpio, uint32_t events) 
{
    uint64_t curr_micros = to_us_since_boot(get_absolute_time());
    hall_bin_prev = hall_bin_curr;
    if(gpio == PIN_HALL_LCEN) {
        prev_micros_lcen = curr_micros_lcen;
        curr_micros_lcen = curr_micros;
        uint32_t delta_micros_lcen = curr_micros_lcen - prev_micros_lcen;
        if(events & GPIO_IRQ_EDGE_RISE) {          
            gpio_put(PIN_LED_LCEN, 1);
            hall_lcen_tic_counter_rise = delta_micros_lcen;  
            hall_bin_curr ^= 0b10;
        } else if(events & GPIO_IRQ_EDGE_FALL) {
            gpio_put(PIN_LED_LCEN, 0);
            hall_lcen_tic_counter_fall = delta_micros_lcen;
            hall_bin_curr &= ~0b10; 
        }
    } else if(gpio == PIN_HALL_RBTW) {
        prev_micros_rbtw = curr_micros_rbtw;
        curr_micros_rbtw = curr_micros;
        uint32_t delta_micros_rbtw = curr_micros_rbtw - prev_micros_rbtw; 
        if(events & GPIO_IRQ_EDGE_RISE) {
            coil_A_pwm(-motor_pwm_sp); 
            gpio_put(PIN_LED_RBTW, 1);            
            hall_rbtw_tic_counter_rise = delta_micros_rbtw;      
            hall_bin_curr ^= 0b01;
        } else if(events & GPIO_IRQ_EDGE_FALL) {
            coil_A_pwm(motor_pwm_sp);
            gpio_put(PIN_LED_RBTW, 0);             
            hall_rbtw_tic_counter_fall = delta_micros_rbtw;
            hall_bin_curr &= ~0b01; 
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

    // setup_motor();     
    setup_coil_A_pwm();
    pid_init(&motor_pid, 1.0, 0.1, 0.01);
    pid_set_setpoint(&motor_pid, motor_target_speed);

    gpio_init(PIN_LED_LCEN);
    gpio_set_dir(PIN_LED_LCEN, GPIO_OUT);
    gpio_put(PIN_LED_LCEN, 0);

    gpio_init(PIN_LED_RBTW);
    gpio_set_dir(PIN_LED_RBTW, GPIO_OUT);
    gpio_put(PIN_LED_RBTW, 0);      

    gpio_init(PIN_HALL_LCEN);
    gpio_set_irq_enabled_with_callback(PIN_HALL_LCEN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(PIN_HALL_RBTW);
    gpio_set_irq_enabled_with_callback(PIN_HALL_RBTW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // uint offset_coil = pio_add_program(pio_coil, &coil_program);
    // coil_program_init(pio_coil, sm_coil, offset_coil, PIN_CA2, PIN_LED_A); 

    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t prev_millis_motor = to_ms_since_boot(get_absolute_time());

    while (true) {
        
        // motor update
        uint32_t curr_millis_motor = to_ms_since_boot(get_absolute_time());
        if( curr_millis_motor - prev_millis_motor > 10){
            uint32_t delta_millis_motor = curr_millis_motor - prev_millis_motor;
            prev_millis_motor = curr_millis_motor;            

            uint64_t curr_micros = to_us_since_boot(get_absolute_time());
            if (curr_micros - curr_micros_rbtw < 1000000) {
                motor_curr_rps = 1000000.0F/((float)(hall_rbtw_tic_counter_rise + hall_rbtw_tic_counter_fall)*NUM_ROTOR_POLE_PAIRS);
                if(hall_ccw_seq[hall_bin_prev] == hall_bin_curr) {
                    motor_curr_rps = -motor_curr_rps;
                }
            } else {
                motor_curr_rps = 0.0f;
            }
            float pid_output = pid_output_update(&motor_pid, motor_curr_rps/motor_max_rps, delta_millis_motor);
            motor_pwm_sp = pid_output;
        }

        // display update
        uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());
        if( curr_millis_display - prev_millis_display > 1000){
            prev_millis_display = curr_millis_display;            
            // printf("lcen: rise ,%d, fall ,%d, <> rbtw: rise ,%d, fall ,%d, rps, %.2f\n", \
            //     hall_lcen_tic_counter_rise, hall_lcen_tic_counter_fall, \
            //     hall_rbtw_tic_counter_rise, hall_rbtw_tic_counter_fall, \
            //     motor_curr_rps);
            // printf("LCEN %d, RBTW %d\n", gpio_get(PIN_LED_LCEN), gpio_get(PIN_LED_RBTW));
            printf("speed: %.2f, pwm: %.2f, target: %.2f\n", motor_curr_rps/motor_max_rps, motor_pwm_sp, motor_target_speed);

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


void setup_motor() 
{
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
void coil_A(bool a, bool b)
{
    uint32_t mask = (1 << PIN_CA1) | (1 << PIN_CA2); // Mask for PIN_CA1 and PIN_CA2
    uint32_t value = (a << PIN_CA1) | (b << PIN_CA2); // Set values for PIN_CA1 and PIN_CA2
    gpio_put_masked(mask, value);
}

void coil_A_pwm(float value) 
{
    uint16_t value_int = (uint16_t)(fabs(value) * (coil_A_wrap));
    if(value >= -1 && value < 0) {
        pwm_set_chan_level(coil_A_slice_num, PWM_CHAN_A, value_int);
        pwm_set_chan_level(coil_A_slice_num, PWM_CHAN_B, 0);
        // printf("n coil_A_pwm: %f\n", value);
    } else if(value > 0 && value <= 1) {
        pwm_set_chan_level(coil_A_slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(coil_A_slice_num, PWM_CHAN_B, value_int);
        // printf("p coil_A_pwm: %f\n", value);
    } else {
        pwm_set_chan_level(coil_A_slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(coil_A_slice_num, PWM_CHAN_B, 0);
        // printf("0 coil_A_pwm: %f\n", value);        
    }

}
void setup_coil_A_pwm() 
{
    gpio_set_function(PIN_CA1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_CA2, GPIO_FUNC_PWM);
    coil_A_slice_num = pwm_gpio_to_slice_num(PIN_CA2);
    pwm_set_wrap(coil_A_slice_num, coil_A_wrap);
    float clkdiv = ((float)FSYS / 1000000.0); 
    pwm_set_clkdiv(coil_A_slice_num, clkdiv); 
    pwm_set_enabled(coil_A_slice_num, true);   
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

void coil_program_init(PIO pio, uint sm, uint offset, uint base_coil_pin, uint base_led_pin) 
{

    pio_gpio_init(pio, base_coil_pin);
    pio_gpio_init(pio, base_coil_pin + 1);

    pio_gpio_init(pio, base_led_pin);

    pio_sm_set_consecutive_pindirs(pio, sm, base_coil_pin, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, base_led_pin, 1, true);

    pio_sm_config c = coil_program_get_default_config(offset);
 
    sm_config_set_out_pins(&c, base_coil_pin, 2);
    sm_config_set_sideset_pins(&c, base_led_pin);

    sm_config_set_out_shift(&c, false, true, 32); 
 
    float clkdiv = ((float)FSYS / 1000000.0); 
    sm_config_set_clkdiv(&c, clkdiv);
 
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
 }