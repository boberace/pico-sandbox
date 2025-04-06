#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "math.h"
#include "string.h"

#include "blink.pio.h"
#include "pin_watch.pio.h"
#include "coil.pio.h"
#include "quadrature_encoder.pio.h"

#include "pid.h"
#include "ssd1306.h"

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

#define PIN_AB_ENC1  6 // 7 also used
#define PIN_BUT_ENC1 8

#define NUM_STATOR_POLE_PAIRS 12
#define NUM_ROTOR_POLE_PAIRS 12

#define FSYS clock_get_hz(clk_sys)

#define PIN_I2C0_SDA 12
#define PIN_I2C0_SCL 13
#define I2C0_BUADRATE 400*1000
ssd1306_t disp; // create oled display instance

uint coil_A_slice_num = 0;
uint16_t coil_A_wrap = 100;

float motor_pwm_sp = 0.0f;
float motor_curr_rps = 0.0f;
float motor_max_rps = 30.0f;
float motor_target_speed = 0.0f;

PIO pio_pin_watch = pio1;
uint sm_pin_watch = 0;

PIO pio_quadenc = pio0;
uint sm_quadenc1 = 0;

PIO pio_coil = pio0;   
uint sm_coil = 1; 

PIO pio_blink = pio0;
uint sm_blink = 2;

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

static char event_str[128];
uint8_t interrupt_gpio = 0;
uint32_t interrupt_event = 0;

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

int new_value_enc1, delta_enc1, old_value_enc1 = 0, new_position_enc1 = 0, old_position_enc1 = 0;
int last_value_enc1 = -1, last_delta_enc1 = -1;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin, float freq);
void pin_watch_program_init(PIO pio, uint sm, uint offset, uint pin_watch, uint pin_led);
void coil_program_init(PIO pio, uint sm, uint offset, uint base_coil_pin, uint base_led_pin);
void setup_coil_A_pwm(void);
void coil_A_pwm(float value);
void setup_i2c0(void);
void setup_oled(void);
void update_display(void);
static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate);
static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm);
void gpio_event_string(char *buf, uint32_t events);
void encoder_callback(uint gpio, uint32_t events);
void setup_encoders(void);
uint update_encoders(void);
void setup_halls(void);
void hall_callback(uint gpio, uint32_t events);

int main()
{
    stdio_init_all();

    sleep_ms(1000); 

    setup_encoders();
    setup_i2c0();
    setup_oled(); 

    pio_add_program(pio_pin_watch, &pin_watch_program);
    pin_watch_program_init(pio_pin_watch, sm_pin_watch, 0, PIN_SPD_SEN, PIN_SPD_LED); 
 
    uint offset = pio_add_program(pio_blink, &blink_program);
    blink_program_init(pio_blink, sm_blink, offset, PIN_LED_BI, 1.0f); // 1Hz    

    setup_coil_A_pwm();
    pid_init(&motor_pid, 1.0, 0.1, 0.01);
    pid_set_setpoint(&motor_pid, motor_target_speed);

    setup_halls();

    uint hall_lcen = gpio_get(PIN_HALL_LCEN);
    uint hall_rbtw = gpio_get(PIN_HALL_RBTW);

    // uint offset_coil = pio_add_program(pio_coil, &coil_program);
    // coil_program_init(pio_coil, sm_coil, offset_coil, PIN_CA2, PIN_LED_A);    

    uint32_t prev_millis_uart = to_ms_since_boot(get_absolute_time());
    uint32_t prev_millis_motor = to_ms_since_boot(get_absolute_time());
    uint32_t prev_millis_oled = to_ms_since_boot(get_absolute_time());


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
            pid_set_setpoint(&motor_pid, motor_target_speed);
            float pid_output = pid_output_update(&motor_pid, motor_curr_rps/motor_max_rps, delta_millis_motor);
            motor_pwm_sp = pid_output;
        }
        // oled update
        uint32_t curr_millis_oled = to_ms_since_boot(get_absolute_time());
        if( curr_millis_oled - prev_millis_oled > 100){
            prev_millis_oled = curr_millis_oled;  
            update_encoders(); 
            motor_target_speed = (float)(new_position_enc1%10) / 10.0;
            update_display();      
        }
        // uart update
        uint32_t curr_millis_uart = to_ms_since_boot(get_absolute_time());
        if( curr_millis_uart - prev_millis_uart > 1000){
            prev_millis_uart = curr_millis_uart;   
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

void coil_A_pwm(float value) 
{
    uint16_t value_int = (uint16_t)(fabs(value) * (coil_A_wrap + 1));
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
void setup_coil_A_pwm(void) 
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

 void setup_i2c0(void)
 {
 
     i2c_init(i2c0, I2C0_BUADRATE);
     gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
     gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
     gpio_pull_up(PIN_I2C0_SDA);
     gpio_pull_up(PIN_I2C0_SCL);
 }
 
 
 void setup_oled(void) 
 {
 
     disp.external_vcc=false;
     ssd1306_init(&disp, 128, 32, 0x3C, i2c0);
     ssd1306_clear(&disp);
     ssd1306_draw_string(&disp, 8, 0, 1, (char*)"SSD1306");
     ssd1306_draw_string(&disp, 8, 16, 2, (char*)"DISPLAY");
     ssd1306_show(&disp);
 }

 void update_display(void)
{ 
    
    ssd1306_clear(&disp);

    char str[128];

    // ssd1306_draw_string(&disp, 0, 0, 1, "SINGLE COIL");  

    memset(str, 0, sizeof(char));
    sprintf(str, "ENC: %d", new_position_enc1);
    ssd1306_draw_string(&disp, 0, 0, 1, str);

    memset(str, 0, sizeof(char));
    sprintf(str, "PWM: %.2f", motor_pwm_sp);
    ssd1306_draw_string(&disp, 0, 8, 1, str);

    memset(str, 0, sizeof(char));
    sprintf(str, "SPD: %.2f",  motor_curr_rps/motor_max_rps);
    ssd1306_draw_string(&disp, 0, 16, 1, str); 

    memset(str, 0, sizeof(char));
    sprintf(str, "SP : %.2f", motor_target_speed);
    ssd1306_draw_string(&disp, 0, 24, 1, str);

    ssd1306_show(&disp);

}

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

void encoder_callback(uint gpio, uint32_t events) 
{

    gpio_event_string(event_str, events);
    interrupt_gpio = gpio;
    interrupt_event = events;

}

void setup_encoders(void)
{

    pio_add_program(pio_quadenc, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_quadenc, sm_quadenc1, PIN_AB_ENC1, 2);
    // quadrature_encoder_program_init(pio_quadenc, sm_quadenc2, PIN_AB_ENC2, 2);
    // quadrature_encoder_program_init(pio_quadenc, sm_quadenc3, PIN_AB_ENC3, 2);

    gpio_init(PIN_BUT_ENC1);
    gpio_pull_up(PIN_BUT_ENC1);
    gpio_set_irq_enabled_with_callback(PIN_BUT_ENC1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    // gpio_init(PIN_BUT_ENC2);
    // gpio_pull_up(PIN_BUT_ENC2);
    // gpio_set_irq_enabled_with_callback(PIN_BUT_ENC2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    // gpio_init(PIN_BUT_ENC3);
    // gpio_pull_up(PIN_BUT_ENC3);
    // gpio_set_irq_enabled_with_callback(PIN_BUT_ENC3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);

}


uint update_encoders(void)
{
    uint new_event = 0;

    new_value_enc1 = quadrature_encoder_get_count(pio_quadenc, sm_quadenc1);
    delta_enc1 = new_value_enc1 - old_value_enc1;
    old_value_enc1 = new_value_enc1;
    old_position_enc1 = new_position_enc1;
    new_position_enc1 = new_value_enc1 >> 2;


    if (new_value_enc1 != last_value_enc1 || delta_enc1 != last_delta_enc1 ) {
        last_value_enc1 = new_value_enc1;
        last_delta_enc1 = delta_enc1;
    }

    if(old_position_enc1 != new_position_enc1){
        new_event = 1;
    }

    return new_event;
}

void setup_halls(void) 
{
    gpio_init(PIN_LED_LCEN);
    gpio_set_dir(PIN_LED_LCEN, GPIO_OUT);
    gpio_put(PIN_LED_LCEN, 0);

    gpio_init(PIN_LED_RBTW);
    gpio_set_dir(PIN_LED_RBTW, GPIO_OUT);
    gpio_put(PIN_LED_RBTW, 0);      

    gpio_init(PIN_HALL_LCEN);
    gpio_set_irq_enabled_with_callback(PIN_HALL_LCEN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &hall_callback);

    gpio_init(PIN_HALL_RBTW);
    gpio_set_irq_enabled_with_callback(PIN_HALL_RBTW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &hall_callback);
}

void hall_callback(uint gpio, uint32_t events) 
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

// void update_motor_sp(void)
// {
//     uint de = new_position_enc1 - old_position_enc1;
//     int new_tone = tone_current + de;
//     if (new_tone < tone_lowest) {
//         new_tone = tone_highest;
//     } else if (new_tone > tone_highest) {
//         new_tone = tone_lowest;
//     }
//     tone_current = new_tone;
//     update_frequency();
// }