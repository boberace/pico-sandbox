#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

// #include "pid.h"
#include "pin_tic_counter.pio.h"

#define GPIO_LED1 2
#define GPIO_LED2 3
#define GPIO_LED3 4
#define GPIO_LED4 5

#define GPIO_STROBE 15

#define GPIO_MOT_DIR 16
#define GPIO_MOT_SPD 17 // reverse acting
#define GPIO_MOT_PWM 18

#define GPIO_HALL 22

#define PWM_MOT_TOP 999 // highest value for pwm motor - must update pwm parameter if not 999

#define FSYS clock_get_hz(clk_sys)

uint mot_rpm = 5;


PIO pio_pin_tic_counter = pio0;   
uint sm_hall_tic_counter = 0; 
uint sm_speed_tic_counter = 1; 

static void setup_pwm_motor();
static void set_motor_speed(float speed);
void pin_tic_counter_program_init(PIO pio, uint sm, uint mon_pin, uint fb_pin);
static inline int32_t pin_tic_counter_get_tics(PIO pio, uint sm);
static void setup_pwm_strobe();


void core1_entry() {

    uint ms_prev0 = 0;
    uint ms_curr0 = 0;

    while (true) {

        ms_curr0 = time_us_32() / 1000; // Get current milliseconds
        if (ms_curr0 - ms_prev0 > 500) {
            ms_prev0 = ms_curr0;
            gpio_xor_mask(1u << GPIO_LED2);
            // printf("1\n");
        }
    }
}

int main()
{
    stdio_init_all();

    uint gpio_out_pins[] = {\
        PICO_DEFAULT_LED_PIN,\
        GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4,\
        GPIO_STROBE,\
        GPIO_MOT_DIR};

    for (int i = 0; i < sizeof(gpio_out_pins) / sizeof(gpio_out_pins[0]); i++) {
        gpio_init(gpio_out_pins[i]);
        gpio_set_dir(gpio_out_pins[i], GPIO_OUT);
        gpio_put(gpio_out_pins[i], 0);
    }
    
    setup_pwm_motor();

    uint ms_prev = 0;
    uint ms_curr = 0;

     // Set up the interrupt for GPIO_HALL
    //  gpio_set_irq_enabled_with_callback(GPIO_HALL, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    uint offset = pio_add_program(pio_pin_tic_counter, &pin_tic_counter_program);
    pin_tic_counter_program_init(pio_pin_tic_counter, sm_hall_tic_counter, GPIO_HALL, GPIO_LED3);
    pin_tic_counter_program_init(pio_pin_tic_counter, sm_speed_tic_counter, GPIO_MOT_SPD, GPIO_LED4);
      
    sleep_ms(100); // core1 will not start unless delay is added before calling multicore_launch_core1 when using swd to flash
    multicore_launch_core1(core1_entry);



    for(int i = 0; i < 7; i++){
        set_motor_speed(i/10.0);
        sleep_ms(500);
    }

    setup_pwm_strobe();

    uint counter = 0;
    while (true) {

        ms_curr = time_us_32() / 1000; // Get current milliseconds
        if (ms_curr - ms_prev > 1000) {
            ms_prev = ms_curr;
            // printf("0\n");
            gpio_xor_mask(1u << GPIO_LED1);
            uint32_t htics = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_hall_tic_counter);
            printf("%u, hall tics: %u\n", counter, htics);
            uint32_t stics = pin_tic_counter_get_tics(pio_pin_tic_counter, sm_speed_tic_counter);
            printf("%u, speed tics: %u\n", counter, stics);
            counter++;
        }
    }
    
}

/** \brief set up pwm for motor
 *
 */
static void setup_pwm_motor(){

    gpio_set_function(GPIO_MOT_PWM, GPIO_FUNC_PWM); 
    uint8_t pwm_slice1 = pwm_gpio_to_slice_num(GPIO_MOT_PWM);
    
    // 4.5.2.6. Configuring PWM Period
    uint16_t pwm_TOP = PWM_MOT_TOP;
    uint8_t pwm_DIV_int = 6;
    uint8_t pwm_DIV_frac = 4;
    bool pwm_CSR = 0;
    // uint32_t pwm_freq = FSYS / ((pwm_TOP + 1)*(pwm_CSR + 1)*(pwm_DIV_int + pwm_DIV_frac/16 ));    
    // 20k = 125m / ((999+1)*(0+1)*(6+4/16))
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, pwm_TOP); 
    pwm_config_set_clkdiv_int_frac(&config, pwm_DIV_int, pwm_DIV_frac);
    pwm_config_set_phase_correct(&config,pwm_CSR);
    pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
    pwm_config_set_output_polarity(&config, 0, 0);
    pwm_init(pwm_slice1, &config, true);

    set_motor_speed(0);
}


/* \brief set motor speed
 *
 * \param speed float from 0 to 1
 */
static void set_motor_speed(float speed){

    if(speed >= 0.0f && speed <= 1.0f)
        pwm_set_gpio_level(GPIO_MOT_PWM, (1-speed)*PWM_MOT_TOP);
    else{
        pwm_set_gpio_level(GPIO_MOT_PWM, PWM_MOT_TOP);
        printf("Speed must be between 0 and 1. You asked for %f \n", speed);
    }
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

    sm_config_set_clkdiv(&c, 1.0);

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

static void setup_pwm_strobe(){

    gpio_set_function(GPIO_STROBE, GPIO_FUNC_PWM); 
    uint8_t pwm_slice1 = pwm_gpio_to_slice_num(GPIO_STROBE);
    
    // 4.5.2.6. Configuring PWM Period
    uint16_t pwm_TOP = 65535;
    uint8_t pwm_DIV_int = 85;
    uint8_t pwm_DIV_frac = 8;
    bool pwm_CSR = 0;
    // uint32_t pwm_freq = FSYS / ((pwm_TOP + 1)*(pwm_CSR + 1)*(pwm_DIV_int + pwm_DIV_frac/16 ));    
    // 20k = 125m / ((999+1)*(0+1)*(6+4/16))
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, pwm_TOP); 
    pwm_config_set_clkdiv_int_frac(&config, pwm_DIV_int, pwm_DIV_frac);
    pwm_config_set_phase_correct(&config,pwm_CSR);
    pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
    pwm_config_set_output_polarity(&config, 0, 0);
    pwm_init(pwm_slice1, &config, true);

    pwm_set_gpio_level(GPIO_STROBE, pwm_TOP*0.01);
}