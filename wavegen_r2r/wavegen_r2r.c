#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "math.h"
#include "string.h"
#include "quadrature_encoder.pio.h"
#include "r2r.pio.h"
#include "ssd1306.h"

#define system_frequency clock_get_hz(clk_sys)

#define tone_lowest 21
#define tone_highest 108
#define tone_starting 69
#define num_tones (tone_highest - tone_lowest)

const uint PIN_AB_ENC1 = 2; // 3 also used
const uint PIN_BUT_ENC1 = 4;
const uint PIN_BUT_ENC2 = 5;
const uint PIN_AB_ENC2 = 6; // 7 also used
const uint PIN_AB_ENC3 = 20; // 21 also used
const uint PIN_BUT_ENC3 = 22;

#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

#define R2R_BITS 6
#define R2R_BASE_PIN 10 // 11-15 also used
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define NUM_WAV_SAMPLES 125

#define I2C0_BUADRATE 400*1000
ssd1306_t disp; // create oled display instance

static char event_str[128];

PIO pio_quadenc = pio0;
uint sm_quadenc1 = 0;
uint sm_quadenc2 = 1;
uint sm_quadenc3 = 2;

int new_value_enc1, delta_enc1, old_value_enc1 = 0, new_position_enc1 = 0, old_position_enc1 = 0;
int last_value_enc1 = -1, last_delta_enc1 = -1;
int new_value_enc2, delta_enc2, old_value_enc2 = 0, new_position_enc2 = 0, old_position_enc2 = 0;
int last_value_enc2 = -1, last_delta_enc2 = -1;    
int new_value_enc3, delta_enc3, old_value_enc3 = 0, new_position_enc3 = 0, old_position_enc3 = 0;
int last_value_enc3 = -1, last_delta_enc3 = -1;    

PIO pio_r2r = pio1;
uint sm_r2r = 0;

uint8_t fun_wave[NUM_WAV_SAMPLES];

int dma_chan_wave_bytes;
int dma_chan_wave_loop;

typedef struct {
    uint8_t midi;
    uint8_t note;
    uint8_t octave;
    float freq;
} tone_t;
tone_t tones[num_tones];
float con_pitch = 440.0; // A4 - 440Hz
float freq_current = 440.0; // A4 - 440Hz
uint8_t tone_current = tone_starting; // A4 - 440Hz
float cent_offset = 0.0;

char* notes[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};

uint midi_to_note(uint midi){(midi + 3) % 12;}
uint midi_to_octave(uint midi){midi/12.0-1;}

void setup_waves();
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq);
void setup_r2r(void);
void setup_dma_wave();
void dma_handler_wave();
void set_frequency(float freq);
static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate);
static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm);
void gpio_event_string(char *buf, uint32_t events);
void gpio_callback(uint gpio, uint32_t events);
void setup_i2c0(void); 
void setup_oled(void);
void update_display(void);
void setup_encoders(void);
uint update_encoders(void);
void setup_tones(void);
void update_current_tone(void);
// void change_fundamental(void);

uint refresh_counter = 0;
int main()
{
    stdio_init_all();
    sleep_ms(1000);

    setup_waves();  
    setup_r2r();
    setup_dma_wave();
    setup_encoders();
    setup_i2c0();
    setup_oled();

    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());

    while(true){

        sleep_ms(1);
        // display
        curr_millis_display = to_ms_since_boot(get_absolute_time());
        if( curr_millis_display - prev_millis_display > 100){
            prev_millis_display = curr_millis_display;

            refresh_counter++;                    
            uint new_event = update_encoders();
            if(new_event){
                if(new_event == 1){
                    update_current_tone();
                    printf("tone_current: %d\n", tone_current);
                }
                if(new_event == 2){
                    printf("new_position_enc2: %d\n", new_position_enc2);
                }
                if(new_event == 3){
                    printf("new_position_enc3: %d\n", new_position_enc3);
                }
            }
            update_display(); 
        }
    }


}

void setup_waves() 
{ // =ROUND(63*(SIN(A1*2*PI()/256)/2+0.5),0)
    for (int i = 0; i < NUM_WAV_SAMPLES; i++) {
        fun_wave[i] = (uint8_t)round(R2R_MAX * (sin(i * 2 * M_PI / NUM_WAV_SAMPLES) / 2 + 0.5));
    }
}

void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq) 
{
    pio_sm_config c = r2r_program_get_default_config(offset);
    sm_config_set_out_pins(&c, base_pin, num_pins); 
    sm_config_set_out_shift(&c, true, true, 8); 
    for(int i = 0; i < num_pins; i++) {
        pio_gpio_init(pio, base_pin + i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, base_pin, num_pins, true);
    sm_config_set_clkdiv(&c, system_frequency/NUM_WAV_SAMPLES/freq);
    pio_sm_init(pio, sm, offset, &c);
}

void setup_r2r(void) 
{
    uint offset_r2r = pio_add_program(pio_r2r, &r2r_program);
    printf("Loaded r2r program at %d\n", offset_r2r);     
    r2r_program_init(pio_r2r, sm_r2r, offset_r2r, R2R_BASE_PIN, R2R_BITS, 440.0f);  
    pio_sm_set_enabled(pio_r2r, sm_r2r, true);

}

void setup_dma_wave()
{

    dma_chan_wave_bytes = 0; 
    dma_chan_wave_loop = 1;

    dma_channel_config c_bytes = dma_channel_get_default_config(dma_chan_wave_bytes);
    dma_channel_config c_loop = dma_channel_get_default_config(dma_chan_wave_loop);

    // uint8_t * p_fun_wave = fun_wave;
    // channel_config_set_read_increment(&c_loop, false);
    // channel_config_set_chain_to(&c_loop, dma_chan_wave_bytes);
    // channel_config_set_irq_quiet(&c_loop, true);
    // dma_channel_configure(
    //     dma_chan_wave_loop, 
    //     &c_loop, 
    //     &dma_hw->ch[dma_chan_wave_bytes].read_addr, 
    //     &p_fun_wave, 
    //     1, 
    //     false
    // );
      
    channel_config_set_dreq(&c_bytes, pio_get_dreq(pio_r2r, sm_r2r, true));
    // channel_config_set_chain_to(&c_bytes, dma_chan_wave_loop);
    channel_config_set_transfer_data_size(&c_bytes, DMA_SIZE_8);
    channel_config_set_irq_quiet(&c_bytes, false);
    dma_channel_configure(
        dma_chan_wave_bytes, 
        &c_bytes, 
        &pio1_hw->txf[sm_r2r], 
        fun_wave,
        NUM_WAV_SAMPLES, 
        false
    );
    dma_channel_set_irq0_enabled(dma_chan_wave_bytes, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler_wave);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_chan_wave_bytes);

}

void dma_handler_wave() 
{
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan_wave_bytes;
    dma_channel_set_read_addr(dma_chan_wave_bytes, &fun_wave[0], true);
    
}

void set_frequency(float freq) 
{
    pio_sm_set_clkdiv_int_frac(pio_r2r, sm_r2r, system_frequency/NUM_WAV_SAMPLES/freq, 0);
}   


// max_step_rate is used to lower the clock of the state machine to save power
// if the application doesn't require a very high sampling rate. Passing zero
// will set the clock to the maximum

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

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

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

void gpio_callback(uint gpio, uint32_t events) 
{
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
}

void setup_i2c0(void)
{

    i2c_init(i2c0, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
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

    ssd1306_draw_string(&disp, 0, 0, 1, "wavegen_r2r");

    memset(str, 0, sizeof(char));
    sprintf(str, " %d", refresh_counter);
    ssd1306_draw_string(&disp, 0, 8, 1, str);

    // memset(str, 0, sizeof(char));
    // sprintf(str, "%d, %d", midi_idx, midi_cent);
    // ssd1306_draw_string(&disp, 0, 16, 2, str);

    ssd1306_show(&disp);

}

void setup_encoders(void)
{
    // we don't really need to keep the offset, as this program must be loaded
    // at offset 0
    pio_add_program(pio_quadenc, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_quadenc, sm_quadenc1, PIN_AB_ENC1, 2);
    quadrature_encoder_program_init(pio_quadenc, sm_quadenc2, PIN_AB_ENC2, 2);
    quadrature_encoder_program_init(pio_quadenc, sm_quadenc3, PIN_AB_ENC3, 2);

    gpio_init(PIN_BUT_ENC1);
    gpio_pull_up(PIN_BUT_ENC1);
    gpio_set_irq_enabled_with_callback(PIN_BUT_ENC1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_init(PIN_BUT_ENC2);
    gpio_pull_up(PIN_BUT_ENC2);
    gpio_set_irq_enabled_with_callback(PIN_BUT_ENC2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_init(PIN_BUT_ENC3);
    gpio_pull_up(PIN_BUT_ENC3);
    gpio_set_irq_enabled_with_callback(PIN_BUT_ENC3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

}

uint update_encoders(void)
{
    uint new_event = 0;

    new_value_enc1 = quadrature_encoder_get_count(pio_quadenc, sm_quadenc1);
    delta_enc1 = new_value_enc1 - old_value_enc1;
    old_value_enc1 = new_value_enc1;
    old_position_enc1 = new_position_enc1;
    new_position_enc1 = new_value_enc1 >> 2;


    new_value_enc2 = quadrature_encoder_get_count(pio_quadenc, sm_quadenc2);
    delta_enc2 = new_value_enc2 - old_value_enc2;
    old_value_enc2 = new_value_enc2;
    old_position_enc2 = new_position_enc2;
    new_position_enc2 = new_value_enc2 >> 2;


    new_value_enc3 = quadrature_encoder_get_count(pio_quadenc, sm_quadenc3);
    delta_enc3 = new_value_enc3 - old_value_enc3;
    old_value_enc3 = new_value_enc3;
    old_position_enc3 = new_position_enc3;
    new_position_enc3 = new_value_enc3 >> 2;


    if (new_value_enc1 != last_value_enc1 || delta_enc1 != last_delta_enc1 ) {
        last_value_enc1 = new_value_enc1;
        last_delta_enc1 = delta_enc1;
    }

    if (new_value_enc2 != last_value_enc2 || delta_enc2 != last_delta_enc2 ) {
        last_value_enc2 = new_value_enc2;
        last_delta_enc2 = delta_enc2;
    }

    if (new_value_enc3 != last_value_enc3 || delta_enc3 != last_delta_enc3 ) {
        last_value_enc3 = new_value_enc3;
        last_delta_enc3 = delta_enc3;
    }

    if(old_position_enc1 != new_position_enc1){
        new_event = 1;
    }
    if(old_position_enc2 != new_position_enc2){
        new_event = 2;
    }
    if(old_position_enc3 != new_position_enc3){
        new_event = 3;
    }

    return new_event;
}

void setup_tones(void)
{
    for (int i = 0; i < num_tones; i++) {
        int m = i + tone_lowest;
        tones[i].midi = m;
        tones[i].note = midi_to_note(m);
        tones[i].octave = midi_to_octave(m);
        tones[i].freq = 440.0 * pow(2.0, (m - 69) / 12.0);
    }
}
void update_current_tone(void)
{
    uint dt = new_position_enc1 - old_position_enc1;
    int new_tone = tone_current + dt;
    if (new_tone < tone_lowest) {
        new_tone = tone_highest;
    } else if (new_tone > tone_highest) {
        new_tone = tone_lowest;
    }
    tone_current = new_tone;
    // set_frequency(tones[tone_current - tone_lowest].freq);
}

