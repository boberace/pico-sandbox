#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "string.h"

#include "r2r.pio.h"
#include "quadrature_encoder.pio.h"

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

#define R2R_BITS 6
#define PIN_R2R_BASE 10 // 11-15 also used
#define R2R_MAX ((1 << R2R_BITS) - 1)
#define NUM_WAV_SAMPLES 125

PIO pio_r2r = pio1;
uint sm_r2r = 0;

volatile uint8_t fun_wave[NUM_WAV_SAMPLES];
volatile uint8_t * p_fun_wave = &fun_wave[0];

int dma_chan_wave_bytes;
int dma_chan_wave_loop;

#define PIN_DIGIPOT_SDO 16
#define PIN_DIGIPOT_CS 17
#define PIN_DIGIPOT_SCK 18
#define PIN_DIGIPOT_SDI 19
#define PIN_DIGIPOT_SHDN 27
#define SPI_A_BAUD_RATE  1 * 1000 * 1000
#define SPI_A_INST spi0

#define PIN_I2C0_SDA 8
#define PIN_I2C0_SCL 9
#define I2C0_BUADRATE 400*1000
ssd1306_t disp; // create oled display instance

static char event_str[128];
uint8_t interrupt_gpio = 0;
uint32_t interrupt_event = 0;

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

uint16_t intensity = 0; // 256 total (256 is full scale, 0 is off)
uint16_t max_to_amp = 2; // LM386 has minimum 20 gain so need to us other pot to prevent clipping and allow full resolution of first pot

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
int8_t cent_offset = 0;


char* notes[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};


static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};


void setup_waves();
void r2r_program_init(PIO pio, uint sm, uint offset, uint base_pin, uint num_pins, float freq);
void setup_r2r(float freq);
void setup_dma_wave();
void set_frequency(float freq);
void printBinary(uint num, int num_bits);
uint setup_spia();
int digipot_set_value(bool wiper, uint16_t value);
int digipot_read_value(bool wiper, uint16_t * dst);
static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate);
static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm);
void gpio_event_string(char *buf, uint32_t events) ;
void gpio_callback(uint gpio, uint32_t events);
void setup_encoders(void);
uint update_encoders(void);
void setup_tones(void);
void update_current_tone(void);
void update_intensity(void);
void update_cent_off(void);
void update_frequency(void);
void setup_i2c0(void);
void setup_oled(void);
void update_display(void);

int num_written;
int num_read;
uint16_t rv;

uint refresh_counter = 0;

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    setup_waves();  
    setup_r2r(220.0);
    setup_dma_wave();
    setup_encoders();
    setup_i2c0();
    setup_oled();
    setup_tones();

    int spi_ret = setup_spia();
    if(abs((int)(SPI_A_BAUD_RATE-spi_ret)) > 0.05*SPI_A_BAUD_RATE ){
        printf("SPIA setup failed %d\r\n", spi_ret);
        return 1;
    };
    
    num_written = digipot_set_value(0, max_to_amp);
    printf("pot 0, half words written %d, write value %d\n", num_written, max_to_amp); 
    num_read = digipot_read_value(1, &rv);
    printf("pot 0, half words read %d, read value %d, read command ",num_read, rv & 0x1FF);
    printBinary(rv >> 9, 7);

    num_written = digipot_set_value(1, intensity );
    printf("pot 1, half words written %d, write value %d\n", num_written, 0); 
    num_read = digipot_read_value(1, &rv);
    printf("pot 1, half words read %d, read value %d, read command ",num_read, rv & 0x1FF);
    printBinary(rv >> 9, 7);

    uint32_t prev_millis_display = to_ms_since_boot(get_absolute_time());
    uint32_t curr_millis_display = to_ms_since_boot(get_absolute_time());   
    while (true) {
        
        curr_millis_display = to_ms_since_boot(get_absolute_time());
        if( curr_millis_display - prev_millis_display > 100){
            prev_millis_display = curr_millis_display;

            refresh_counter++;                    
            uint new_event = update_encoders();
            if(new_event){
                if(new_event == 1){
                    update_current_tone();
                }
                if(new_event == 2){
                    update_intensity(); 
                    num_written = digipot_set_value(1, intensity);
                    printf("pot 1, half words written %d, write value %d\n", num_written, intensity); 
                    num_read = digipot_read_value(1, &rv);
                    printf("pot 1, half words read %d, read value %d, read command ",num_read, rv & 0x1FF);
                    printBinary(rv >> 9, 7);
                }
                if(new_event == 3){
                    update_cent_off();
                }
                printf("tone_current: %d, freq_current %f hz, cent_offset %d, intensity %d\n", tone_current, freq_current, cent_offset, intensity);
            }
            update_display(); 
        }
        if(interrupt_event){
            printf("interrupt_gpio %d\n", interrupt_gpio);
            printf("GPIO %d %s\n", interrupt_gpio, event_str);
            if(interrupt_event == 0x8){
                // num_read = digipot_read_value(1, &rv);
                // printf("pot 1, half words read %d, read value %d, read command ",num_read, rv & 0x1FF);
                // printBinary(rv >> 9, 7);
            }
            interrupt_event = 0;
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

void setup_r2r(float freq) 
{
    uint offset_r2r = pio_add_program(pio_r2r, &r2r_program);
    printf("Loaded r2r program at %d\n", offset_r2r);     
    r2r_program_init(pio_r2r, sm_r2r, offset_r2r, PIN_R2R_BASE, R2R_BITS, freq);  
    pio_sm_set_enabled(pio_r2r, sm_r2r, true);

}

void setup_dma_wave()
{

    dma_chan_wave_bytes = dma_claim_unused_channel(true); 
    dma_chan_wave_loop = dma_claim_unused_channel(true);

    dma_channel_config c_wave = dma_channel_get_default_config(dma_chan_wave_bytes);
    dma_channel_config c_loop = dma_channel_get_default_config(dma_chan_wave_loop);

    uint pio_r2r_dreq = pio_get_dreq(pio_r2r, sm_r2r, true);
      
    channel_config_set_read_increment(&c_wave, true);
    channel_config_set_write_increment(&c_wave, false);
    channel_config_set_dreq(&c_wave, pio_r2r_dreq);
    channel_config_set_chain_to(&c_wave, dma_chan_wave_loop);
    channel_config_set_transfer_data_size(&c_wave, DMA_SIZE_8);
    channel_config_set_irq_quiet(&c_wave, false);
    dma_channel_configure(
        dma_chan_wave_bytes, 
        &c_wave, 
        &pio1_hw->txf[sm_r2r], 
        NULL, // loop channel fills this in
        NUM_WAV_SAMPLES, 
        false
    );

    channel_config_set_read_increment(&c_loop, false);
    channel_config_set_write_increment(&c_loop, false);
    channel_config_set_chain_to(&c_loop, dma_chan_wave_bytes);
    channel_config_set_transfer_data_size(&c_loop, DMA_SIZE_32);
    channel_config_set_irq_quiet(&c_loop, false);
    dma_channel_configure(
        dma_chan_wave_loop, 
        &c_loop, 
        &dma_hw->ch[dma_chan_wave_bytes].read_addr, 
        &p_fun_wave,
        1, 
        true
    );
}

void set_frequency(float freq) 
{
    pio_sm_set_clkdiv(pio_r2r, sm_r2r, system_frequency/NUM_WAV_SAMPLES/freq);

}   



void printBinary(uint num, int num_bits) 
{
    for (int i = num_bits - 1; i >= 0; i--) {
        printf("%d", (num >> i) & 1);
    }
    printf("\n");
}

uint setup_spia()
{   // setup for 16 bit SPI

    uint spi_ret = spi_init(SPI_A_INST, SPI_A_BAUD_RATE);
    spi_set_format(SPI_A_INST, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_DIGIPOT_SDO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DIGIPOT_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DIGIPOT_SDI, GPIO_FUNC_SPI); 
    gpio_set_function(PIN_DIGIPOT_CS, GPIO_FUNC_SPI); 
    gpio_set_dir(PIN_DIGIPOT_CS, GPIO_OUT);
    gpio_put(PIN_DIGIPOT_CS, 1);

    return spi_ret;

}

int digipot_set_value(bool wiper, uint16_t value)
{ 
    // DS22060B-TABLE 7-2

    value = value & 0x1FF; // 9 bits
    int nw = 0;         
    uint16_t buf = (wiper << 12) | value;
    nw = spi_write16_blocking(SPI_A_INST, &buf, 1);
    sleep_us(1);

    return nw;
}

int digipot_read_value(bool wiper, uint16_t * dst)
{ 
    // DS22060B-TABLE 7-2  
    uint16_t buf;    
    buf = ((wiper << 4) | (0b11 << 2)) << 8;
    int nr = 0;
    nr = spi_read16_blocking(SPI_A_INST, buf, dst, 1);
    sleep_us(1);

    return nr;
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

void gpio_callback(uint gpio, uint32_t events) {

    gpio_event_string(event_str, events);
    interrupt_gpio = gpio;
    interrupt_event = events;

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
        tones[i].note = (m) % 12;
        tones[i].octave = m/12-1;
        tones[i].freq = con_pitch * pow(2.0, (m - 69) / 12.0);
    }
}
void update_current_tone(void)
{
    uint de = new_position_enc1 - old_position_enc1;
    int new_tone = tone_current + de;
    if (new_tone < tone_lowest) {
        new_tone = tone_highest;
    } else if (new_tone > tone_highest) {
        new_tone = tone_lowest;
    }
    tone_current = new_tone;
    update_frequency();
}

void update_intensity(void)
{
    int de = new_position_enc2 - old_position_enc2;
    int16_t new_intensity = intensity + de;
    if (new_intensity < 0) {
        new_intensity = 256;
    } else if (new_intensity > 256) {
        new_intensity = 0;
    }
    intensity = new_intensity;
}

void update_cent_off(void)
{
    int de = (new_position_enc3 - old_position_enc3) ;
    int tco = cent_offset + de;
    if(tco < -50){
        tco = 50;
    } else if(tco > 50){
        tco = -50;
    }
    cent_offset = tco;
    update_frequency();

}

void update_frequency(void)
{
    freq_current = tones[tone_current - tone_lowest].freq*pow(2.0, cent_offset / 1200.0);
    set_frequency(freq_current);
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
    uint tindex = tone_current - tone_lowest;
    int octave = tones[tindex].octave;
    int note = tones[tindex].note;
    float note_freq = tones[tindex].freq;

    ssd1306_draw_string(&disp, 0, 0, 1, "TONE STATION");

    memset(str, 0, sizeof(char));
    sprintf(str, " %s %d: %.2f", notes[note], octave, note_freq);
    ssd1306_draw_string(&disp, 0, 8, 1, str);

    memset(str, 0, sizeof(char));
    sprintf(str, " %d: %.2f", cent_offset, freq_current);
    ssd1306_draw_string(&disp, 0, 16, 1, str);

    memset(str, 0, sizeof(char));
    sprintf(str, " %d / 256", intensity);
    ssd1306_draw_string(&disp, 0, 24, 1, str);


    ssd1306_show(&disp);

}
