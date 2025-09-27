/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#if PICO_ON_DEVICE
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#endif

#include "pico/stdlib.h"
#include "pico/audio_i2s.h"

#if PICO_ON_DEVICE
#include "pico/binary_info.h"
bi_decl(bi_3pins_with_names(PICO_AUDIO_I2S_DATA_PIN, "I2S DIN",
                           PICO_AUDIO_I2S_CLOCK_PIN_BASE, "I2S BCK",
                           PICO_AUDIO_I2S_CLOCK_PIN_BASE + 1, "I2S LRCK"));
#endif

#define SINE_WAVE_TABLE_LEN 2048
#define SAMPLES_PER_BUFFER 256
#define SAMPLE_RATE 24000

static int16_t sine_wave_table[SINE_WAVE_TABLE_LEN];

const struct audio_format *output_format;

struct audio_buffer_pool *init_audio(void) {

    /* Request a PCM S16 stereo format at 24 kHz for I2S */
    static audio_format_t audio_format = {
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,
            .sample_freq = SAMPLE_RATE,
            .channel_count = 2,
    };

    struct audio_buffer_format producer_format;
    producer_format.format = &audio_format;
    /* bytes per frame (channels * 2 bytes per int16 sample) */
    producer_format.sample_stride = sizeof(int16_t) * audio_format.channel_count;

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3,
                                                                      SAMPLES_PER_BUFFER);
    bool __unused ok;

    struct audio_i2s_config config = {
            .data_pin = PICO_AUDIO_I2S_DATA_PIN,
            .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
            .dma_channel = 0,
            .pio_sm = 0,
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open I2S audio device.\n");
    } else {
        printf("I2S: sample rate %u, channels %u\n", output_format->sample_freq,
               output_format->channel_count);
    }


    
    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);

    return producer_pool;
}

/* compute step for desired frequency (safe 64-bit intermediate) */
static uint32_t compute_step_for_freq(uint32_t freq_hz, uint32_t sample_rate_hz) {
    return (uint32_t)((uint64_t)freq_hz * (uint64_t)SINE_WAVE_TABLE_LEN * 65536ULL / sample_rate_hz);
}

/* inverse: compute frequency (Hz) from step and sample rate */
static double compute_freq_from_step(uint32_t step_q16, uint32_t sample_rate_hz) {
    return ((double)sample_rate_hz * (double)step_q16 / 65536.0) / (double)SINE_WAVE_TABLE_LEN;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000); // wait for usb serial to connect
    printf("\nSine wave generator (I2S stereo - independent L/R)\n");

    for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 32767 * cosf(i * 2 * (float)(M_PI / SINE_WAVE_TABLE_LEN));
    } 

    struct audio_buffer_pool *ap = init_audio();
    uint32_t sample_rate = output_format ? output_format->sample_freq : SAMPLE_RATE;

    /* Use the same frequency on both channels, but offset the right channel by 90 degrees */
    uint32_t desired_hz = 220; /* frequency for both channels */
    uint32_t step_left  = compute_step_for_freq(desired_hz, sample_rate);
    uint32_t step_right = step_left; /* same frequency */

    uint32_t pos_left = 0;
    uint32_t pos_max  = 0x10000 * SINE_WAVE_TABLE_LEN;
/* 90 degree phase offset for right channel = 1/4 of the cycle */
    uint32_t pos_right = (pos_left + pos_max / 4) % pos_max;

    uint vol = 128;

    const uint32_t channels = output_format ? output_format->channel_count : 2;

    while (true) {
        int c = getchar_timeout_us(0);
        if (c >= 0) {
            if (c == '-' && vol) vol -= 4;
            if ((c == '=' || c == '+') && vol < 255) vol += 4;
            /* keyboard controls to change pitch independently:
               '[' / ']' : decrease/increase LEFT step by one table-index/sample
               'a' / 'd' : decrease/increase RIGHT step by one table-index/sample
               'q' : quit
            */
            if (c == '[' && step_left > 0x10000) step_left -= 0x10000;
            if (c == ']' && step_left < (SINE_WAVE_TABLE_LEN / 16) * 0x20000) step_left += 0x10000;
            if (c == 'a' && step_right > 0x10000) step_right -= 0x10000;
            if (c == 'd' && step_right < (SINE_WAVE_TABLE_LEN / 16) * 0x20000) step_right += 0x10000;
            if (c == 'q') break;

            double freqL = compute_freq_from_step(step_left, sample_rate);
            double freqR = compute_freq_from_step(step_right, sample_rate);
            printf("vol=%d  stepL=%d stepR=%d  freqL=%.2fHz freqR=%.2fHz      \r",
                   vol, step_left >> 16, step_right >> 16, freqL, freqR);
        }

        // give_audio_buffer(ap, buffer);
        struct audio_buffer *buffer = take_audio_buffer(ap, true);
        int16_t *samples = (int16_t *) buffer->buffer->bytes;
        // interleaved stereo: L,R,L,R,... 
        for (uint i = 0; i < buffer->max_sample_count; i++) {
            int16_t sl = (vol * sine_wave_table[pos_left >> 16u]) >> 8u;
            int16_t sr = (vol * sine_wave_table[pos_right >> 16u]) >> 8u;
            samples[i * 2] = sl;
            samples[i * 2 + 1] = sr;
            pos_left  += step_left;
            pos_right += step_right;
            if (pos_left  >= pos_max) pos_left  -= pos_max;
            if (pos_right >= pos_max) pos_right -= pos_max;
        }
        buffer->sample_count = buffer->max_sample_count;
        give_audio_buffer(ap, buffer);
    }

    puts("\n");
    return 0;
}