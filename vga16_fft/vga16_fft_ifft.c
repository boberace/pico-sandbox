/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green lo-bit |__ both wired to VGA Green 
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green hi_bit |   
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 * Protothreads v1.3
 * Threads:
 * core 0:
 * Analog i/o ISR
 * graphics annotation
 * core 1:
 * FFT and iFFT 
 * Serial i/o 
 * 
 * 50% overlap pseudocode
 * https://dsp.stackexchange.com/questions/48768/real-time-overlapping-buffer-for-fft
  repeat forever
    b = readSamples(1024)
    c = concat(a, b)
    fft(c)
    a = b
end repeat

Actual time to do 512  FFT+log+mag+plot is 2.8 mSec.
Frame time for 50% overlap is 20 mSec at 12.8 kHz sample rate.

So 512 FFT with 50% overlap would run real-time at 40 kHz!

 */
// ==========================================
// === VGA graphics library
// ==========================================
#include "vga16_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// pico specific
#include "pico/stdlib.h"
#include "hardware/pio.h"

//#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/regs/timer.h"
//
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "string.h"
// protothreads header
#include "pt_cornell_rp2040_v1_3.h"

#pragma region fp_s1x14
// ==========================================
// === fixed point s1x14 ===
// ==========================================
// s1.14 format
// == resolution 2^-14 = 6.1035e-5
// == dynamic range is +1.9999/-2.0
typedef signed short s1x14 ;
#define muls1x14(a,b) ((s1x14)((((int)(a))*((int)(b)))>>14)) 
#define float_to_s1x14(a) ((s1x14)((a)*16384.0)) // 2^14
#define s1x14_to_float(a) ((float)(a)/16384.0)
#define abss1x14(a) abs(a) 
#define divs1x14(a,b) ((s1x14)((((signed int)(a)<<14)/(b)))) 
// shift 12 bits into 13 bits so full scale ADC is about 0.25
#define adc_to_s1x14(a) ((s1x14)((a)<<1))
// unity in s1x14
#define s1x14_one 16384

#pragma endregion

#pragma region fp_prothreads_globals
// ==========================================
// === protothreads globals
// ==========================================

// data semaphore to control fft on core 1
struct pt_sem run_fft_s, finish_fft_s, data_ready_s ;

// protection for printing
//spin_lock_t * lock_stdout ;

// unprotected global to halt display
static int run_stop = 1 ;
// turn on compressed record, decompressed playback
int record, playback ;

#pragma endregion

#pragma region start_message
// ==========================================
// === set up start-up message ===
// ==========================================
void start_message(char* app_name){
  // get file path and parse to find file name
  char pname[256] = __FILE__ ;

  // get file name
  // Find the last occurrence of '/'
  // === This if/then/else block was written by Goggle AI ===
  char *fname;
  fname = strrchr(pname, '/');
  // If '/' is found, increment pointer to start of filename
  if (fname != NULL) {
      fname++;
  } else {
      // If '/' is not found, the entire string is the filename
      fname = pname;
  }
  // === end AI ===
  //
  printf("\n\nCompiled program: ");
  printf("%s\n", fname);
  //
  // // get the windows user name from the path
  // char* token ;
  // token = strtok(pname, "/");
  // while(strcmp(token,"Users")){
  //   token = strtok(NULL, "/");
  // }
  // token = strtok(NULL, "/");
  // printf("Windows User name: ") ;
  // printf("%s\n", token) ;
  // //
  // // get time date
  // printf("Compile time/date: ");
  // printf("%s  %s\n", __TIME__, __DATE__);
  //
  // anything else you wish to print
  printf("Protothreads: RP2350 v1.3 two-core, priority\n");
  printf("Application: %s\n\n", app_name) ;
}
#pragma endregion

#pragma region timer_ISR

// //
// ==========================================
// === set up timer ISR  used in this pgm
// ==========================================
// === timer alarm ========================
// !! modifiying alarm zero trashes the cpu 
//        and causes LED  4 long - 4 short
// !! DO NOT USE alarm 0
// This low-level setup is ocnsiderably faster to execute
// than the hogh-level callback

#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)
// ISR interval will be 78 uSec
// 12.8 KHz
volatile int alarm_period = 78 ; //78 ; //125
//float Fs = 12800 ;
//
// the actual ISR
void compute_sample(void);
//
static void alarm_irq(void) {
    // mark ISR entry
    gpio_put(15,1);
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    // arm the next interrupt
    // Write the lower 32 bits of the target time to the alarm to arm it
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + alarm_period ;
    //
    compute_sample();

    // mark ISR exit//
    gpio_put(15,0);
}

// set up the timer alarm ISR
static void alarm_in_us(uint32_t delay_us) {
    // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Set irq handler for alarm irq
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm irq
    irq_set_enabled(ALARM_IRQ, true);
    // Enable interrupt in block and at processor
    // Alarm is only 32 bits 
    uint64_t target = timer_hw->timerawl + delay_us;
    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[ALARM_NUM] = (uint32_t) target;   
}
#pragma endregion

#pragma region r2r_dac
void r2r_dac_out(uint32_t value){
  gpio_put_masked(0xFF, value);
}
void setup_r2r() 
{
    gpio_init_mask(0xFF); // Initialize GPIO pins 0-7 for output
    gpio_set_dir_out_masked(0xFF); // Set GPIO pins 0-7 as outputs   
}
#pragma endregion

#pragma region ADC_setup
// ===========================================
// ADC setup 
// ===========================================
// setup ADC
#define adc_array_length 512 //512
#define adc_half_length 256 //256
// raw from ADC
short adc_data[adc_array_length] ;
int adc_data_count = adc_half_length ;
// copy of input data
short analysis_data[adc_array_length] ;
// formatting for scope trace
short display_data[adc_array_length] ;
short fft_display_data[adc_array_length] ;
short log_display_data[adc_array_length] ;
// reconstructed wave from iFFT

int adc_offset = 2000 ;

void ADC_setup(void){
  adc_init();
  adc_gpio_init(28);
  // p26 is ADC input 0 - gpio 28 is 2
  adc_select_input(2);
  // free run
  adc_run(1);
  // result is in adc_hw->result
}
#pragma endregion

#pragma region FFT

// ===========================================
// dSP definitions 
// ===========================================
//

// === FFT setup
#define N_WAVE          adc_array_length    /* size of FFT 512 */
#define LOG2_N_WAVE     9     /* log2(N_WAVE) 0 */

s1x14 Sinewave[N_WAVE]; // a table of sines for the FFT
s1x14 window[N_WAVE]; // a table of window values for the FFT
s1x14 window_inv[N_WAVE]; // a table of window values for the iFFT
s1x14 fr[N_WAVE], fi[N_WAVE]; // input data
s1x14 ifr[N_WAVE], ifi[N_WAVE]; // ifft input data
// ifft de-windowing
s1x14 ifft_out[N_WAVE/2];
signed short ifft_play_buffer[N_WAVE/2] ;
int out_count = 0 ;

// ==================================
// === Init FFT arrays
//====================================
void FFTinit(void){
// one cycle sine table
  //  required for FFT
  for (int ii = 0; ii < N_WAVE; ii++) {
    // one cycle per window for FFT -- scall amp for number of bits
    Sinewave[ii] = float_to_s1x14(0.5 * sin(6.283 * ((float) ii) / N_WAVE));
    // Raised cos window
    window[ii] = float_to_s1x14(1.0 - cos(6.283 * ((float) ii) / (N_WAVE - 1)));
    // Hann
    //window[ii] = float_to_s1x14(pow(sin(3.142 * ((float) ii) / (N_WAVE)), 2));
    
  }
  /*
  for (int ii = 0; ii < N_WAVE/2; ii++) {
    //iffT window
    float n = 20 ;
    float fii = ii ;
    window_inv[ii] = (fii<=n)? float_to_s1x14(0.5 - 0.5*cos(6.283 * (float) ii / n)) : float_to_s1x14(1.0) ;
    window_inv[N_WAVE-1-ii] = window_inv[ii] ;
  }
  */
}

// ==================================
// === FFT
//====================================
void FFTfix(s1x14 fr[], s1x14 fi[], int m){
//Adapted from code by:
//Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
//fr[n],fi[n] are real,imaginary arrays, INPUT AND RESULT.
//size of data = 2**m
// This routine does foward transform only

  int mr,nn,i,j,L,k,istep, n;
  s1x14 qr,qi,tr,ti,wr,wi;

  mr = 0;
  n = 1<<m;   //number of points
  nn = n - 1;

  /* decimation in time - re-order data */
  for(m=1; m<=nn; ++m){
      L = n;
      do L >>= 1; while(mr+L > nn);
      mr = (mr & (L-1)) + L;
      if(mr <= m) continue;
      tr = fr[m];
      fr[m] = fr[mr];
      fr[mr] = tr;
      ti = fi[m];   //for real inputs, don't need this
      fi[m] = fi[mr]; //for real inputs, don't need this
      fi[mr] = ti; //for real inputs, don't need this
  }

  L = 1;
  k = LOG2_N_WAVE-1;
  while(L < n) {
      istep = L << 1;
      for(m=0; m<L; ++m){
          j = m << k;
          wr =  Sinewave[j+N_WAVE/4];
          wi = -Sinewave[j];
          //wr >>= 1; //dont need if scale table
          //wi >>= 1;

          for(i=m; i<n; i+=istep){
              j = i + L;
              tr = muls1x14(wr, fr[j]) - muls1x14(wi, fi[j]);
              ti = muls1x14(wr, fi[j]) + muls1x14(wi, fr[j]);
              qr = fr[i] >> 1;
              qi = fi[i] >> 1;
              fr[j] = qr - tr;
              fi[j] = qi - ti;
              fr[i] = qr + tr;
              fi[i] = qi + ti;
          }
      }
      --k;
      L = istep;
  }
}

// ==================================
// === iFFT
//====================================
void iFFTfix(s1x14 fr[], s1x14 fi[], int m){
//Adapted from code by:
//Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
//fr[n],fi[n] are real,imaginary arrays, INPUT AND RESULT.
//size of data = 2**m
// This routine does foward transform only

  int mr,nn,i,j,L,k,istep, n;
  s1x14 qr,qi,tr,ti,wr,wi;

  mr = 0;
  n = 1<<m;   //number of points
  nn = n - 1;

  /* decimation in time - re-order data */
  for(m=1; m<=nn; ++m){
      L = n;
      do L >>= 1; while(mr+L > nn);
      mr = (mr & (L-1)) + L;
      if(mr <= m) continue;
      tr = fr[m];
      fr[m] = fr[mr];
      fr[mr] = tr;
      ti = fi[m];   //for real inputs, don't need this
      fi[m] = fi[mr]; //for real inputs, don't need this
      fi[mr] = ti; //for real inputs, don't need this
  }

  L = 1;
  k = LOG2_N_WAVE-1;
  while(L < n) {
      istep = L << 1;
      for(m=0; m<L; ++m){
          j = m << k;
          wr =  Sinewave[j+N_WAVE/4];
          wi = -Sinewave[j];
          //wr >>= 1; //dont need if scale table
          //wi >>= 1;

          for(i=m; i<n; i+=istep){
              j = i + L;
              tr = muls1x14(wr, fr[j]) - muls1x14(wi, fi[j]);
              ti = muls1x14(wr, fi[j]) + muls1x14(wi, fr[j]);
              qr = fr[i] ; //>> 1;
              qi = fi[i] ; //>> 1;
              fr[j] = qr - tr;
              fi[j] = qi - ti;
              fr[i] = qr + tr;
              fi[i] = qi + ti;
          }
      }
      --k;
      L = istep;
  }
}

//====================================
// === magnitude approx good to about +/-2%
// see https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
//====================================
void magnitude(s1x14 fr[], s1x14 fi[], int length){
  s1x14 mmax, mmin ;
  s1x14 c1 = float_to_s1x14(0.89820) ;
  s1x14 c2 = float_to_s1x14(0.48597) ;
  for (int ii = 0; ii < length; ii++) {
        mmin = min(abs(fr[ii]), abs(fi[ii])); //>>9
        mmax = max(abs(fr[ii]), abs(fi[ii]));
        // reuse fr to hold magnitude
        fr[ii] = max(mmax, (muls1x14(mmax,c1) + muls1x14(mmin,c2) )); 
        fi[ii] = 0;
      }
}
//====================================
// log2 approx
// see:
    // Generation of Products and Quotients Using Approximate Binary Logarithms 
    // for Digital Filtering Applications, 
    // IEEE Transactions on Computers 1970 vol.19 Issue No.02
//====================================
/*
void log2_approx0(s1x14 fr[], int length){
  s15x16 log_input, log_output ;
  // reduced range variable for interpolation
  s15x16 x;
  // low cutoff
  s15x16 low_cutoff = float_to_s15x16(0.00006) ;
  s15x16 c1 = float_to_s15x16(0.984) ;
  s15x16 c2 = float_to_s15x16(0.065) ;


  for (int ii = 0; ii < length; ii++) {
    log_input = s1x14_to_s15x16(fr[ii]) ;    
    //
    // check for too small or negative
    // and return smallest log2
    if(log_input <= low_cutoff){
      fr[ii] = -15 ;
      continue ;
    }
    // if the input is less than 2 the scale up by
    // 2^14 so always working on an integer
    // so we can get logs down to input of 0.00003 or so approx -14.85
    int frac_factor = 0 ;
    if (log_input < int_to_s15x16(2) ){
      // max size of shift to not overflow
      frac_factor = 14 ;
      log_input <<= frac_factor ;
    }

    // temp for finding msb
    s15x16 sx ;  
    sx = log_input ;

    // find the most-significant bit
    // equivalent to finding the characteristic of the log
    s15x16 y=1; // value of MSB
    s15x16 ly=0; // position of MSB
    while(sx>int_to_s15x16(2)) {
        y=y<<1 ; ly=ly+int_to_s15x16(1) ; sx=sx>>1;
    }
    // bound the bottom and detect negative input values
    // Two-segment approx is good to better than  0.02 log unit
    // equiv to finding the mantissa of the log, then adding the charastic
    // see:
    // Generation of Products and Quotients Using Approximate Binary Logarithms 
    // for Digital Filtering Applications, 
    // IEEE Transactions on Computers 1970 vol.19 Issue No.02
    // normalize the bits after dleting MSB
    x = (log_input-y)>>(int)ly ;
    // piecewise linear curve fit
    //if(x<0.5) log_output = (ly + x*1.163 + 0.0213) - frac_factor ;
    //else      log_output = (ly + x*0.828 + 0.1815) - frac_factor ;
    // one segment approx goodd to about 0.07 log unit
    log_output = (ly + x*c1 + c2) - frac_factor ;
    // and store it
    fr[ii] = log_output>>16 ;
  // 
  }
}
*/

#define log_min 0x00  
// reuse fr to hold log magnitude
// shifting finds most significant bit
// then make approxlog  = ly + (fr-y)./(y) + 0.043;
// BUT for an 8-bit approx (4 bit ly and 4-bit fraction)
// ly 1<=ly<=14
// omit the 0.043 because it is too small for 4-bit fraction
void log2_approx(s1x14 fr[], int length) {
  //
  int sx, y, ly, temp ;
  for (int i = 0; i < length; i++) {
    // interpret bits as integer
    sx = fr[i];
    y=1; ly=0;
    while(sx>1) {
        y=y*2; ly=ly+1; sx=sx>>1;
    }
    // shift ly into upper 4-bits as integer part of log
    // take bits below y and shift into lower 4-bits
    // !!NOTE that fr is no longer in s1x14 format!!
    fr[i] = ((ly)<<4) + ((fr[i]-y)>>(ly-4) ) ;
    // bound the noise at low amp
    if(fr[i]<log_min) fr[i] = log_min;
  }
}

#pragma endregion

#pragma region spi_dac_setup
// ========================================
// === spi setup 
// =======================================
//SPI configurations
#define PIN_CS   9
#define PIN_SCK  10
#define PIN_MOSI 11
#define SPI_PORT spi1

// constant to tell SPI DAC what to do
// prepend to each 12-bit sample
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000
uint16_t DAC_data ; 
void spi_setup(void){
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    // connected to spi DAC
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);
    // Map SPI signals to GPIO ports
    //gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
}
#pragma endregion

// ==================================================
// === graphics demo -- RUNNING on core 0
// ==================================================
static PT_THREAD (protothread_graphics(struct pt *pt)) {
    PT_BEGIN(pt);

    // DMA channel -- needed to check for DMA done
    //static int ADC_data_chan ;

    // the protothreads interval timer
    PT_INTERVAL_INIT() ;

    // Draw some filled rectangles
    fillRect(20, 1, 400, 50, LIGHT_BLUE); // blue box
    setCursor(100, 20) ;
    setTextColor(WHITE) ;
    writeStringBig("FFT/iFFT") ;

    fillRect(435, 1, 200, 50, YELLOW); // green box
    // Write some text
    setTextColor(BLACK) ;
    short text_x = 445 ;
    setCursor(text_x-1, 2) ;
    setTextSize(1) ;
    writeStringBold("Raspberry Pi Pico2: ece4760") ;
    setCursor(text_x, 12) ;
    writeString("Bruce Land: BRL4@cornell.edu") ;
    setCursor(text_x, 22) ;
    writeString("Hunter Adams:VHA3@cornell.edu") ;
    setCursor(text_x, 32) ;
    writeString("Protothreads rp2350 v1.3") ;
    setCursor(text_x, 42) ;
    writeString("16 color VGA") ;

    setCursor(80, 122) ;
    setTextSize(1) ;
    setTextColor2(BLACK, YELLOW);
    //writeString(" Spectrum 0-6.4 KHz ") ;
    setCursor(350, 122) ;
    //writeString(" Log Spectrum 0-6.4 KHz ") ;

    drawHLine(0,479,10, WHITE);
    setCursor(1, 469) ;
    setTextSize(1) ;
    setTextColor2(WHITE, BLACK);
    writeString("0");
    //
    drawHLine(0,479-40,10, WHITE);
    setCursor(1, 469-40) ;
    writeString("1k");
    //
    drawHLine(0,479-80,10, WHITE);
    setCursor(1, 469-80) ;
    writeString("2k");
    //
    drawHLine(0,479-120,10, WHITE);
    setCursor(1, 469-120) ;
    writeString("3k");
    //
    drawHLine(0,479-160,10, WHITE);
    setCursor(1, 469-160) ;
    writeString("4k");

    //drawHLine(0,479-175,640, GREEN);
    setCursor(200, 469-170) ;
    setTextColor2(WHITE, BLACK);
    //writeStringBig("FFT Spectogram -- n_bands = 160");
    setCursor(200, 469-350) ;
    setTextColor2(WHITE, BLACK);
   // writeStringBig("Filter Bank Spectrogram -- n_bands = 32");


    // === congure the ADC =========
    // ===  and start a buffer fill
    ADC_setup() ;
    
    //for(int i=0; i<adc_array_length-1; i++){
        // init  initial dispplay points
    //    display_data[i] = 120 ;
    //}

    while(true) {
        
        // A brief nap
        PT_YIELD_usec(1000000) ;
   }
   PT_END(pt);
} // graphics thread/

// ==================================================
// === toggle25 thread on core 0
// disabled because of noise to ADC
// ==================================================
// the on-board LED blinks
static PT_THREAD (protothread_toggle25(struct pt *pt))
{
    PT_BEGIN(pt);
    static bool LED_state = false ;
    
     // set up LED p25 to blink
     gpio_init(25) ;	
     gpio_set_dir(25, GPIO_OUT) ;
     gpio_put(25, true);
     // data structure for interval timer
     PT_INTERVAL_INIT() ;

      while(1) {
        // yield time 0.1 second
        //PT_YIELD_usec(100000) ;
        PT_YIELD_INTERVAL(500000) ;

        // toggle the LED on PICO
        LED_state = LED_state? false : true ;
        gpio_put(25, LED_state);
        //
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // blink thread


// ==================================================
// === fft thread -- RUNNING on core 1
// ==================================================
//  
static PT_THREAD (protothread_fft(struct pt *pt))
{
    PT_BEGIN(pt);
     static short time_column = 12 ;
     static short fr_disp ;
     static short color_map[7] ={0, 1, 2, 3, 11, 15, 15};
     static uint64_t start_time ;
    
     PT_INTERVAL_INIT() ;

     FFTinit();
     spi_setup();
     setup_r2r();

      while(1) {
        //
        // yield until ADC buffer full
        PT_YIELD_UNTIL(pt, run_stop);
        PT_SEM_SAFE_WAIT(pt, &data_ready_s) ;

        // get ADC buffer (restart is done in iSR)
        memcpy(analysis_data, adc_data, adc_array_length*2) ;
        // move second half to first half to cause 50% overlap
        memcpy(&adc_data[0], &adc_data[adc_half_length], adc_half_length*2) ;

        start_time = PT_GET_TIME_usec() ;


        // plot the raw waveform
        for(int i=0; i<adc_array_length; i++){
            //erase a point
            drawPixel(i+40, display_data[i], BLACK) ;
            display_data[i] = -(short)(analysis_data[i]>>4) + 120 ;
            drawPixel(i+40, display_data[i], RED) ;
        }
        
        // === do the windowing
        for (int i = 0; i < N_WAVE; i++) {
          fr[i] = muls1x14(adc_to_s1x14(analysis_data[i]), window[i]); 
          fi[i] = 0;
        }

        // ==================================================
        // === do the FFT
        FFTfix(fr, fi, LOG2_N_WAVE);

        // set up ifft
        for (int i = 0; i < N_WAVE; i++) {
          ifr[i] = fr[i]; 
          ifi[i] = fi[i];
        } 

        // ===============================
        // normally there would be some spectral manipulation here
        // but this example is just inverting back to original waveform

        // ================================
        // do iFFT
        iFFTfix(ifr, ifi, LOG2_N_WAVE);

        // === do the output windowing
        // for (int i = 0; i < N_WAVE; i++) {
        //   ifr[i] = muls1x14((ifr[i]),  (window[i])); 
        //  // ifi[i] = 0;
        // }

        // add first half ifr to ifft_out: s1x14 ifft_out[N_WAVE/2]
        // should be mostly real
        for (int i = 0; i < N_WAVE/2; i++) {
          ifft_out[i] += ifr[i];   
          // then scale and offset for DAC 
          // !!! (scale and offset depends on source voltage)
          ifft_play_buffer[i] = (ifft_out[i]<<0) + adc_offset ;
        } 
        // ifft_play_buffer is used by ISR for reconstruction staert with  out_count = 0
        out_count = 0 ;
        // mmemcpy second half of ifr to ifft_out
        memcpy(&ifft_out[0], &ifr[N_WAVE/2], sizeof(ifft_out)) ;
      
        // ================================
        // === plot ifft waveform
       for(int i=0; i<adc_array_length; i++){
            //erase a point
            drawPixel(i+40, fft_display_data[i], BLACK) ;
            fft_display_data[i] = -(short)(ifr[i]>>3) + 200 ;
            drawPixel(i+40, fft_display_data[i], GREEN) ;      
        }
 
        //
        // === compute power spectrum
        magnitude(fr, fi, N_WAVE);

        // === plot single FFT spectrum for testing
      //  for(int i=2; i<adc_array_length/2; i++){
      //     //erase a point
      //    drawPixel(i+40, fft_display_data[i], BLACK) ;
      //    fft_display_data[i] = -(short)(fr[i]>>4) + 120 ;
      //    drawPixel(i+40, fft_display_data[i], GREEN) ;
      //   }

        // === plot log-spectrum for testing
        // !!After log, the returned fr is no longer in s1x14 format!!
        // do the APPROXIMATE log in u4x4 format!
        log2_approx(fr, adc_array_length/2) ;
        // plot 
        
        // === plot fft spectrogram
        // Spectrogram -- draw and move right
        // wrap the screen
        // at right edge of screen, reset to left edge
        time_column++ ;
        if (time_column == 636) time_column = 12;
        //drawHLine(0, 80,5,WHITE) ;
        for(int i=4; i<165; i++){
          // bound to 0 to 15
          fr[i] = max(fr[i], 64) - 64;
          fr_disp = color_map[min(7, max((fr[i]>>4)-1, 0))] ; //fr[i]>>3)-3
          drawPixel(time_column, 480-i, fr_disp ) ;
          //drawVLine(time_column, 480-2*i, 2, fr_disp) ;
        } 

      } // END WHILE(1)
  PT_END(pt);
} // blink thread

// ==================================================
// === user's serial input thread on core 1
// ==================================================
// serial_read an serial_write do not block any thread
// except this one
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input record playback: ");
        // spawn a thread to do the non-blocking write
        serial_write ;

        // spawn a thread to do the non-blocking serial read
         serial_read ;
         
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d %d", &record, &playback) ;
        

        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // serial thread

// ==================================================
// === dsp ISR -- 
// ==================================================
// 

void compute_sample(void){
    // read and subtrace zero point
    // fill buffer
    adc_data[adc_data_count] = (adc_hw->result) - adc_offset;

    gpio_put(14,0);
    
    // check buffer filling
    if (adc_data_count == adc_array_length-1){
        adc_data_count = adc_half_length ;
        if (run_stop) PT_SEM_SAFE_SIGNAL(pt, &data_ready_s) ;
    }
    else adc_data_count++;

    //output the iFFT to the DAC and increment the buffer counter
    DAC_data = ifft_play_buffer[out_count] ;

    uint8_t DAC_data_8b = (DAC_data >> 4) & 0xff ;  
    r2r_dac_out(DAC_data_8b);
    

    DAC_data = (DAC_config_chan_A | ((DAC_data) & 0xfff))  ; 
    if (out_count < N_WAVE/2-1) out_count++ ;
    
    // spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;
    // NOTE === nonblocking SPI write ===tempo 4
    spi0_hw->dr = DAC_data ;

      gpio_put(14,1);
}

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){ 
  //
  //  === add threads  ====================
  // for core 1
  pt_add_thread(protothread_fft) ;
  pt_add_thread(protothread_serial) ;
  //
  // === initalize the scheduler ==========
  pt_schedule_start ;
  // NEVER exits
  // ======================================
}

// ========================================
// === core 0 main
// ========================================
int main(){
  sleep_ms(10);
  // set the clock
  //set_sys_clock_khz(250000, true); // 171us
  // start the serial i/o
  stdio_init_all() ;
  // announce the threader version on system reset
  //printf("\n\rProtothreads RP2040 v1.1.3 two-core\n\r");
  char app_name[] = "FFT/iFFT test";
  start_message(app_name);

  // Initialize the VGA screen
  initVGA() ;

  // init the global fft ready signals
  PT_SEM_SAFE_INIT(&run_fft_s, 1) ;
  PT_SEM_SAFE_INIT(&finish_fft_s, 1) ;
  PT_SEM_SAFE_INIT(&data_ready_s, 0) ;
  
  // anybody can print first
  //PT_LOCK_INIT(lock_stdout, 31, 0) ;
     
  //
  // fire off interrupt
  alarm_in_us(alarm_period);
  

  gpio_init(15) ;	
   gpio_set_dir(15, GPIO_OUT) ;
   gpio_init(14) ;	
   gpio_set_dir(14, GPIO_OUT) ;

  // start core 1 threads
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);
  sleep_ms(10);



  // === config threads ========================
  // for core 0
 //
  pt_add_thread(protothread_graphics);
  pt_add_thread(protothread_toggle25);
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;
  // NEVER exits
  // ===========================================
} // end main