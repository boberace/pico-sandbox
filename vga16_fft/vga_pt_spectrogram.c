/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Green lo-bit |__ both wired to 150 ohm to ground 
 *  - GPIO 19 ---> 220 ohm resistor ---> VGA Green hi_bit |   and to VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 * Protothreads v1.1.1
 * Threads:
 * core 0:
 * Graphics demo
 * blink LED25 
 * core 1:
 * Toggle gpio 4 
 * Serial i/o 
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
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

// // Our assembled programs:
// // Each gets the name <pio_filename.pio.h>
// #include "hsync.pio.h"
// #include "vsync.pio.h"
// #include "rgb.pio.h"

#pragma region protothreads
// ==========================================
// === protothreads globals
// ==========================================
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "string.h"
// protothreads header
#include "pt_cornell_rp2040_v1_3.h"

// data semaphore to control fft on core 1
struct pt_sem run_fft_s, finish_fft_s ;

// protection for printing
spin_lock_t * lock_stdout ;

// unprotected global to halt display
static int run_stop = 1 ;
#pragma endregion

#pragma region timer
// ==========================================
// Some globals for storing timer information
// ==========================================
volatile unsigned int time_accum = 0;
unsigned int time_accum_old = 0 ;
char timetext[40];

// Timer interrupt
bool repeating_timer_callback(struct repeating_timer *t) {

    time_accum += 1 ;
    return true;
}
#pragma endregion

#pragma region ADC_DMA
// ===========================================
// ADC setup + DMA channel for ADC
// ===========================================
// setup ADC
#define adc_array_length 512
// raw from ADC
short adc_data[adc_array_length] ;
// formatted for scope trace
short display_data[adc_array_length] ;
short fft_display_data[adc_array_length] ;
short log_display_data[adc_array_length] ;
// copy of  data
short analysis_data[adc_array_length] ;
// copy of  data
short fft_data[adc_array_length] ;

int ADC_setup(void){
  adc_init();
  adc_gpio_init(28);
  // p26 is ADC input 0
  adc_select_input(2);
  //  48 MHz clock / 12.8 KHz sample rate
  //adc_set_clkdiv (3750);
  // 16 KHz  32 mSec window
  adc_set_clkdiv (3000);
  // free run
  adc_run(1);
  // result is in adc_hw->result
  // but we are going to forward that to the FIFO
  //adc_fifo_setup(bool en, bool dreq_en, uint16_t dreq_thresh, bool err_in_fifo, bool byte_shift)
  // fifo_enable, dreq_enable, thresh=1, no error, dont shift to 8-bits(for 8-bit PWM)
  adc_fifo_setup(1,1,1,0,0);
  //
  // the DMA channel
  int ADC_data_chan = 11 ; //
  // Conflict with video gen code:
  //int ADC_data_chan = dma_claim_unused_channel(true);
  // The acdtual data channel
   dma_channel_config c2 = dma_channel_get_default_config(ADC_data_chan);
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    channel_config_set_irq_quiet(&c2, true);
    channel_config_set_enable(&c2, true); 
    //channel_config_set_chain_to(&c2, ctrl_chan) ;
    channel_config_set_dreq(&c2, DREQ_ADC);
    //
    dma_channel_configure(ADC_data_chan, &c2, 
        adc_data, // write_addr, to array
        &adc_hw->fifo,  // read_addr, the table
        adc_array_length, // one ADC value,
        1) ; // trigger
  return ADC_data_chan ;
}
#pragma endregion

#pragma region fp_s1x14

// ==========================================
// === fixed point s1x14
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

#pragma endregion

#pragma region fp_s15x16
// ==========================================
// === fixed point s15x16
// ==========================================
// s15x16 fixed point macros ==
// == resolution 2^-16 = 1.5e-5
// == dynamic range is 32767/-32768
typedef signed int s15x16;
#define muls15x16(a,b) ((s15x16)(((( signed long long )(a))*(( signed long long )(b)))>>16)) //multiply two fixed 16:16
#define float_to_s15x16(a) ((s15x16)((a)*65536.0)) // 2^16
#define s15x16_to_float(a) ((float)(a)/65536.0)
#define s15x16_to_int(a)    ((int)((a)>>16))
#define int_to_s15x16(a)    ((s15x16)((a)<<16))
#define divs15x16(a,b) ((s15x16)((((signed long long)(a)<<16)/(b)))) 
#define abss15x16(a) abs(a)
// the weird shift is to move the sign bits correctly
#define s1x14_to_s15x16(a) ((s15x16)(a)<<2) ;

#pragma endregion

#pragma region dsp_defs
// ===========================================
// dSP definitions 
// ===========================================
//

// FFT setup
#define N_WAVE          adc_array_length    /* size of FFT 512 */
#define LOG2_N_WAVE     9     /* log2(N_WAVE) 0 */

s1x14 Sinewave[N_WAVE]; // a table of sines for the FFT
s1x14 window[N_WAVE]; // a table of window values for the FFT
s1x14 fr[N_WAVE], fi[N_WAVE]; // input data
#pragma endregion

#pragma region FFT_arrays
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
  }
}
#pragma endregion


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
          //wr >>= 1; //do need if scale table
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

// ==================================================
// === graphics demo -- RUNNING on core 0
// ==================================================
static PT_THREAD (protothread_graphics(struct pt *pt)) {
    PT_BEGIN(pt);

    // DMA channel -- needed to check for DMA done
    static int ADC_data_chan ;

    // the protothreads interval timer
    PT_INTERVAL_INIT() ;

    // Draw some filled rectangles
    fillRect(20, 1, 400, 50, LIGHT_BLUE); // blue box
    fillRect(435, 1, 200, 50, YELLOW); // green box

    // Write some text
    setTextColor(BLACK) ;
    short text_x = 445 ;
    setCursor(text_x, 2) ;
    setTextSize(1) ;
    writeString("Raspberry Pi Pico2") ;
    setCursor(text_x, 12) ;
    writeString("VGA scope demo - ECE 4760") ;
    setCursor(text_x, 22) ;
    writeString("bruce.land@cornell.edu") ;
    setCursor(text_x, 32) ;
    writeString("Protothreads rp2350 v1.3") ;
    setCursor(text_x, 42) ;
    writeString("16 color VGA") ;

    setCursor(80, 252) ;
    setTextSize(1) ;
    setTextColor2(BLACK, YELLOW);
    writeString(" Spectrum 0-8KHz ") ;
    setCursor(350, 252) ;
    writeString(" Log Spectrum 0-8KHz ") ;
    // Setup a 1Hz timer
    //static struct repeating_timer timer;
    //add_repeating_timer_ms(-1000, repeating_timer_callback, NULL, &timer);

    // === congure the ADC =========
    // ===  and start a buffer fill
    ADC_data_chan = ADC_setup() ;
    //

    while(true) {

      // yield until ADC buffer full
      PT_YIELD_UNTIL(pt, run_stop);
      PT_YIELD_UNTIL(pt, !dma_channel_is_busy(ADC_data_chan));
      //dma_channel_wait_for_finish_blocking (ADC_data_chan) ;
      // get ADC buffer and start next buffer fill by reseting DMA source addr
      memcpy(analysis_data, adc_data, adc_array_length*2) ;
      memcpy(fft_data, adc_data, adc_array_length*2) ;
      // 

      // restart DMA from ADC
      dma_channel_set_write_addr (ADC_data_chan, adc_data, true) ;

      // tell core 1 to do fft
      PT_SEM_SAFE_SIGNAL(pt, &run_fft_s);

      // plot time series 
      // void drawLine(short x0, short y0, short x1, short y1, char color) {
      for(int i=0; i<adc_array_length-1; i++){
        // erase a point
        drawPixel(i+40, display_data[i], BLACK) ;
        //drawLine(i+40, display_data[i], i+41, display_data[i+1], BLACK);
        display_data[i] = (analysis_data[i]>>5) + 100 ;
        drawPixel(i+40, display_data[i], GREEN) ;
        //drawLine(i+40, display_data[i], i+41, display_data[i+1], GREEN);
      }

        
        // A brief nap
        //PT_YIELD_usec(10000) ;
   }
   PT_END(pt);
} // graphics thread

// ==================================================
// === toggle25 thread on core 0
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
        PT_YIELD_INTERVAL(100000) ;

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
     static short time_column ;
     static short fr_disp ;
     static short color_map[7] ={0, 1, 2, 3, 7, 11, 15};
    
     PT_INTERVAL_INIT() ;

     FFTinit();

      while(1) {
        //
        PT_SEM_SAFE_WAIT(pt, &run_fft_s) ;
        // compute FFT on ADC buffer and draw one column
        // covert ADC to fixed point
        for (int i = 0; i < N_WAVE; i++) {
          fr[i] =  adc_to_s1x14(analysis_data[i]) ;  //adc_to_s1x14
        }
        
        // do the windowing
        for (int i = 0; i < N_WAVE; i++) {
          fr[i] = muls1x14(fr[i], window[i]); 
          fi[i] = 0;
        }

        absolute_time_t start_time = get_absolute_time();
        // do the FFT
        FFTfix(fr, fi, LOG2_N_WAVE);

        absolute_time_t end_time = get_absolute_time();

        int64_t diff_us = absolute_time_diff_us(start_time, end_time);
        //
        // compute power spectrum
        magnitude(fr, fi, N_WAVE);

        // plot single spectrum for testing
        for(int i=2; i<adc_array_length/2; i++){
          // erase a point
          drawPixel(i+40, fft_display_data[i], BLACK) ;
          fft_display_data[i] = -(short)(fr[i]>>4) + 250 ;
          drawPixel(i+40, fft_display_data[i], GREEN) ;
        }
        // find max of magnitude for freq estimate
        s1x14 max_mag = 0 ;
        int max_mag_index = 0;
        for(int i=2; i<adc_array_length/2; i++){
          // 
          if(fr[i] > max_mag) {
            max_mag = fr[i] ;
            max_mag_index = i ;
          } 
        }
        // print frequency estimate
        char vga_buffer[50] ;
        sprintf(vga_buffer, "fft us = %d  ", diff_us ) ;
        setCursor(25, 8) ;
        setTextSize(1) ;
        setTextColor2(BLACK, LIGHT_BLUE);
        writeString(vga_buffer) ;

        sprintf(vga_buffer, "Freq = %6.1f  ", max_mag_index * 31.25 ) ;
        setCursor(25, 32) ;
        setTextSize(1) ;
        setTextColor2(BLACK, LIGHT_BLUE);
        writeString(vga_buffer) ;

        // plot log-spectrum for testing
        // !!After log, the returned fr is no longer in s1x14 format!!
        // do the APPROXIMATE log in u4x4 format!
        log2_approx(fr, adc_array_length/2) ;
        // plot 
        for(int i=2; i<adc_array_length/2; i++){
          // erase a point
          drawPixel(i+300, log_display_data[i], BLACK) ;
          fr[i] = max(fr[i], 64) - 64;
          log_display_data[i] = -(short)(fr[i]>>1) + 250 ;
          drawPixel(i+300, log_display_data[i], GREEN) ;
        }

        // plot a vertical slice for spectrogram
        // Spectrogram -- draw and move right
        // wrap the screen
        // at right edge of screen, reset to left edge
        time_column++ ;
        if (time_column == 636) time_column = 3;
        for(int i=1; i<200; i++){
          // bound to 0 to 7
          fr_disp = color_map[min(6, max((fr[i]>>3)-3, 0))] ; //4-1
          drawPixel(time_column, 480-i, fr_disp ) ;
          //drawVLine(time_column, 480-2*i, 2, fr_disp) ;
        } 

        // NEVER exit while
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
      
      //
      //for (int i = 0; i < N_WAVE; i++) {
       //   printf("%f \n\r", s1x14_to_float(window[i])) ; //m        
      //}

      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input 0=stop 1=run: ");
        // spawn a thread to do the non-blocking write
        serial_write ;

        // spawn a thread to do the non-blocking serial read
         serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d ", &run_stop) ;


        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // serial thread

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
  // set the clock
  //set_sys_clock_khz(250000, true); // 171us
  // start the serial i/o
  stdio_init_all() ;
  // announce the threader version on system reset
  printf("\n\rProtothreads RP2040 v1.3 two-core\n\r");

  // Initialize the VGA screen
  initVGA() ;

  // init the global fft ready signals
  PT_SEM_SAFE_INIT(&run_fft_s, 1) ;
  PT_SEM_SAFE_INIT(&finish_fft_s, 1) ;
  // anybody can print first
  PT_LOCK_INIT(lock_stdout, 31, 0) ;
     
  // start core 1 threads
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // === config threads ========================
  // for core 0
  pt_add_thread(protothread_graphics);
  pt_add_thread(protothread_toggle25);
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;
  // NEVER exits
  // ===========================================
} // end main