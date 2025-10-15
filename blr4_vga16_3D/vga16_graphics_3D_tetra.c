/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green 
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
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
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// // Our assembled programs:
// // Each gets the name <pio_filename.pio.h>
// #include "hsync.pio.h"
// #include "vsync.pio.h"
// #include "rgb.pio.h"

// ==========================================
// === protothreads globals
// ==========================================
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "string.h"
// protothreads header
#include "pt_cornell_rp2040_v1_3.h"

int new_str = 1;

#define screen_center_x 320
#define screen_center_y 240
#define screen_width 200

// ==========================================
// === fixed point s15x16
// ==========================================
// s15x16 fixed point macros ==
// == resolution 2^-16 = 1.5e-5
// == dynamic range is 32767/-32768
typedef signed int s15x16;
//multiply two fixed 16:16
#define muls15x16(a,b) (((a)==0 | (b)==0)?0: \
                        ((a)==0x10000)? (b): \
                        ((b)==0x10000)? (a): \
                        (s15x16)(((( signed long long )(a))*(( signed long long )(b)))>>16)) 
//#define muls15x16(a,b) ((s15x16)(((( signed long long )(a))*(( signed long long )(b)))>>16)) 
#define float_to_s15x16(a) ((s15x16)((a)*65536.0)) // 2^16
#define s15x16_to_float(a) ((float)(a)/65536.0)
#define s15x16_to_int(a)    ((int)((a)>>16))
#define int_to_s15x16(a)    ((s15x16)((a)<<16))
#define divs15x16(a,b) (((a)==0)?0:(s15x16)((((signed long long)(a)<<16)/(b)))) 
#define abss15x16(a) abs(a)
#define one  0x00010000

// see below for better sqrt
//#define sqrts15x16(a) (float_to_s15x16(sqrt(s15x16_to_float(a)))) 
/*
Algorithm and code Author Christophe Meessen 1993. 
  Initially published in usenet comp.lang.c, Thu, 28 Jan 1993 08:35:23 GMT, 
  Subject: Fixed point sqrt ; by Meessen Christophe
  https://groups.google.com/forum/?hl=fr%05aacf5997b615c37&fromgroups#!topic/comp.lang.c/IpwKbw0MAxw/discussion
  Note: there was a bug in the published sqrtL2L routine. It is corrected in
  this implementation.
  This code from https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
*/
// the squrare root of a fixed point with 16 bit
// fractional part and returns a fixed point with 16 bit fractional part. It 
// requires that v is positive. The computation use only 32 bit registers and 
// simple operations.
s15x16 sqrts15x16(s15x16 v) {
    uint32_t t, q, b, r;
    r = v;
    b = 0x40000000;
    q = 0;
    while( b > 0x40 )
    {
        t = q + b;
        if( r >= t )
        {
            r -= t;
            q = t + b; // equivalent to q += 2*b
        }
        r <<= 1;
        b >>= 1;
    }
    q >>= 8;
    return q;
}

// ==========================================
// === graphics math
// ==========================================
/*/ NEED
vector type
matrix type
VxM
MxM
VxTrans  makeTrans
VxRot    makeRot
VxScale  makeScale
VxCamera makeCamera
Vcat
mag
*/
typedef struct vector_s{
  s15x16 x, y, z ;
} vector;

/////////////////////////////////////////////////////////////////////
// copied from
// https://ece4760.github.io/Projects/Fall2023/av522_dy245/code.html
typedef struct matrix_s {
    s15x16 m11;
    s15x16 m12;
    s15x16 m13;
    s15x16 m14;
    s15x16 m21;
    s15x16 m22;
    s15x16 m23;
    s15x16 m24;
    s15x16 m31;
    s15x16 m32;
    s15x16 m33;
    s15x16 m34;
    s15x16 m41;
    s15x16 m42;
    s15x16 m43;
    s15x16 m44;
} matrix;

void MxM(matrix* A, matrix* B, matrix* prod) {
    prod->m11 = muls15x16(A->m11, B->m11) + muls15x16(A->m12, B->m21) + muls15x16(A->m13, B->m31) + muls15x16(A->m14, B->m41);
    prod->m12 = muls15x16(A->m11, B->m12) + muls15x16(A->m12, B->m22) + muls15x16(A->m13, B->m32) + muls15x16(A->m14, B->m42);
    prod->m13 = muls15x16(A->m11, B->m13) + muls15x16(A->m12, B->m23) + muls15x16(A->m13, B->m33) + muls15x16(A->m14, B->m43);
    prod->m14 = muls15x16(A->m11, B->m14) + muls15x16(A->m12, B->m24) + muls15x16(A->m13, B->m34) + muls15x16(A->m14, B->m44);
    prod->m21 = muls15x16(A->m21, B->m11) + muls15x16(A->m22, B->m21) + muls15x16(A->m23, B->m31) + muls15x16(A->m24, B->m41);
    prod->m22 = muls15x16(A->m21, B->m12) + muls15x16(A->m22, B->m22) + muls15x16(A->m23, B->m32) + muls15x16(A->m24, B->m42);
    prod->m23 = muls15x16(A->m21, B->m13) + muls15x16(A->m22, B->m23) + muls15x16(A->m23, B->m33) + muls15x16(A->m24, B->m43);
    prod->m24 = muls15x16(A->m21, B->m14) + muls15x16(A->m22, B->m24) + muls15x16(A->m23, B->m34) + muls15x16(A->m24, B->m44);
    prod->m31 = muls15x16(A->m31, B->m11) + muls15x16(A->m32, B->m21) + muls15x16(A->m33, B->m31) + muls15x16(A->m34, B->m41);
    prod->m32 = muls15x16(A->m31, B->m12) + muls15x16(A->m32, B->m22) + muls15x16(A->m33, B->m32) + muls15x16(A->m34, B->m42);
    prod->m33 = muls15x16(A->m31, B->m13) + muls15x16(A->m32, B->m23) + muls15x16(A->m33, B->m33) + muls15x16(A->m34, B->m43);
    prod->m34 = muls15x16(A->m31, B->m14) + muls15x16(A->m32, B->m24) + muls15x16(A->m33, B->m34) + muls15x16(A->m34, B->m44);
    prod->m41 = muls15x16(A->m41, B->m11) + muls15x16(A->m42, B->m21) + muls15x16(A->m43, B->m31) + muls15x16(A->m44, B->m41);
    prod->m42 = muls15x16(A->m41, B->m12) + muls15x16(A->m42, B->m22) + muls15x16(A->m43, B->m32) + muls15x16(A->m44, B->m42);
    prod->m43 = muls15x16(A->m41, B->m13) + muls15x16(A->m42, B->m23) + muls15x16(A->m43, B->m33) + muls15x16(A->m44, B->m43);
    prod->m44 = muls15x16(A->m41, B->m14) + muls15x16(A->m42, B->m24) + muls15x16(A->m43, B->m34) + muls15x16(A->m44, B->m44);
}
// end of copied code
/////////////////////////////////////////////////////////////////////
// test matrices
matrix m1={one, 0, 0, 0,
           0, one, 0, 0,
           0, 0, one, 0,
           0, 0, 0, one} ;

matrix m2={one, 2*one, 3*one, 4*one,
           5*one, 6*one, 7*one, 8*one,
           9*one, 10*one, 11*one, 12*one,
           13*one, 14*one, 15*one, 16*one} ;           

matrix p ={one, 0, 0, 0,
           0, one, 0, 0,
           0, 0, one, 0,
           0, 0, 0, one} ;

matrix m3 = {one, 0, 0, 0,
             0, one, 0, 0,
             0, 0, one, 0,
            one, 2*one, 3*one, one};

matrix m4 = {one, 0, 0, 0,
             0, one, 0, 0,
              0, 0, one, one, 
              0, 0, -2*one, 0};
//======================================

// 3 vector times 4x4 matrix returns 3 vector
// last element in vector is implicitly unity
// this works for all transforms EXCEPT camera perspective transform
// returns -1 if illegal w cood
int VxM( vector* v, matrix* mm, vector* prod ){
  s15x16 w ;
  prod->x = muls15x16(mm->m11, v->x) + muls15x16(mm->m12, v->y) + muls15x16(mm->m13, v->z) + (mm->m14) ;
  prod->y = muls15x16(mm->m21, v->x) + muls15x16(mm->m22, v->y) + muls15x16(mm->m23, v->z) + (mm->m24) ;
  prod->z = muls15x16(mm->m31, v->x) + muls15x16(mm->m32, v->y) + muls15x16(mm->m33, v->z) + (mm->m34) ;
  w       = muls15x16(mm->m41, v->x) + muls15x16(mm->m42, v->y) + muls15x16(mm->m42, v->z) + (mm->m44) ;
  if (w <= 0) return -1 ;
  if (w != one){
    prod->x = divs15x16(prod->x, w) ;
    prod->y = divs15x16(prod->y, w) ;
    prod->z = divs15x16(prod->z, w) ;
  }
  return 0 ;
}

// ======
void Vadd(vector* v1, vector* v2, vector* result){
  result->x = v1->x + v2->x ;
  result->y = v1->y + v2->y ; 
  result->z = v1->z + v2->z ;
}

// ======
void Vsub(vector* v1, vector* v2, vector* result){
  result->x = v1->x - v2->x ;
  result->y = v1->y - v2->y ; 
  result->z = v1->z - v2->z ;
}

// ======
// scale a vector
void VxS(vector* v1, s15x16 scale, vector* result){
  result->x = muls15x16(v1->x, scale) ;
  result->y = muls15x16(v1->y, scale) ; 
  result->z = muls15x16(v1->z, scale) ;
}

// ======
s15x16 Vmag(vector* v){
  return sqrts15x16(muls15x16(v->x, v->x) + muls15x16(v->y, v->y) + muls15x16(v->z, v->z)) ;
}

// ======
void Vnorm(vector* v){
   s15x16 mag = sqrts15x16(muls15x16(v->x, v->x) + muls15x16(v->y, v->y) + muls15x16(v->z, v->z)) ;
   v->x = divs15x16(v->x, mag) ;
   v->y = divs15x16(v->y, mag) ;
   v->z = divs15x16(v->z, mag) ;
}

// ======
s15x16 Vdot(vector* v1, vector* v2){
  return (muls15x16(v1->x, v2->x) + muls15x16(v1->y, v2->y) + muls15x16(v1->z, v2->z)) ;
}

// ======
// cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
// cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
// cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
// ======
void Vcross(vector* v1, vector* v2, vector* cross){
  cross->x = muls15x16(v1->y, v2->z) - muls15x16(v1->z, v2->y) ;
  cross->y = muls15x16(v1->z, v2->x) - muls15x16(v1->x, v2->z) ;
  cross->z = muls15x16(v1->x, v2->y) - muls15x16(v1->y, v2->x) ;
}

// ======
// build z rotate -- angle degrees
s15x16 cosine[360], sine[360] ;

void build_RotZ(int angle, matrix* TrotZ){
    TrotZ->m11 = cosine[angle];
    TrotZ->m12 = sine[angle] ;
    TrotZ->m13 = 0;
    TrotZ->m14 = 0;
    TrotZ->m21 = -sine[angle] ;
    TrotZ->m22 = cosine[angle] ;
    TrotZ->m23 = 0 ;
    TrotZ->m24 = 0 ;
    TrotZ->m31 = 0 ;
    TrotZ->m32 = 0 ;
    TrotZ->m33 = one ;
    TrotZ->m34 = 0 ;
    TrotZ->m41 = 0 ;
    TrotZ->m42 = 0 ;
    TrotZ->m43 = 0 ;
    TrotZ->m44 = one ;
}

// ======
/*
// build view matrix
void build_Tview(vector* from, vector* to, vector* approxUp, s15x16 proj_d, vector* Tresult){
  // view direction: N = LookTo - LookFrom (normalized) 
  // approx UP: V' convert to true UP: V = V'- (V' dot N) N (normalized) 
  // screen-right U = N cross V (normalized) 
  // camera location: cam_x, cam_y, cam_z == LookFrom
  // the projection distance (LookFrom to screen) proj_d
  vector N, V, U ;
  vector temp1, temp2 ;
  // get N
  Vsub(to, from, &N);
  Vnorm(&N) ;
  // get V
  Vnorm(approxUp) ;
  Vdot(approxUp, &N)
  
  Vsub(approxUp, )

}
*/


// =================
// test view/persp matrices
// cam=2,0,0; N=-1,0,0; V=0,1,0, U=0,0,-1
// looking toward the origin from x=2 with up in y direction, with screen-right along neg z
matrix Tview = {0, 0, -one, 0, 
                0, one, 0, 0,
                -one, 0, 0, 0,
                0, 0, 2*one, one} ;

// d=1, h=200 pixels, 
matrix Tpersp = {one, 0, 0, 0,
                0, one, 0, 0,
                0, 0, one, one,
                0, 0, -one, one} ;
 // =================              

matrix Tcamera ;

// some objects
typedef struct tetrahedron_s{ 
  int N_vertex ;
  int N_face ;
  vector vertex[4] ;
  int face[4][3] ;
  } tetrahedron ;

tetrahedron tetra1 = {
  4, 4,
  {one,one,one,  one,-one,-one,  -one,-one,one, -one,one,-one},
  {0,2,1,  0,1,3,  0,3,2,  1,2,3}
} ;

//void clear_frame(void){
//  fillRect(200, 140, 200, 200, BLACK);
//}

// ==================================================
// === graphics demo -- RUNNING on core 0
// ==================================================
static PT_THREAD (protothread_graphics(struct pt *pt)) {
    PT_BEGIN(pt);
    // the protothreads interval timer
    PT_INTERVAL_INIT() ;

    // Draw some filled rectangles
    fillRect(64, 0, 176, 50, BLUE); // blue box
    //fillRect(250, 0, 176, 50, DARK_ORANGE); // red box
    fillRect(435, 0, 176, 50, LIGHT_BLUE); // green box

    // Write some text
    setTextColor(WHITE) ;
    setCursor(65, 0) ;
    setTextSize(1) ;
    writeString("Raspberry Pi Pico") ;
    setCursor(65, 10) ;
    writeString("3D demo") ;
    setCursor(65, 20) ;
    writeString("Hunter Adams") ;
    setCursor(65, 30) ;
    writeString("Bruce Land") ;
    setCursor(65, 40) ;
    writeString("ece4760 Cornell") ;
    //
    setCursor(445, 10) ;
    setTextColor(BLACK) ;
    setTextSize(1) ;
    writeString("Protothreads rp2040 v1.3") ;
    setCursor(445, 20) ;
    writeString("VGA 4-bit color") ;
    setCursor(445, 30) ;
    writeString("Tested on pico") ;
    //
    setCursor(270, 20) ;
    setTextColor2(WHITE, BLACK) ;
    writeStringBig("3D Render Test") ;
    setCursor(270, 37) ;
    //setTextColor2(WHITE, BLACK) ;
   //writeStringBold("GLCD BOLD Font Test") ;
    
    static vector view_vertex[4], tetra_rot_vertex[4] ;
    static matrix TrotZ ;
    static int t;

    while(true) {

        PT_YIELD_UNTIL(pt, new_str) ;
        new_str = false ;
        
        MxM(&Tview, &Tpersp, &Tcamera) ;
        for (t=0; t<1000; t++){
          build_RotZ(t%360, &TrotZ);
          //printf("%d\n\r", t);
          clear_frame(50, 250) ;
          for (int i=0; i<4; i++){
            VxM(&tetra1.vertex[i], &TrotZ, &tetra_rot_vertex[i]); //
            VxM(&tetra_rot_vertex[i], &Tcamera, &view_vertex[i]); //&Tcamera
            //printf("%f %f  \n\r", s15x16_to_float(view_vertex[i].x), s15x16_to_float(view_vertex[i].y)) ;
            view_vertex[i].x = muls15x16(int_to_s15x16(screen_width), view_vertex[i].x) + int_to_s15x16(screen_center_x) ;
            view_vertex[i].y = muls15x16(int_to_s15x16(screen_width), view_vertex[i].y) + int_to_s15x16(screen_center_y) ;
            //printf("%f %f  \n\r\n\r", s15x16_to_float(view_vertex[i].x), s15x16_to_float(view_vertex[i].y)) ;
          }
          
          // draw faces
          for (int i=0; i<4; i++){
            int x1, y1, x2, y2 ;
            x1 = s15x16_to_int(view_vertex[tetra1.face[i][0]].x) ;
            y1 = s15x16_to_int(view_vertex[tetra1.face[i][0]].y) ;
            x2 = s15x16_to_int(view_vertex[tetra1.face[i][1]].x) ;
            y2 = s15x16_to_int(view_vertex[tetra1.face[i][1]].y) ;
            drawLine(x1, y1, x2, y2, WHITE ) ;
          // printf("%d %d %d %d\n\r", x1, y1, x2 ,y2) ;

            x1 = s15x16_to_int(view_vertex[tetra1.face[i][2]].x) ;
            y1 = s15x16_to_int(view_vertex[tetra1.face[i][2]].y) ;
            x2 = s15x16_to_int(view_vertex[tetra1.face[i][1]].x) ;
            y2 = s15x16_to_int(view_vertex[tetra1.face[i][1]].y) ;
            drawLine(x1, y1, x2, y2, WHITE ) ;
            //printf("%d %d %d %d\n\r", x1, y1, x2 ,y2) ;

            x1 = s15x16_to_int(view_vertex[tetra1.face[i][0]].x) ;
            y1 = s15x16_to_int(view_vertex[tetra1.face[i][0]].y) ;
            x2 = s15x16_to_int(view_vertex[tetra1.face[i][2]].x) ;
            y2 = s15x16_to_int(view_vertex[tetra1.face[i][2]].y) ;
            drawLine(x1, y1, x2, y2, WHITE ) ;
            //printf("%d %d %d %d\n\r", x1, y1, x2 ,y2) ;
          }
  
        // A brief nap
        PT_YIELD_usec(30000) ;
        }
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
// === user's serial input thread on core 1
// ==================================================
// serial_read an serial_write do not block any thread
// except this one
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
      static float a,b;
      static s15x16 af, bf ;
      //
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input 2 floats: ");
        // spawn a thread to do the non-blocking write
        serial_write ;

        // spawn a thread to do the non-blocking serial read
         serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%f %f", &a, &b);
        printf("afix x bf =%f     axb=%f \n\r", s15x16_to_float(muls15x16(float_to_s15x16(a),float_to_s15x16(b))), a*b);
        printf("afix / bf =%f     a/b=%f \n\r", s15x16_to_float(divs15x16(float_to_s15x16(a),float_to_s15x16(b))), a/b);
        printf("sqrt(afix)  =%f     sqrt(a) =%f \n\r", s15x16_to_float(sqrts15x16(float_to_s15x16(a))), sqrt(a) ) ;

        //matrix p;
        MxM(&m3, &m4, &p) ;
        printf ("%f\t%f\t%f\t%f\n\r", s15x16_to_float(p.m11), s15x16_to_float(p.m12), s15x16_to_float(p.m13), s15x16_to_float(p.m14)) ;
        printf ("%f\t%f\t%f\t%f\n\r", s15x16_to_float(p.m21), s15x16_to_float(p.m22), s15x16_to_float(p.m23), s15x16_to_float(p.m24)) ;
        printf ("%f\t%f\t%f\t%f\n\r", s15x16_to_float(p.m31), s15x16_to_float(p.m32), s15x16_to_float(p.m33), s15x16_to_float(p.m34)) ;
        printf ("%f\t%f\t%f\t%f\n\r", s15x16_to_float(p.m41), s15x16_to_float(p.m42), s15x16_to_float(p.m43),s15x16_to_float( p.m44)) ;
        
        new_str = true ;

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
  //pt_add_thread(protothread_toggle_gpio4) ;
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
  printf("\n\rProtothreads RP2040 v1.3 two-core, priority\n\r");

  // Initialize the VGA screen
  initVGA() ;

  // sine, cos tables
  for (int i=0; i<360; i++){
    sine[i] = float_to_s15x16(sin(2*3.14159*(float)i/360)) ;
    cosine[i] = float_to_s15x16(cos(2*3.14159*(float)i/360)) ;
  }
     
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