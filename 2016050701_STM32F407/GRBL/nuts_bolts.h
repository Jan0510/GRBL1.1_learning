#ifndef nuts_bolts_h
#define nuts_bolts_h
#include "stdint.h"

#define false 0
#define true 1

// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value. 
#define Y_AXIS 1
#define Z_AXIS 2
// #define A_AXIS 3

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
#ifdef COREXY
 #define A_MOTOR X_AXIS // Must be X_AXIS
 #define B_MOTOR Y_AXIS // Must be Y_AXIS
#endif

// Conversions
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)
#define F_CPU 128000000
#define F_TIMER 64000000
#define TICKS_PER_MICROSECOND (F_TIMER/1000000)

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

// Bit field and masking macros
#define cli()      __set_PRIMASK(1)  
#define sei()      __set_PRIMASK(0) 
#define bit(n) (1 << n) 

#define bit_true_atomic(x,mask) {(x) |= (mask); }
#define bit_false_atomic(x,mask) {(x) &= ~(mask); }
#define bit_toggle_atomic(x,mask) {(x) ^= (mask); }
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while float_ptr is 
// a pointer to the result variable. Returns true when it succeeds
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
void delay_ms(uint16_t ms);

// Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
void delay_us(uint32_t us);

// Computes hypotenuse, avoiding avr-gcc's bloated version and the extra error checking.
float hypot_f(float x, float y);

#endif
