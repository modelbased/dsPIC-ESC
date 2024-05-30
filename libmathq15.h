/*** README ***

From https://github.com/slightlynybbled/libmathq15
Reference: Latest commit 458d60d  on 9 Nov 2017
Licence: MIT

# Q1.15 Library for 8- and 16-bit Embedded Applications #

For a good look at how Q-math operations work, head over to  
[for(embed)](http://www.forembed.com/how-fixed-point-math-works.html) 
and drop a line!


# Library Details #

## Defines ##

The user may customize the header to suit the application for some operations.  
For instance, a look-up table is utilized for trigonometric functions.  
Simply 'define' the appropriate SINE_TABLE_xBIT based on the amount of program 
memory that you can dedicate to the task.

## Macros ##

There are currently no macros implemented, but I suspect that a conversion 
from float-to-q15 will be implemented in the future.

## Type Definitions ##

There are two type definitions that the user must be aware of:

 * q15_t
 * q16angle_t

The 'q15_t' type is a 16-bit, signed number that represents decimal values 
ranging from -1.0 to +0.9997.  Actual values are -32768 to +32767.  The C 
compiler will allow the user to simply add two q15_t numbers together, but it 
is always recommended to use the q15_add() function to keep the number from 
rolling over.

The 'q16angle_t' type is a 16-bit, unsigned number that represents an angle 
between 0 degrees and 359.99 degrees (0 radians to 6.283 radians).  Angles 
can be added or subtracted at will and do not saturate.  For instance, 359 
degrees + 1 result in 0 degrees and 65535 + 1 results in 0.  The rollover 
model is the same.

## Conversion Functions ##

 * double q15_to_dbl(q15_t num) - This function will take a q15 number and 
 convert it to a double between -1.0 and +0.9997.  Note that some embedded 
 target compilers implement 'double' as 'float'.
 * float q15_to_float(q15_t num) - This function will take a q15 number and 
 convert it to a float between -1.0 and +0.9997
 * int16_t q15_to_int(q15_t num) - This function will take a q15 number and 
 convert it to an integer between -1 and 1
 * q15_t q15_from_dbl(double num) - This function will take a double in the 
 range of -1.0 to +0.99997 and convert it to the q15 format
 * q15_t q15_from_float(float num) - This function will take a double in the 
 range of -1.0 to +0.99997 and convert it to the q15 format
 * q15_t q15_from_int(int num) - This function will take an integer in the 
 range of -1 to 1 and convert it to a q15 format.  This function is included 
 for completenes, but it is not expected that it will be utilized in most 
 applications.  Generally, typecasts will be used to convert integers to q15 
 format.

## Basic Arithmetic ##

 * q15_t q15_mul(q15_t multiplicand, q15_t multiplier) - Multiplies two q15 
 numbers and returns the result
 * q15_t q15_div(q15_t dividend, q15_t divisor) - Divides two q15 numbers and 
 returns the result.  Note that that the dividend MUST be smaller than the 
 divisor or the output will saturate!
 * q15_t q15_add(q15_t addend, q15_t adder) - Adds two q15 numbers together 
 and returns the result.  Note that if the result is greater than 32767 or 
 less than -32768, the result will saturate!
 * q15_t q15_abs(q15_t num) - Returns the absoluate value of the given q15 number.
 * q15_t q15_sqrt(q15_t num) - Returns the square root of the given q15 number.  
 Due to resolution, the input 'num' should generally be larger than '1000' or 
 the results will suffer significant skew.
 
## Trigonometry ##
 * q15_t q15_sin(q16angle_t theta) - Returns the sine of the angle in q15 
 format.  Note that this function uses a lookup table (see 'defines') and 
 interpolation.
 * q15_t q15_fast_sin(q16angle_t theta) - Returns the sine of the angle in q15 
 format.  This function uses a lookup table only, but is faster than the 
 standard implementation.
 * q15_t q15_cos(q16angle_t theta) - Returns the cosine of the angle in q15 
 format.  Note that this function uses a lookup table (see 'defines') and 
 interpolation.
 * q15_t q15_fast_cos(q16angle_t theta) - Returns the cosine of the angle in 
 q15 format.  This function uses a lookup table only, but is faster than the 
 standard implementation.
 * q15_t q15_tan(q16angle_t theta) - Returns the tangent of the angle in q15 
 format.   Note that this function uses a lookup table (see 'defines') and 
 interpolation.  Note also that the return type is q15_t, which will saturate 
 at values which are less than -1.0 (-32768) or greater than 0.9997 (+32767).  
 If you require a higher range, then you should probably use a method that 
 offers that range.
 * q15_t q15_fast_tan(q16angle_t theta) - Returns the cosine of the angle in 
 q15 format.  This function uses a lookup table only, but is faster than the 
 standard implementation.
 
Note about trigonometric functions - There is a list of undefined defines at 
the top of 'libmathq15.h' that determines the length of the table used for 
trigonometric functions.  The default is 8-bit, but the user may specify a 
smaller value at compile time in order to reduce memory usage.  Accuracy will 
suffer somewhat as well.
 


***/

#ifndef _Q15_MATH
#define _Q15_MATH

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define Q15_MAX (32767)
#define Q15_MIN (-32768)

/* define the desired trigonometric resolution (higher bit values create larger tables)
 * note that, if all are undefined, the default will be the 8-bit table */
#define SINE_TABLE_7BIT
/*
#undef SINE_TABLE_4BIT
#undef SINE_TABLE_5BIT
#undef SINE_TABLE_6BIT
#undef SINE_TABLE_7BIT
#undef SINE_TABLE_8BIT
*/

typedef int16_t q15_t;
typedef uint16_t q16angle_t;

double q15_to_dbl(q15_t num);
float q15_to_float(q15_t num);
int16_t q15_to_int(q15_t num);
q15_t q15_from_dbl(double num);
q15_t q15_from_float(float num);
q15_t q15_from_int(int num);

q15_t q15_mul(q15_t multiplicand, q15_t multiplier);
q15_t q15_div(q15_t dividend, q15_t divisor);
q15_t q15_add(q15_t addend, q15_t adder);
q15_t q15_abs(q15_t num);
q15_t q15_neg(q15_t num);
q15_t q15_sqrt(q15_t num);

q15_t q15_sin(q16angle_t theta);
q15_t q15_fast_sin(q16angle_t theta);
q15_t q15_cos(q16angle_t theta);
q15_t q15_fast_cos(q16angle_t theta);
q15_t q15_tan(q16angle_t theta);
q15_t q15_fast_tan(q16angle_t theta);

q16angle_t q15_atan2(q15_t sine, q15_t cosine);


#ifdef __cplusplus
}
#endif

#endif

