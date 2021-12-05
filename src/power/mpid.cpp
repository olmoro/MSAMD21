/*
  Прототип: A fixed point PID controller with a 32-bit internal calculation pipeline.
  From:  https://github.com/mike-matera/FastPID/tree/master/examples/VoltageRegulator
  
  Отмерены проверки вводимых параметров, что возложено на ведущий контроллер.
  Версия от 23 октября 2020г.
  20210512 - исправлено clear()
*/

#include "mpid.h"
#include <Arduino.h>

MPid::~MPid()
{}

void MPid::clear() 
{
  lastSp  = 0; 
    lastOut = 0; 
  sum     = 0; 
  lastErr = 0;
}

int16_t MPid::getLastSp()               { return lastSp; }
int32_t MPid::getLastErr()              { return lastErr; }
void    MPid::setLastSp(int16_t _sp)    { lastSp = _sp; }
void    MPid::setLastErr(int32_t _err)  { lastErr = _err; }

void MPid::setCoefficients(uint16_t kp, uint16_t ki, uint16_t kd)
{
  p = uint32_t( kp );
  i = uint32_t( ki );
  d = uint32_t( kd );
}

void MPid::setOutputRange(int16_t min, int16_t max)
{
  outMin = int64_t( min ) * param_mult;
  outMax = int64_t( max ) * param_mult;
}

void MPid::configure(uint16_t kp, uint16_t ki, uint16_t kd, int16_t min, int16_t max)
{
  clear();
  setCoefficients( kp, ki, kd );
  setOutputRange( min, max);
}

void MPid::replaceConfig(uint16_t kp, uint16_t ki, uint16_t kd, int16_t min, int16_t max) 
{
  setCoefficients( kp, ki, kd );
  setOutputRange( min, max );
}


int16_t MPid::step(int16_t sp, int16_t fb) 
{
  // int16 + int16 = int17
  int32_t err = int32_t( sp ) - int32_t( fb );
  int32_t P = 0;
  int32_t I = 0;
  int32_t D = 0;

  if ( p ) 
  {
    // uint16 * int16 = int32
    P = int32_t( p ) * int32_t( err );
  }

  if ( i ) 
  {
    // int17 * int16 = int33
    sum += int64_t( err ) * int64_t( i );

    // Limit sum to 32-bit signed value so that it saturates, never overflows.
    if      ( sum > integ_max )  sum = integ_max;
    else if ( sum < integ_min )  sum = integ_min;

    // int32
    I = sum;
  }

  if ( d ) 
  {
    // (int17 - int16) - (int16 - int16) = int19
    int32_t deriv = ( err - lastErr ) - int32_t( sp - lastSp );
    lastSp = sp; 

    //SerialUSB.print("lastSp =0x"); SerialUSB.println(lastSp, HEX);

    lastErr = err; 

    // Limit the derivative to 16-bit signed value.
    if      ( deriv > deriv_max )  deriv = deriv_max;
    else if ( deriv < deriv_min )  deriv = deriv_min;

    // int16 * int16 = int32
    D = int32_t( d ) * int32_t( deriv );
  }

  // int32 (P) + int32 (I) + int32 (D) = int34
  int64_t out = int64_t( P ) + int64_t( I ) + int64_t( D );

  // Make the output saturate
  if      ( out > outMax )  out = outMax;
  else if ( out < outMin )  out = outMin;

// SerialUSB.print("shift="); SerialUSB.println(param_shift);
// SerialUSB.print("param_max =0x"); SerialUSB.println(param_max, HEX);
// SerialUSB.print("param_mult=0x"); SerialUSB.println(param_mult, HEX);
// SerialUSB.print("lastSp =0x"); SerialUSB.println(lastSp, HEX);
// SerialUSB.print("lastErr=0x"); SerialUSB.println(lastErr, HEX);
// SerialUSB.print("min,max "); SerialUSB.print  ((uint16_t)(outMin >> 8)); 
// SerialUSB.print(" ");        SerialUSB.println((uint16_t)(outMax >> 8));

  // Remove the integer scaling factor. 
  int16_t rval = out >> param_shift;

  // Fair rounding.
  if ( out & ( 0x1ULL << ( param_shift - 1 )))  rval++;

  return rval;
}
