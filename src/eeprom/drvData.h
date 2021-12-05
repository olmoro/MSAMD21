/*

*/

#ifndef _DRVDATA_H_
#define _DRVDATA_H_

#include "stdint.h"

//struct
//{
  //----------------- настройки PWM управления источником
  const uint16_t pwm_period = 1000;   // Freq = 24 kHz
  uint16_t pwmPeriod = pwm_period;     // Freq = 48Mhz/(2*N*PER), N = 1
  
  const bool pwm_invert = true;
  bool pwmInvert = pwm_invert;                     // 0 - активный уровень

//} drvData;

#endif  //!_DRVDATA_H_