/*
  Output PWM 24Khz on digital pin PA16 (D11 SAMD21 MINI)
  SAMD21 pin 11
  using timer TCC2 (10-bit resolution), channel CH0
  Версия от 23 октября 2020г.
*/

#include "power/power_pwm.h"
#include <Arduino.h>

  // Переменные настройки PWM (пока нет eeprom)
constexpr uint16_t pwm_period = 0x1000;       // Freq = 5.865 kHz (без вариантов 12бит)
uint16_t            pwmPeriod = pwm_period;   // 1/Freq
constexpr bool     pwm_invert = false;        //
bool pwmInvert                = pwm_invert;   // 0 - активный уровень

void initPwm()
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |      // Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);        // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);          // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital 
  // pin D11 g_APinDescription() converts Arduino Pin to SAMD21 pin
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC2 timer to digital output D11 - port pin
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXE_E ;   // use device E on TCC2/WO[0]

  // Feed GCLK4 to TCC2
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  if( pwmInvert )
  {
    REG_TCC2_WAVE |= TCC_WAVE_WAVEGEN_DSBOTH;
    while (TCC2->SYNCBUSY.bit.WAVE);              // Wait for synchronization
  }
  else
  {
    REG_TCC2_WAVE |= TCC_WAVE_POL(0X0F) | TCC_WAVE_WAVEGEN_DSBOTH;
    while (TCC2->SYNCBUSY.bit.WAVE);              // Wait for synchronization
  }

  // Timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // Freq = 48Mhz/(2*N*PER)
  REG_TCC2_PER = pwmPeriod;                       // Set the FreqTcc of the PWM on TCC1 to specified Khz
  while (TCC2->SYNCBUSY.bit.PER);                 // Wait for synchronization
 
  // Set the PWM signal to output,
  // PWM ds = 2*N(TOP-CCx)/Freqtcc => PWM=0 => CCx=PER,
  // PWM=50% => CCx = PER/2
  REG_TCC2_CC0 = 0;                               // TCC2 CC0 - on D11
  while (TCC2->SYNCBUSY.bit.CC0);                 // Wait for synchronization
 
  // Divide the GCLOCK signal by 1 giving 
  // in this case 48MHz (20.83ns) TCC2 timer tick and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void writePwm(uint16_t value)
{
  REG_TCC2_CC0 = value;                           // TCC2 CC0 - on D11 - PWM signalling
  while (TCC2->SYNCBUSY.bit.CC3);                 // Wait for synchronization
}
