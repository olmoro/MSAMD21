/*
  
 * Версия от 23 октября 2020г.
*/

#include "board/mboard.h"
#include "board/mpins.h"
#include "adc/adc.h"    // DAC
#include <Arduino.h>

  // state1 - состояния, true - включено:
extern bool switchStatus;         // коммутатор нагрузки ( sw_pin   )
extern bool powerStatus;          // преобразователь     (  )
// extern bool currentControlStatus;  // регулирование по току
extern bool voltageControlStatus;  // регулирование по напряжению
extern bool chargeStatus;         // заряд               (  )
extern bool dischargeStatus;      // разряд (тот же вывод, !chargeStatus )
// extern bool pauseStatus;           // пауза
extern bool pidStatus;             // управление регулятором

  // Включение силовых ключей
void swPinOn()
{
  #ifdef REMONT
    digitalWrite( MPins::sw15_pin, LOW );
  #endif
  digitalWrite( MPins::sw_pin, LOW );
}

  // Выключение силовых ключей
void swPinOff()
{
  #ifdef REMONT
    digitalWrite( MPins::sw15_pin, HIGH );
  #endif
  digitalWrite( MPins::sw_pin, HIGH );
}


void p15PinOn()  { digitalWrite( MPins::pa15_pin, LOW  ); }  // Включение тестового вывода
void p15PinOff() { digitalWrite( MPins::pa15_pin, HIGH ); }  // Отключение  тестового вывода

void p14PinOn()  { digitalWrite( MPins::pa14_pin, LOW  ); }  // Включение тестового вывода
void p14PinOff() { digitalWrite( MPins::pa14_pin, HIGH ); }  // Отключение  тестового вывода

void tstPinOn()  { digitalWrite( MPins::tst_pin, LOW  ); }  // Включение тестового вывода
void tstPinOff() { digitalWrite( MPins::tst_pin, HIGH ); }  // Отключение  тестового вывода


  // Инициализация дискретных портов
void portsInit()
{
  pinMode( MPins::pa15_pin, OUTPUT);
  pinMode( MPins::pa14_pin, OUTPUT);  
  pinMode( MPins::tst_pin, OUTPUT);
  pinMode( MPins::sw_pin,  OUTPUT);
  // #ifdef REMONT
  //   pinMode( MPins::sw15_pin, OUTPUT );
  // #endif
  
  #ifdef WEMOS    // using pcb SAMD21 MINI
    pinMode( MPins::led_rx, OUTPUT);  // led_rx   = 25   no   PB03/LED1 (LED_BUILTIN, LED_RX)
    pinMode( MPins::led_tx, OUTPUT);  // led_tx   = 26   no   PA27/LED2 (LED_TX)
  #endif

  swPinOff();               // Силовые ключи нагрузки отключены
  p15PinOff();
  p14PinOff();
  tstPinOn();               //
  dacInit();                // Set reference
  dacWrite10bit( 0x0000 );  //
}

  // Конфигурация режимов
void configMode(uint8_t mode)
{
  switch ( mode )
  {
  case 1:                             // U - регулирование по напряжению
    //chargeStatus = true;              // коммутатор на заряд
    //dischargeStatus = !chargeStatus;
    powerStatus = true;               // преобразователь включить
//    powPinOn();
    switchStatus = true;              // к клеммам подключить
    swPinOn();
  break;

  case 2:                             // I - регулирование по току заряда
    //chargeStatus = true;              // коммутатор на заряд
    //dischargeStatus = !chargeStatus;
    powerStatus = true;               // преобразователь включить
//    powPinOn();
    switchStatus = true;              // к клеммам подключить
    swPinOn();
  break;

  case 3:                             // D - регулирование по току разряда
    //dischargeStatus = true;           // коммутатор на заряд
    //chargeStatus = !dischargeStatus;
    powerStatus = false;              // преобразователь выключить
//    powPinOff();
    switchStatus = true;              // к клеммам подключить
    swPinOn();
  break;

  default:    // OFF
    //chargeStatus = true;              // коммутатор на заряд
    //dischargeStatus = !chargeStatus;
    powerStatus = false;              // преобразователь выключить
//    powPinOff();
    switchStatus = false;             // от клемм отключить
    swPinOff();
  break;
  }
}
