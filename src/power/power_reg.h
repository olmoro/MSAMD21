/*

*/

#ifndef _POWER_REG_H_
#define _POWER_REG_H_

#include "stdint.h"

void portsInit();
void initPids();                       // 

//void doPid();
void doPid( int16_t fbU, int16_t fbI );

//void doPidU();
//void doPidI();

void saveState( int mode );
void restoreState( int mode );

//void surgeCompensation( int16_t u );  // Компенсация всплеска напряжения
void idleLoad();
void powerFailure(uint8_t err);       // В том числе и аварийное отключение


  // Команды управления процессами
void doPowerGo();                     // 0x20 Старт преобразователя с заданными максимальными U и I
void doPowerStop();                   // 0x21 отключение преобразователя
void doSetPid();                      // 0x22 пока не реализована


void doPidConfigure();                // 0x40 Конфигурирование пид-регулятора с очисткой регистров
void doPidSetCoefficients();          // 0x41 ввод коэффициентов kp, ki, kd для заданного режима
void doPidOutputRange();              // 0x42 ввод диапазона вывода для заданного режима
void doPidReconfigure();              // 0x43 Конфигурирование пид-регулятора без очистки регистров
void doPidClear();                    // 0x44 Очистка регистров регулятора

void doPidTest();                     // 0x46 Тестовая. Тест пид-регулятора
void doPwmConfigure();                // 0x47 Конфигурирование pwm-регулятора
void doPidGetConfigure();             // 0x48 Возвращает параметры текущего режима регулирования
void doPidSetMaxSum();                // 0x49 Задает максимальный интеграл при вычислении шага регулирования



  // Команды для тестирования
void doSwPin();                       // 0x54

void setPower();                      // 0x56
void setDischg();                     // 0x57 
void doSetVoltage();                  // 0x58 задать напряжение и включить
void doSetCurrent();                  // 0x59 задать ток заряда и включить
void doSetDiscurrent();               // 0x5A задать ток разряда и включить
void doSurgeCompensation();           // 0x5B задать параметры компенсации перенапряжения
void doIdleLoad();                    // 0x5C задать параметры доп. нагрузки на ХХ

#endif  //!_POWER_REG_H_