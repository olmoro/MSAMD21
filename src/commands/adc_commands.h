#ifndef _ADC_COMMANDS_H_
#define _ADC_COMMANDS_H_
/*

  Версия от 25 ноября 2020г.
*/

#include "stdint.h"

  // Процедуры команд обмена по UART
void doReadUI();                  // 0x10   Чтение напряжения и тока

  // Множитель преобразования в милливольты
void doGetFactorU();              // 0x30   Чтение калибровки измерителя напряжения
void doSetFactorU();              // 0x31   Запись калибровки измерителя напряжения
void doSetFactorDefaultU();       // 0x32   Возврат
void doGetSmoothU();              // 0x33   Чтение параметра сглаживания по напряжению
void doSetSmoothU();              // 0x34   Запись параметра сглаживания по напряжению
void doGetOffsetU();              // 0x35   Чтение приборного смещения, мВ 
void doSetOffsetU();              // 0x36   Запись приборного смещения

  // Множитель преобразования в миллиамперы
void doGetFactorI();              // 0x38   Чтение калибровки измерителя тока
void doSetFactorI();              // 0x39   Запись калибровки измерителя тока
void doSetFactorDefaultI();       // 0x3A   Восстановление заводских калибровок
void doGetSmoothI();              // 0x3B   Чтение параметра сглаживания по току
void doSetSmoothI();              // 0x3C   Запись параметра сглаживания по току
void doGetOffsetI();              // 0x3D   Чтение приборного смещения, мA 
void doSetOffsetI();              // 0x3E   Запись приборного смещения


void doReadProbes();              // 0x50   Чтение измерений  в HEX и состояний
void doAdcGetOffset();            // 0x51   Чтение смещения АЦП
void doAdcSetOffset();            // 0x52   Запись смещения АЦП

void doGetWinLtU();               // 0x60   Чтение предела напряжения снизу
void doSetWinLtU();               // 0x61   Запись предела напряжения снизу 
void doSetWinLtDefaultU();        // 0x62   Возврат к заводскому уровню
void doGetWinUpU();               // 0x63   Чтение предела напряжения сверху  
void doSetWinUpU();               // 0x64   Запись предела напряжения сверху   
void doSetWinUpDefaultU();        // 0x65   Возврат к заводскому уровню 

void doGetWinLtI();               // 0x68   Чтение предела тока снизу  
void doSetWinLtI();               // 0x69   Запись предела тока снизу  
void doSetWinLtDefaultI();        // 0x6A   Возврат к заводскому уровню 
void doGetWinUpI();               // 0x6B   Чтение предела тока сверху 
void doSetWinUpI();               // 0x6C   Запись предела тока сверху 
void doSetWinUpDefaultI();        // 0x6D   Возврат к заводскому уровню 

#endif  //!_ADC_COMMANDS_H_
