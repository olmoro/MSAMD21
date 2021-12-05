/*

  Версия от 25 ноября 2020г.
*/

#include "board/mpins.h"
#include "wake/wake.h"
#include "adc/adc.h"
#include "adc_commands.h"
#include "SAMD_AnalogCorrection.h"
#include "power/power_reg.h"
#include "stdint.h"
#include <Arduino.h>

  // Переменные протокола обмена  
extern uint8_t        rxNbt;  // принятое количество байт в пакете
extern uint8_t rxDat[frame];  // массив принятых данных
extern uint8_t      command;  // код команды на выполнение
extern uint8_t        txNbt;  // количество байт данных в пакете
extern uint8_t txDat[frame];  // массив данных для передачи

extern int16_t   adcVoltage;  // Данные АЦП
extern int16_t   adcCurrent;  //

  // Пересчитанные в физические величины - mV, mA, mC
extern int16_t mvVoltage;
extern int16_t maCurrent;

extern uint8_t state1;
extern uint8_t state2;

  // comm 0x24, 0x25, 0x26 параметры измерителей, доступные пользователю
//extern int16_t  prbOffset[];      // приборное смещение
//extern uint16_t prbFactor[];      // коэффициент преобразования в физическую величину

  // коэффициенты преобразования в физические величины
extern uint16_t factorU;
extern uint16_t factorI;

  // приборные смещения
extern int16_t offsetU;
extern int16_t offsetI;

  // параметры сглаживания для измерителей
extern uint8_t smoothU;           // параметр сглаживания для измерителя напряжения
extern uint8_t smoothI;           // параметр сглаживания для измерителя тока

  // конструктивное смешение АЦП
extern int16_t adcOffset;         // смещение АЦП (ед)

  // Пороги
extern int16_t winLtU;
extern int16_t winLtI;
extern int16_t winUpU;
extern int16_t winUpI;


  // comm 0x51, 0x52, 0x53 настройки АЦП
// extern uint8_t adcRef     [];     // опорное напряжение
// extern uint8_t adcGain    [];     // усиление
// extern uint8_t adcBits    [];     // 0x00(12), 0x01(16), 0x02(10), 0x03(8)
// extern uint8_t adcSamples [];     // 0x00 ... 0x0a (1, 2, 4, 8 ... 1024)
// extern uint8_t adcDivider [];     // 0x00 ... 0x07 (2^0, 2^1, 2^2 ... 2^7)
// extern uint8_t adcRefComp [];     // 0x00, 0x01  Это резервная позиция
// //extern int16_t adcOffset  [];     // if disable the reference buffer offset compensation
// extern uint8_t adcGainComp[];     // Gain compensation << 0, << 2

// extern uint8_t refComp;


// 0x10 Чтение текущих напряжения, тока и состояний 
void doReadUI()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, (uint16_t)mvVoltage );
    id = replyU16( id, (uint16_t)maCurrent );
    id = replyU08( id, state1 );              // информация о состоянии 
    id = replyU08( id, state2 );              // информация о состоянии 
    txReplay( id, 0 );                        // всего байт, в нулевом - сообщение об ошибках (nu)
  }
  else  txReplay(1, err_tx);                  // ошибка протокола
}


  // 0x30 Чтение калибровки измерителя напряжения
void doGetFactorU()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, factorU );   // коэффициент преобразования в милливольты
    txReplay( id, 0x00 );           // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
} 

  // 0x31 Запись калибровки измерителя напряжения
  // Запрос: 0xC0 0x31 0x03 0x00 0x01 0x23 0x8C
  // Ответ:  0xC0 0x31 0x01 0x00 0x9C
void doSetFactorU()
{
  if( rxNbt == 2 )
  {
    factorU = get16(0);             // коэффициент преобразования в милливольты
    txReplay( 1, 0x00 );            // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x32 Возврат к заводскому
void doSetFactorDefaultU()
{
  if( rxNbt == 0 )
  {
    factorU = SetFactorDefaultU();  // коэффициент преобразования в милливольты
    txReplay( 1, 0x00 );            // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x33 Читать параметр сглаживания по напряжению
void doGetSmoothU()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU08( id, smoothU ); 
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x34 Записать параметр сглаживания по напряжению
  // Запрос: 0xC0 0x34 //0x03 0x00 0x45 0x67 0xD9
  // Ответ:  0xC0 0x34 0x01 0x00 0x9C               ???? байт!
void doSetSmoothU()
{
  if( rxNbt == 1 )
  {
    uint8_t val = rxDat[0];      // 0...8: 0, 2, 4, 8 ... 
    ( val >= 8 ) ? smoothU = 8 : smoothU = val;
    txReplay( 1, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x35 Чтение приборного смещения, мВ 
void doGetOffsetU()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, offsetU );   // смещение, мВ
    txReplay( id, 0x00 );           // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x36 Запись приборного смещения, мВ
  // Запрос: 0xC0 0x36 0x03 0x00 0x00 0x89 0xC8
  // Ответ:  0xC0 0x36 0x01 0x00 0xE6
void doSetOffsetU()
{
  if( rxNbt == 2 )
  {
    offsetU = get16(0);             // смещение, мВ
    txReplay( 1, 0x00 );            // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x38 Чтение калибровки измерителя тока
void doGetFactorI()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, factorI );   // коэффициент преобразования в миллиамперы
    txReplay( id, 0x00 );           // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x39 Запись калибровки измерителя тока
void doSetFactorI()
{
  if( rxNbt == 2 )
  {
    factorI = get16(0);             // коэффициент преобразования в миллиамперы
    txReplay( 1, 0x00 );            // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x3A Возврат к заводскому множителю
void doSetFactorDefaultI()
{
  if( rxNbt == 0 )
  {
    factorI = SetFactorDefaultI();  // коэффициент преобразования в миллиамперы
    txReplay( 1, 0x00 );            // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x3B Чтение параметра сглаживания по току
void doGetSmoothI()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU08( id, smoothI ); 
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x3C Запись параметра сглаживания по току
void doSetSmoothI()
{
  if( rxNbt == 1 )
  {
    uint8_t val = rxDat[0];      // 0...8: 0, 2, 4, 8 ... 
    ( val >= 8 ) ? smoothI = 8 : smoothI = val;
    txReplay( 1, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x3D Чтение приборного смещения, мA 
void doGetOffsetI()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, offsetI );   // смещение, мА
    txReplay( id, 0x00 );           // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x3E Запись приборного смещения, мА
void doSetOffsetI()
{
  if( rxNbt == 2 )
  {
    offsetU = get16(0);             // смещение, мА
    txReplay( 1, 0x00 );            // подтверждение
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

// 0x50 Чтение измерений  в HEX и состояний
void doReadProbes()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, adcVoltage );
    id = replyU16( id, adcCurrent );
    id = replyU08( id, state1 );      // информация о состоянии 
    id = replyU08( id, state2 );      // информация о состоянии 
    txReplay( id, 0x00 );             // всего байт, в нулевом - сообщение об ошибках(нет)
  }
  else  txReplay(1, err_tx);          // ошибка протокола
}

  // 0x51 Чтение смещения АЦП
void doAdcGetOffset()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, uint16_t( adcOffset ) ); 
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x52 Запись смещения АЦП
void doAdcSetOffset()
{
  if( rxNbt == 2 )
  {
    adcOffset = int16_t( get16(0) );
    txReplay( 1, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x60 Чтение предела напряжения снизу
void doGetWinLtU()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, uint16_t( winLtU ) ); 
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x61 Запись предела напряжения снизу 
void doSetWinLtU()
{
  if( rxNbt == 2 )
  {
    winLtU = int16_t( get16(0) );
    txReplay( 1, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x62 Возврат к заводскому уровню
void doSetWinLtDefaultU()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    winLtU = win_less_default_u;
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x63 Чтение предела напряжения сверху 
void doGetWinUpU() 
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, uint16_t( winUpU ) ); 
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x64 Запись предела напряжения сверху  
void doSetWinUpU() 
{
  if( rxNbt == 2 )
  {
    winUpU = int16_t( get16(0) );
    txReplay( 1, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x65 Возврат к заводскому уровню 
void doSetWinUpDefaultU()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    winUpU = win_up_default_u;
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x68 Чтение предела тока снизу 
void doGetWinLtI() 
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, uint16_t( winLtI ) ); 
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x69 Запись предела тока снизу  
void doSetWinLtI()
{
  if( rxNbt == 2 )
  {
    winLtI = int16_t( get16(0) );
    txReplay( 1, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x6A Возврат к заводскому уровню 
void doSetWinLtDefaultI()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    winUpU = win_less_default_i;
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x6B Чтение предела тока сверху 
void doGetWinUpI()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU16( id, uint16_t( winUpI ) ); 
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x6C Запись предела тока сверху 
void doSetWinUpI()
{
  if( rxNbt == 2 )
  {
    winUpI = int16_t( get16(0) );
    txReplay( 1, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}

  // 0x6D Возврат к заводском
void doSetWinUpDefaultI()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    winUpU = win_up_default_i;
    txReplay( id, 0x00 );            // Об ошибках параметров не сообщается
  }
  else  txReplay(1, err_tx);        // ошибка протокола
}
