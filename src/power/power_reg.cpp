/*
  Вдохновение от:

  https://github.com/mike-matera/FastPID/tree/master/examples/VoltageRegulator
  FastPID: A fast 32-bit fixed-point PID controller for Arduino

  v55 - Релейный режим. Преобразователь включается на вычисленное время посредством PWM.
  При частоте 6кГц...

  Подбор коэффициентов ПИД-регулятора
  
  версия от 23 октября 2020г.
*/

#include <Arduino.h>
#include "board/mboard.h"
#include "adc/adc.h"      // DAC
#include "wake/wake.h"
//#include "eeprom/drvData.h"
#include "power/power_pwm.h"
#include "power/power_reg.h"
#include "commands/commands.h"
#include "power/mpid.h"
#include "stdint.h"

    // Переменные настройки PWM
extern uint16_t pwmPeriod;          // Freq
extern bool pwmInvert;              // активный уровень

    // Переменные протокола Wake 
extern uint8_t  rxNbt;              // принятое количество байт в пакете
extern uint8_t  rxDat[frame];       // массив принятых данных
extern uint8_t  command;            // код команды на выполнение
extern uint8_t  txNbt;              // количество байт данных в пакете
extern uint8_t  txDat[frame];       // массив данных для передачи

extern uint8_t state1;  // state1 - состояния и управление, true - включено(включить):
extern bool switchStatus;           // коммутатор нагрузки ( sw_pin PA15 )
extern bool powerStatus;            // управление преобразователем
extern bool currentControlStatus;   // регулирование по току
extern bool voltageControlStatus;   // регулирование по напряжению
extern bool chargeStatus;           // заряд
extern bool dischargeStatus;        // разряд (тот же вывод, !chargeStatus )
extern bool pauseStatus;            // пауза
extern bool pidStatus;              // управление регулятором

extern uint8_t state2;  // state2
// extern bool overHeatingStatus;     // перегрев
// extern bool overloadStatus;        // перегрузка
// extern bool powerLimitationStatus; // ограничение мощности
// extern bool reversePolarityStatus; // обратная полярность
// extern bool shortCircuitStatus;    // короткое замыкание
// extern bool calibrationStatus;     // калибровка
// extern bool upgradeStatus;         // обновление
// extern bool reserve2Status;        // резерв 2

uint8_t errorCode = 0x00;

  // Данные АЦП nu?
extern uint16_t adcVoltage;
extern uint16_t adcCurrent;
  // Пересчитанные в физические величины - mV, mA
extern int16_t mvVoltage;
extern int16_t maCurrent;

  // Опорное напряжение ADC в милливольтах
constexpr int16_t avcc = 3300;

  // Приборные диапазоны задания напряжения и токов ??? согласовать 
  // с параметрами отключения (adc.cpp L80)?
  // Предварительно, для тестирования
constexpr int16_t volt_min       =  2000;  //  2.0 в
constexpr int16_t volt_max       = 16200;  // 16.2 в
constexpr int16_t curr_ch_min    =    50;  //  0.05 А
constexpr int16_t curr_ch_max    = 12000;  // 12.0 А
constexpr int16_t curr_disch_min =    50;  //  0.05 А
constexpr int16_t curr_disch_max =  3000;  //  3.0 А
constexpr float   hz             = 10.0f;  //  всегда 10

// Дефолтные параметры регулирования для всех режимов 
// Это тестовые значения - задавать через целочисленные значения,
// используя согласованный множитель
constexpr uint16_t kp_def   =   0.1f         * MPid::param_mult;   // 0.1  0x0019
constexpr uint16_t ki_def   =  (0.5f  / hz ) * MPid::param_mult;   // 0.5  0x000C
constexpr uint16_t kd_def   =  (0.01f * hz ) * MPid::param_mult;   // 0.01 0x0019
// bits и sign заданы жестко в отличие от прототипа.

// Ограничения на output приборные, вводятся setOutputRange(min,max),
// будут в инициализации? 
constexpr int16_t min_pwm   = 0x0220;   // Не 0x0000 - с учетом частотных характеристик оптопары.
constexpr int16_t max_pwm   = 0x1000;   // Freq = 5.865 kHz (без вариантов 12бит - см. power_pwm.cpp)
constexpr int16_t min_dac   = 0x0020;
constexpr int16_t max_dac   = 0x03FF;   //

// Варианты настройки для разных режимов (modes) регулирования 
// напряжения, тока заряда, тока разряда (дефолтные значения)
// Для разряда (режим D) используется другой экземпляр регулятора
// Разрядность (10бит) и опорное (AVCC) DAC заданы жестко, как и частота регулирования (hz=10Hz) 

enum mode { OFF = 0, U, I, D };

// Массивы параметров настройки ПИД-регуляторов
//             mode =      OFF       U        I        D
uint16_t kP[]       = {       0,  kp_def,  kp_def,  kp_def };  
uint16_t kI[]       = {       0,  ki_def,  ki_def,  ki_def };
uint16_t kD[]       = {       0,  kd_def,  kd_def,  kd_def };
int16_t minOut[]    = { min_pwm, min_pwm, min_pwm, min_dac };
int16_t maxOut[]    = { max_pwm, max_pwm, max_pwm, max_dac };

// Заданный уровень регулирования - мВ или мА
int16_t setpoint[]  = {  0x0000,  0x0000,  0x0000,  0x0000 };

int16_t output  = 0x0000;

// Параметры компенсация всплеска напряжения минимальной подгрузкой преобразователя
int16_t surgeVoltage = 250;       // Милливольты превышения
int16_t surgeCurrent = 0x200;     // Ток в коде DAC

int16_t idleCurrent = 250;        // Миллиамперы, меньше которых необходима дополнительная нагрузка 
int16_t idleDac     = 0x240;      // Ток в коде DAC

// Выбор режима регулирования (не путать с датчиками)
uint8_t pidMode = OFF;   // OFF-U-I-D: выкл, задать напряжение, ток заряда или ток разряда

// Сохраненные регистры регуляторов
int16_t sLastSpU;
int64_t sSumU;
int32_t sLastErrU;

int16_t sLastSpI;
int64_t sSumI;
int32_t sLastErrI;

// Вариант с общим регулятором по напряжению и току
MPid MyPid ( kP[U], kI[U], kD[U], minOut[U], maxOut[U] );  // Common Voltage and Current control
MPid MyPidD( kP[D], kI[D], kD[D], minOut[D], maxOut[D] );  // Discurrent control


void  initPids()
{
  MyPid.configure ( kP[U], kI[U], kD[U], minOut[U], maxOut[U] );
  MyPidD.configure( kP[D], kI[D], kD[D], minOut[D], maxOut[D] );
  initPwm();
} 

// Запуск и выбор регулятора производится выбором pidMode: OFF, U, I, D и
// powerStatus = true      - Преобразователь включить (включен)
// pidStatus   = true      - Регулятор включить (включен)

//void doPid()
void doPid( int16_t fbU, int16_t fbI )
{
  //unsigned long before = micros();  // Раскомментировать 3 строки для вывода времени исполнения

  //int16_t fbU = mvVoltage;  // feedback U
  //int16_t fbI = maCurrent;
  int16_t outU;
  int16_t outI;
  int16_t outD;

  if( pidStatus )
  {

    swPinOn();
    switchStatus          = true;             // При работающем ПИД-регуляторе коммутатор включен ПОСТОЯННО

    switch ( pidMode )
    {
    case OFF:
      // Выход из регулирования с отключением всего
      #ifdef DEBUG_POWER
        SerialUSB.println(".OFF");
      #endif

      swPinOff();
      switchStatus          = false;            // отключить от нагрузки

      writePwm( 0x0000 );
      powerStatus           = false;            // преобразователь выключен

      currentControlStatus  = false;            // регулирование по току отключено
      voltageControlStatus  = false;            // регулирование по напряжению отключено
      chargeStatus          = false;            // заряд отключен

      dacWrite10bit( surgeCurrent );            // разрядить выходной фильтр 
      dischargeStatus       = false;            // разряд отключен

      pauseStatus           = false;            // пауза отключена

      // Выход из режима регулирования
      idleLoad();
      pidStatus             = false;            // регулятор выключен
      break;

    case U:
      if( fbI < setpoint[I] )                   // если ток менее заданного, но не разряд)) 
      {
        // Режим регулирования по напряжению
        swPinOn();
        switchStatus          = true;           // коммутатор включен (дублирование?)
        voltageControlStatus  = true;           // регулирование по напряжению включено

        outU = MyPid.step( setpoint[U], fbU );  // коррекция 
        writePwm( outU );
        powerStatus           = true;           // преобразователь включен

        currentControlStatus  = false;          // регулирование по току отключено
        chargeStatus          = true;           // заряд включен       дублируется powerStatus ???
        dischargeStatus       = false;          // разряд отключен
        pauseStatus           = false;          // пауза отключена
        pidStatus             = true;           // регулятор включен   дублируется powerStatus ???

        #ifdef DEBUG_POWER
          SerialUSB.print(" ChargeU: ");     
          SerialUSB.print(" spU: ");    SerialUSB.print( setpoint[U] );     
          SerialUSB.print(" fbU: ");    SerialUSB.print( fbU );
          SerialUSB.print(" outU: 0x"); SerialUSB.println( outU, HEX ); 
        #endif
      
        //surgeCompensation( -(MyPid.getLastErr()) );    // Компенсация всплеска напряжения
        idleLoad();
      }
      else                                      // ток выше предела - перейти к регулированию по току
      {
        if( pidMode )                           // если не отключено 
        {
          #ifdef OSC 
            tstPinOff();                        // Метка для осциллографа
          #endif
          //saveState(U);                         // Сохранить регистры регулятора
          //restoreState(I);                      // Перейти к регулированию по току
          MyPid.setCoefficients( kP[I], kI[I], kD[I] );
                //MyPid.replaceConfig( kP[I], kI[I], kD[I], minOut[I], maxOut[I]);
                //MyPid.configure( kP[I], kI[I], kD[I], minOut[I], maxOut[I]);
                //outI = MyPid.step( setpoint[I], fbI );
    //MyPid.clear();
          pidMode = I;
          #ifdef OSC 
            tstPinOn();                         // Метка для осциллографа
          #endif
        }
      }
      break;

    case I:
      if( fbI >= setpoint[I] )                  // если то более или равен заданному, иначе перейти...
      {
        // Режим регулирования по току
        swPinOn();
        switchStatus          = true;           // коммутатор включен (дублирование?)
        currentControlStatus  = true;           // регулирование по току включено

        outI = MyPid.step( setpoint[I], fbI );
        writePwm( outI );
        powerStatus           = true;           // преобразователь включен
        
        voltageControlStatus  = false;          // регулирование по напряжению выключено
        chargeStatus          = true;           // заряд включен
        dischargeStatus       = false;          // разряд отключен
        pauseStatus           = false;          // пауза отключена
        pidStatus             = true;           // регулятор включен

        #ifdef DEBUG_POWER
          SerialUSB.print(" ChargeI: ");     
          SerialUSB.print(" spI: ");    SerialUSB.print( setpoint[I] );     
          SerialUSB.print(" fbI: ");    SerialUSB.print( fbI );
          SerialUSB.print(" outI: 0x"); SerialUSB.println( outI, HEX ); 
        #endif 

        idleLoad(); 
      }
      else                                      // ... перейти к регулированию по напряжению
      {
        if( pidMode )                           // если не отключено 
        {
          #ifdef OSC 
            tstPinOff();                        // Метка для осциллографа
          #endif
          //saveState(I);
          //restoreState(U);
          MyPid.setCoefficients( kP[U], kI[U], kD[U] );
                //MyPid.replaceConfig( kP[U], kI[U], kD[U], minOut[U], maxOut[U]);
                //MyPid.configure( kP[U], kI[U], kD[U], minOut[U], maxOut[U]);
                //outU = MyPid.step( setpoint[U], fbU );
      //MyPid.clear();
          pidMode = U;
          #ifdef OSC 
            tstPinOn();                         // Метка для осциллографа
          #endif
        }
      }
      break;

    case D:
      // Регулирование тока разряда                             !!! ( НЕ ПРОВЕРЕНО ) !!!
      swPinOn();
      switchStatus          = true;   // батарея подключена (не факт))

      writePwm( 0x0000 );
      powerStatus           = false;  // преобразователь выключен

      currentControlStatus  = false;  // регулирование по току выключено
      voltageControlStatus  = false;  // регулирование по напряжению выключено
      chargeStatus          = false;  // заряд выключен

      outD = MyPidD.step( setpoint[I], fbI );  // коррекция ( откорректировать полярности )
      writePwm( outD );

      dischargeStatus       = true;   // разряд включен с регулированием по току
      pauseStatus           = false;  // пауза отключена
      pidStatus             = true;   // регулятор включен

      #ifdef DEBUG_POWER
        SerialUSB.print(" Discharge: ");     
        SerialUSB.print(" spD: ");    SerialUSB.print( setpoint[D] );     
        SerialUSB.print(" fbI: ");    SerialUSB.print( fbI );
        SerialUSB.print(" outD: 0x"); SerialUSB.println( outD, HEX ); 
      #endif  
      break;

    default:
      break;
    }

    //unsigned long after = micros();
    //SerialUSB.print("runtime,us: "); SerialUSB.println((uint16_t)(after - before));
  }
} //!doPid()

// Сохранение и восстановление регистров регулятора для корректного перехода
void saveState( int mode )
{  
  switch (mode)
  {
    case U:
      sLastSpU  = MyPid.getLastSp();
//      sSumU     = MyPid.getSum();
      sLastErrU = MyPid.getLastErr();
      break;

    case I: 
      sLastSpI  = MyPid.getLastSp();
//      sSumI     = MyPid.getSum();
      sLastErrI = MyPid.getLastErr();
      break;

    default: break;
  }
}

void restoreState( int mode )
{
  switch (mode)
  {
    case U:
      MyPid.setLastSp( sLastSpU );
//      MyPid.setSum( sSumU );            // По невыясненным причинам выполняется некорректно
      MyPid.setLastErr( sLastErrU );
      break;

    case I: 
      MyPid.setLastSp( sLastSpI );
//      MyPid.setSum( sSumI );
      MyPid.setLastErr( sLastErrI );
      break;

    default: break;
  }
}

//   // Компенсация всплеска напряжения
// void surgeCompensation( int16_t uErr )
// {
//   uint16_t val;
//   if( uErr > surgeVoltage )
//   {
//     val = surgeCurrent;
//   }
//   else
//   {
//     val = 0x0000;
//   }
  
//   // #ifdef DEBUG_POWER
//   //   SerialUSB.println( val, HEX);
//   // #endif

//   dacWrite10bit( val );
// }

void idleLoad()
{
  if( maCurrent < idleCurrent ) 
  {   //ток мал
    dacWrite10bit( idleDac );
  }
  else
  { 
    dacWrite10bit( 0 );
  }
}



  // Отключение, в том числе и аварийное
void powerFailure(uint8_t err)
{
  swPinOff();                     // Отключение нагрузки
  switchStatus = false;
  powerStatus = false;
  //chargeStatus = false;
  //dischargeStatus = false;
  errorCode = err;
}


// ======================= Команды управления процессами ======================

// 0x20 Старт заряда с заданными максимальными U и I,
// оно же и для режима источника питания.
// ПИД-регулятор(ы) должны быть сконфигурированы и инициализированы ранее.
void doPowerGo()
{
  if( rxNbt == 5 )
  {
//swPinOn();                                                              // TEST      
    setpoint[1] = get16(0);   // U
    setpoint[2] = get16(2);   // I
    pidMode = 1;              // U - начать с установки напряжения
    pidStatus = true;         // Разрешить регулирование
//swPinOff();                                                              // TEST      
    txReplay( 1, 0 );   // всего байт, в нулевом - сообщение об ошибках
  }
  else txReplay(1, err_tx);
}

// 0x21 Стоп заряд или разряд
void doPowerStop()
{
  if( rxNbt == 0 )
  {
    pidMode = 0;
    txReplay(1, 0);
  }

  else txReplay(1, err_tx);
}

  // 0x22 пока не реализована
void  doSetPid()
{
  if( rxNbt == 5 )
  {
    uint8_t _id   = rxDat[0];
    //int32_t _par  = getI32(1);
    
//SerialUSB.print("  0: 0x"); SerialUSB.println( _id, HEX );

    txReplay( 1, _id );
  }
    else txReplay(1, err_tx);      // Ошибка протокола
} 

// 0x40 Тестовая. Конфигурирование пид-регулятора
void doPidConfigure()
{
  //uint8_t err = 0x00;

  if( rxNbt == 11 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( OFF, U, I, D )
    pidMode = m;
    kP[m] = get16(1);
    kI[m] = get16(3);
    kD[m] = get16(5);
    minOut[m] = get16(7);
    maxOut[m] = get16(9);

    switch ( m )
    {
    case U: case I: MyPid.configure( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    case D:        MyPidD.configure( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    default:       break;
    }
    
    #ifdef DEBUG_PID
      SerialUSB.print(" mode: "); SerialUSB.println( m );
      SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
      SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
      SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
      //SerialUSB.print(" sign: "); SerialUSB.println( signOut[m] );
    #endif
    txReplay( 1, 0 );  
  }
  else  txReplay(1, err_tx);
}

// 0x41 Тестовая: ввод коэффициентов kp, ki, kd для заданного режима
void doPidSetCoefficients()
{
  if( rxNbt == 7 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( OFF, U, I, D )
    pidMode = m;
    kP[m] = get16(1);
    kI[m] = get16(3);
    kD[m] = get16(5);

    switch ( m )
    {
    case U: case I: MyPid.setCoefficients( kP[m], kI[m], kD[m] );      break;
    case D:        MyPidD.setCoefficients( kP[m], kI[m], kD[m] );      break;
    default:       break;
    }

    #ifdef DEBUG_PID
      SerialUSB.print(" mode: "); SerialUSB.println( m );
      SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
      SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
      SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
    #endif

    txReplay( 1, 0 );             // только подтверждение
  }
  else txReplay(1, err_tx);       // Ошибка протокола
}

// 0x42 Тестовая: ввод диапазона вывода
void doPidOutputRange()
{
  if( rxNbt == 5 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( OFF, U, I, D )
    pidMode = m;
    minOut[m] = get16(1);
    maxOut[m] = get16(3);

    switch ( m )
    {
    case U: case I: MyPid.setOutputRange( minOut[m], maxOut[m] );      break;
    case D:        MyPidD.setOutputRange( minOut[m], maxOut[m] );      break;
    default:       break;
    }
    
    txReplay( 1, 0 );
  }
  else txReplay(1, err_tx);
}

// 0x43 Тестовая: set as 0x40 w/o clear
void doPidReconfigure()
{
  if( rxNbt == 10 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( OFF, U, I, D )
    pidMode = m;
    kP[m] = get16(1);
    kI[m] = get16(3);
    kD[m] = get16(5);
    minOut[m] = get16(7);
    maxOut[m] = get16(9);
      // Это та же процедура, только без очистки регистров регулятора:
    //replaceConfigure( m, kP[m], kI[m], kD[m], minOut[m], maxOut[m] );
    switch ( m )
    {
    case U: case I: MyPid.replaceConfig( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    case D:        MyPidD.replaceConfig( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    default:       break;
    }
    
    #ifdef DEBUG_PID
      SerialUSB.print(" mode: "); SerialUSB.println( m );
      SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
      SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
      SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
      //SerialUSB.print(" sign: "); SerialUSB.println( signOut[m] );
    #endif
    txReplay( 1, 0 );  
  }
  else  txReplay(1, err_tx);
}

// 0x44 Очистка регистров регулятора
void doPidClear()
{
  if( rxNbt == 1 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( OFF, U, I, D )

    switch ( m )
    {
    case U: case I: MyPid.clear();      break;
    case D:        MyPidD.clear();      break;
    default:       break;
    }

    txReplay(1, 0);
  }
  else txReplay(1, err_tx);
}

  // 0x46 Тестовая. Тест пид-регулятора
  // Задает ПИД-регулятору режим регулирования U,I или D и задает уровень.
  // В режиме OFF ПИД-регулятор отключен, но схема скоммутирована как для регулирования 
  // по напряжению. Уровень предназначен для подачи непосредственно на PWM с осторожностью. 
void doPidTest()
{
  if( rxNbt == 3 )
  {
    uint8_t   m = rxDat[0] & 0x03;  // 0-1-2-3 - выкл или задать напряжение, ток заряда или ток разряда

    pidMode = m;                    // выбор канала регулирования

    if( pidMode == 0 )
    {
      // включать как регулятор напряжения
      setpoint[1] = get16(1);  
      configMode(U);
      pidStatus = false;            // PID-регулятор выключен
    }
    else
    {
      setpoint[m] = get16(1);  
      configMode(m);
      pidStatus = true;             // PID-регулятор включен
  //  }

    #ifdef DEBUG_PID
      SerialUSB.print("mode: ");  SerialUSB.println( m );
      SerialUSB.print(" sp: 0x"); SerialUSB.println( setpoint[m], HEX );
      //SerialUSB.print("min: 0x"); SerialUSB.println( minOut[m], HEX );
      //SerialUSB.print("max: 0x"); SerialUSB.println( maxOut[m], HEX );
    #endif
    }
    txReplay( 1, 0 );               // только подтверждение
  }
  else txReplay(1, err_tx);         // Ошибка протокола     
}

  // 0x47 Конфигурирование pwm-регулятора
void doPwmConfigure()
{
  uint8_t err = 0x00;

  if( rxNbt == 3 )
  {
    pwmInvert = (bool)get08(0);   // Выбор полярности PWM (v55: для отключения при сбросе - 0x00)
    pwmPeriod = get16(1);         // Выбор частоты (через период)
    initPwm();

    #ifdef DEBUG_PWM
      SerialUSB.print(" F, kHz "); SerialUSB.println( 24000/pwmPeriod );
    #endif

    txReplay( 1, err );  
  }
  else txReplay(1, err_tx);
}

// 0x48 Возвращает параметры текущего режима регулирования
void doPidGetConfigure()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU08( id, pidMode );
    id = replyU16( id, kP[pidMode] );
    id = replyU16( id, kI[pidMode] );
    id = replyU16( id, kD[pidMode] );
    id = replyU16( id, (uint16_t)minOut[pidMode] );
    id = replyU16( id, (uint16_t)maxOut[pidMode] );

    txReplay( id, 0 );   // всего байт, в нулевом - сообщение об ошибках (подтверждение)
  }
  else txReplay(1, err_tx);    // ошибка протокола (пакет не полный)
}

// 0x49 Задает максимальный интеграл при вычислении шага регулирования
void doPidSetMaxSum()
{
  if( rxNbt == 12 )
  {
    //integ_min = get64(0);       // Лучше задавать число знаков и сдвигами вычислять мин и макс
    //integ_max = get64(7);  //знак - !!!

    // #ifdef DEBUG_PID
    //   SerialUSB.print(" mode: "); SerialUSB.println( m );
    //   SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
    //   SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
    //   SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
    // #endif

    txReplay( 1, 0 );             // только подтверждение
  }
  else txReplay(1, err_tx);       // Ошибка протокола
}



  // 0x5B задать параметры компенсации перенапряжения - отменено
void doSurgeCompensation()
{
  if( rxNbt == 4 )
  {
    surgeVoltage = get16(0);        // Милливольты превышения
    surgeCurrent = get16(2);        // Ток в коде DAC

    txReplay( 1, 0 );
  }
  else  txReplay(1, err_tx);                    // ошибка протокола
}

  // 0x5C задать параметры доп. нагрузки на ХХ
void doIdleLoad()
{
  if( rxNbt == 4 )
  {
    idleCurrent  = get16(0);        // Минимальный ток, при котором не нужна дополнительная нагрузка 
    idleDac      = get16(2);        // Ток в коде DAC
    txReplay( 1, 0 );
  }
  else  txReplay(1, err_tx);                    // ошибка протокола
}



// ============================ Команды тестирования ===========================

// Команда 0x54. Управление ключами подключения нагрузки.
// MINI: закомментировать powerFailure() или замкнуть D8,9 и A3,4 
void doSwPin()
{
  if( rxNbt == 1 )
  {
    bool sw = (bool)(rxDat[0] & 0x01);

    if( !sw )
    {
      // При отключении нагрузки снять питание
      // и отключить цепь разряда

      pidMode = OFF;      // При включенном регуляторе отключение автоматическое, ниже - дублирование

      swPinOff();
      switchStatus          = false;  // коммутатор отключен

      writePwm( 0x0000 );
      powerStatus           = false;  // преобразователь выключен
      chargeStatus          = false;
      
      dacWrite10bit( 0x0000 );
      dischargeStatus = false;
    }
    else
    {
      swPinOn();
      switchStatus          = true;  // коммутатор включен
    }

    txReplay( 1, 0 );         // Подтверждение
  }
  else txReplay(1, err_tx);   // ошибка протокола  
}

  // Команда 0x56. Для проверки пределов регулирования преобразователя снизу. 
  // Использовать с осторожностью, только для проверки низковольтной схемы.
  // ПИД-регулятор отключается, коммутатор включен, преобразователь включен
void setPower()
{
  if( rxNbt == 4 )
  {
    pidMode = OFF;      // При включенном регуляторе всё отключится
    pidStatus = false;

    swPinOn();
    switchStatus          = true;   // коммутатор включен

    uint16_t val = get16(0);
    writePwm( val );           //     

    if( val )  powerStatus = true;   // преобразователь включен
      else     powerStatus = false;  // преобразователь выключен

    val = get16(2); 
    dacWrite10bit( val );

    if( val ) dischargeStatus = true; // Схема разряда как нагрузка включена
      else    dischargeStatus = false;
    
    #ifdef DEBUG_POWER
      // Реальные значения будут только в следующем запросе.
      // Ток разрядной цепи не учитывается. 
      SerialUSB.print( "mV: " );    SerialUSB.print( mvVoltage );
      SerialUSB.print( "  mA: " );  SerialUSB.println( maCurrent );
    #endif

    txReplay( 1, 0 );         // Подтверждение
  }
  else txReplay(1, err_tx);   // ошибка протокола
}

// Команда 0x57 - проверка управления цепью разряда.
// Пользоваться с осторожностью, выставив порог отключения
void setDischg()
{
  if( rxNbt == 1 )
  {
    uint8_t err = 0x00;         // зарезервировано
    uint16_t proc = rxDat[0];
    if( proc > 100 ) proc = 100;

    if( proc )  // Включить если не 0%
    {
      proc = 1023 - ( proc * 1023 / 100 );
      dacWrite10bit( proc );

      //dischargeStatus = true;   // коммутатор на разряд
      //chargeStatus = !dischargeStatus;
      powerStatus = false;      // преобразователь выключить
      switchStatus = true;      // к клеммам подключить
    }
    else        // Выключить если 0%
    {
      proc = 1023;
      dacWrite10bit( proc );    //

      //dischargeStatus = true;   // оставить подключенным на разряд
      //chargeStatus = !dischargeStatus;
      powerStatus = false;      // преобразователь выключить
      switchStatus = true;      // от нагрузки не отключать
    }

    #ifdef DEBUG_POWER
      SerialUSB.println( proc );
    #endif

    txReplay( 1, err );         // Подтверждение
  }
  else txReplay(1, err_tx);   // ошибка протокола
}

  // 0x58 Включение и поддержание заданного напряжение в мВ
void doSetVoltage()
{
  if( rxNbt == 3 )
  {
    uint8_t _mode = rxDat[0] & 0x03;    // OFF-U-I-D - выкл или задать напряжение, ток заряда или ток разряда
    int16_t sp    = (int16_t)get16(1);  // Заданная величина в mV или mA

    switch (_mode)
    {
      case U :
      {
        if(sp < volt_min) sp = volt_min;           // Если за пределом - задать минимум
        if(sp > volt_max) sp = volt_max;           // Если за пределом - задать максимум


        setpoint[U] = sp;       // милливольты

      SerialUSB.print("x65_U ");  SerialUSB.println(setpoint[U]);


        // Задать условия, установить напряжение 
        //chargeStatus    = true;               // заряд, иное невозможно
        //dischargeStatus = !chargeStatus;

        swPinOn();                            // sw_pin D5 PA15 - силовые ключи (нагрузку) включить
        switchStatus = true; 
        
//        powPinOn();
        powerStatus = true;                   //         !pwr_pin D4 PA14 - преобразователь включить

        pidStatus = true;   //        _pidStatus;
        voltageControlStatus  = true;   //_pidStatus;   // регулирование по напряжению

        //initPid();  // ****

        SerialUSB.print("kP[U]... ");  SerialUSB.println(kP[U]);

          MyPid.configure( kP[U], kI[U], kD[U], minOut[U], maxOut[U] );
          //writePwm( sp );           // запустить это тест на 20 ... 100мс
      }
      break;

      case I :
      {
        int16_t sp = (int16_t)get16(0);        // Заданное напряжение в милливольтах

        if(sp < curr_ch_min) sp = curr_ch_min;           // Если за пределом - задать минимум
        if(sp > curr_ch_max) sp = curr_ch_max;           // Если за пределом - задать максимум


        setpoint[I] = sp;       // миллиамперы

      SerialUSB.print("x65_I ");  SerialUSB.println(setpoint[I]);

      // ...

      }
      break;

    default:
      // Выключить регулятор
        swPinOff();                            // sw_pin D5 PA15 - силовые ключи (нагрузку) выключить
        switchStatus = false; 
        
//        powPinOff();
        powerStatus = false;                   //      !pwr_pin D4 PA14 - преобразователь выключить

        pidStatus = false;      // 
      SerialUSB.print("x65_OFF ");  SerialUSB.println(setpoint[OFF]);

      break;
    }
  
    // Подготовить 3 байта ответа: 0 - нет ошибок и setpoint
    int id = 1;
    id =  replyU16(id, sp);
    txReplay( id, txDat[0] ); 
  }
  else txReplay(1, err_tx);                   // ошибка протокола
} // !doSetVoltage()

  // 0x59 задать ток в мА и включить
void doSetCurrent()
{
  if( rxNbt == 5 )
  {
    txDat[0] = 0x00;                      // Очистить сообщение об ошибках

    bool _pidStatus    = rxDat[0] & 0x01; // Регулятор отключить или включить
    uint16_t _setpoint = get16(1);        // Заданный ток в миллиамперах
    uint16_t _factor   = get16(3);        // Коэффициент преобразования в код ADC
    
    if(_pidStatus)  // Если задаются миллиамперы
    {
      if(_setpoint <= curr_ch_min)         // Если за пределом
      {
        _setpoint = curr_ch_min;          // Задать минимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      }

      if(_setpoint >= curr_ch_max)         // Если за пределом
      {
        _setpoint = curr_ch_max;          // Задать максимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      }
    } 
    else
    {
      if(_setpoint <= min_pwm)                // Если за пределом
      {
        _setpoint = min_pwm;                 // Задать минимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      }

      if(_setpoint >= max_pwm)                // Если за пределом
      {
        _setpoint = max_pwm;                 // Задать максимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      } 
    }

    uint16_t _value = _setpoint / _factor;

    // Задать условия, установить ток 
    //chargeStatus    = true;               // заряд, иное невозможно
    //dischargeStatus = !chargeStatus;
    switchStatus    = true;               // коммутатор включить     ( foff_pin = 21 D21 PA23 ) 
    powerStatus = true;               // преобразователь включить ( pwr_pin =  2 D4  PA14 )
    pidStatus = _pidStatus;

    if(_pidStatus)
    {
      setpoint[I] = _value;
      // запустить
    }
    else
    {
      dacWrite10bit( _value );            // Задать код
    }

    // Подготовить 3 байта ответа: 0 - нет ошибок и код, который ушел в ADC или setpoint
    txDat[1] = ( _value >> 8) & 0xFF;      // Hi
    txDat[2] =   _value & 0xFF;            // Lo
    txNbt = 3;
    txReplay( txNbt, txDat[0] ); 
  }
  else
  {
    txReplay(1, err_tx);                   // ошибка протокола
  }
} // !doSetCurrent()

  // 0x5A задать код DAC или ток разряда в мА и включить
void doSetDiscurrent()
{
  if( rxNbt == 3 )
  {
    uint8_t   m = rxDat[0] & 0x03;  // OFF - задать код, иначе ток разряда в миллиамперах
    pidMode = m;                    // выбор канала регулирования

    if( m == 0 )
    {
      // Задается код
      pidMode = m = D;              // выбор канала регулирования 
      configMode(m);
      pidStatus = false;            // PID-регулятор выключен
      setpoint[m] = get16(1); 

      if( setpoint[m] >= 0x0400 )  setpoint[m] = 0x3ff; // Если за пределами 10 разрядов
    
      dacWrite10bit( setpoint[m] ); // Задать код

      #ifdef DEBUG_DAC
        SerialUSB.print("mode: ");  SerialUSB.print( m );
        SerialUSB.print(" sp: 0x"); SerialUSB.println( setpoint[m], HEX );
      #endif
    }
    else
    {
      // Задается ток в миллиамперах
      pidMode = m = D;              // выбор канала регулирования
      configMode(m);
      pidStatus = true;             // PID-регулятор включен
      setpoint[m] = get16(1);       // Вводится абсолютное значение
    }
  
    txReplay( 1, 0 ); 
  }
  else  txReplay(1, err_tx);                    // ошибка протокола
}
