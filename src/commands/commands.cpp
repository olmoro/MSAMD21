/*
  Команды, доступные по UART
  Версия от 25 ноября 2020г.
*/

#include "commands.h"
#include "adc_commands.h"
#include "board/mpins.h"
#include "board/mboard.h"
#include "power/power_reg.h"
#include "wake/wake.h"
#include "stdint.h"
#include <Arduino.h>

// Имя устройства
static constexpr char Info[] = {"D21 Rev2020.10.23\n\0"};   //

  // state1
bool switchStatus          = false;  // коммутатор выключен ( sw_pin PA14 )
bool powerStatus           = false;  // преобразователь выключен
bool currentControlStatus  = false;  // регулирование по току отключено
bool voltageControlStatus  = false;  // регулирование по напряжению отключено
bool chargeStatus          = false;  // заряд не запущен
bool dischargeStatus       = false;  // разряд не запущен
bool pauseStatus           = false;  // пауза (резерв)
bool pidStatus             = false;  // управление регулятором - не включен

  // state2
bool overHeatingStatus     = false;  // перегрев
bool overloadStatus        = false;  // перегрузка
bool powerLimitationStatus = false;  // ограничение мощности
bool reversePolarityStatus = false;  // обратная полярность
bool shortCircuitStatus    = false;  // короткое замыкание
bool calibrationStatus     = false;  // калибровка
bool upgradeStatus         = false;  // обновление
bool reserve2Status        = false;  // резерв 2


  // Команды измерений
const uint8_t cmd_read_u_i                  = 0x10; // читать текущее напряжение и ток (мВ и мА)

  // Команды управления
const uint8_t cmd_power_go                  = 0x20; // старт преобразователя с заданными максимальными U и I
const uint8_t cmd_power_stop                = 0x21; // отключение ( и разряда в том числе)


const uint8_t cmd_set_pid                   = 0x22; // set all parameters (в разработке)




  // Команды работы с измерителем напряжения
  // Множитель преобразования в милливольты
const uint8_t cmd_get_factor_u              = 0x30; // Чтение
const uint8_t cmd_set_factor_u              = 0x31; // Запись
const uint8_t cmd_set_factor_default_u      = 0x32; // Возврат к заводскому
  // Параметр сглаживания
const uint8_t cmd_get_smooth_u              = 0x33; // Чтение
const uint8_t cmd_set_smooth_u              = 0x34; // Запись
  // Приборное смещение
const uint8_t cmd_get_offset_u              = 0x35; // Чтение
const uint8_t cmd_set_offset_u              = 0x36; // Запись



  // Команды работы с измерителем тока
  // Множитель преобразования в миллиамперы
const uint8_t cmd_get_factor_i              = 0x38; // Чтение
const uint8_t cmd_set_factor_i              = 0x39; // Запись
const uint8_t cmd_set_factor_default_i      = 0x3A; // Возврат
  // Параметр сглаживания
const uint8_t cmd_get_smooth_i              = 0x3B; // Чтение
const uint8_t cmd_set_smooth_i              = 0x3C; // Запись
  // Приборное смещение
const uint8_t cmd_get_offset_i              = 0x3D; // Чтение
const uint8_t cmd_set_offset_i              = 0x3E; // Запись


  // ПИД-регулятор
const uint8_t cmd_pid_configure             = 0x40; // set mode, kp, ki, kd, min, max
const uint8_t cmd_pid_set_coefficients      = 0x41; // set kp, ki, kd
const uint8_t cmd_pid_output_range          = 0x42; // set min, max
const uint8_t cmd_pid_reconfigure           = 0x43; // set kp, ki, kd,min, max w/o clear
const uint8_t cmd_pid_clear                 = 0x44; // clear
const uint8_t cmd_pid_test                  = 0x46; // mode, setpoint, sw
const uint8_t cmd_pwm_configure             = 0x47; // max, n, invert - подбор полярности PWM. Как раритет
const uint8_t cmd_pid_get_configure         = 0x48; // mode, kP, kI, kD, min, max - возвращает параметры текущего режима регулирования
const uint8_t cmd_pid_set_max_sum           = 0x49; // Задает максимальный интеграл при вычислении шага рег

  // АЦП - настройки
const uint8_t cmd_adc_read_probes           = 0x50; // Read all probes
const uint8_t cmd_adc_get_offset            = 0x51; // Читать смещение АЦП
const uint8_t cmd_adc_set_offset            = 0x52; // Запись смещения АЦП

  // Команды тестовые
const uint8_t cmd_set_switch_pin            = 0x54; // sw_pin D4 PA14

const uint8_t cmd_set_power                 = 0x56; // пользоваться с осторожностью - выяснение пределов регулирования
const uint8_t cmd_set_discharge             = 0x57; // не проверена
const uint8_t cmd_set_voltage               = 0x58; // старая, не проверена
const uint8_t cmd_set_current               = 0x59; // старая, не проверена 
const uint8_t cmd_set_discurrent            = 0x5A; // старая, не проверена
const uint8_t cmd_set_surge_compensation    = 0x5B; // параметры подавления всплеска напряжения na
const uint8_t cmd_set_idle_load             = 0x5C; // параметры доп.нагрузки ХХ

  // Команды задания порогов отключения
const uint8_t cmd_get_win_less_u            = 0x60; // 
const uint8_t cmd_set_win_less_u            = 0x61; // 
const uint8_t cmd_set_win_less_default_u    = 0x62; // 
const uint8_t cmd_get_win_up_u              = 0x63; // 
const uint8_t cmd_set_win_up_u              = 0x64; // 
const uint8_t cmd_set_win_up_default_u      = 0x65; // 

const uint8_t cmd_get_win_less_i            = 0x68; // 
const uint8_t cmd_set_win_less_i            = 0x69; // 
const uint8_t cmd_set_win_less_default_i    = 0x6A; // 
const uint8_t cmd_get_win_up_i              = 0x6B; // 
const uint8_t cmd_set_win_up_i              = 0x6C; // 
const uint8_t cmd_set_win_up_default_i      = 0x6D; // 

  // ЦАП - настройки
  // читать... https://stackoverflow.com/questions/53542591/using-external-vref-for-samd21-dac
enum dac_reference 
{
    /** 1V from the internal band-gap reference*/
    DAC_REFERENCE_INT1V = DAC_CTRLB_REFSEL(0),
    /** Analog V<SUB>CC</SUB> as reference */
    DAC_REFERENCE_AVCC  = DAC_CTRLB_REFSEL(1),
    /** External reference on AREF */
    DAC_REFERENCE_AREF  = DAC_CTRLB_REFSEL(2),
};

  // Переменные - обмена по UART  
extern uint8_t  rxNbt;          //+ принятое количество байт в пакете
extern uint8_t  rxDat[frame];   //+ массив принятых данных
extern uint8_t  command;        // код команды на выполнение
extern uint8_t  txNbt;          // количество байт данных в пакете
extern uint8_t  txDat[frame];   //+ массив данных для передачи

uint8_t cmd = cmd_nop;

uint8_t state1 = 0b00000000;
uint8_t state2 = 0b00000000;

void doInfo();
void doEcho();
void doErr();

  // Проверка запроса на исполнение команды
void doCommand()
{
  cmd = command;

//SerialUSB.print("*");   // TEST

  if( cmd != cmd_nop)
  {
    #ifdef DEBUG_COMMANDS
      SerialUSB.print(" command -> 0x"); SerialUSB.println(cmd, HEX);
    #endif

    switch( cmd )
    {                                                                 //              v57#
        // Команды измерения
      case cmd_read_u_i :                 doReadUI();                 break;  // 0x10   57

        // Команды управления
      case cmd_power_go:                  doPowerGo();                break;  // 0x20   57
      case cmd_power_stop:                doPowerStop();              break;  // 0x21   57 

      case cmd_set_pid:                   doSetPid();                 break;  // 0x22

        // Команды работы с измерителем напряжения 
      case cmd_get_factor_u:              doGetFactorU();             break;  // 0x30   57
      case cmd_set_factor_u:              doSetFactorU();             break;  // 0x31   57
      case cmd_set_factor_default_u:      doSetFactorDefaultU();      break;  // 0x32   57
      case cmd_get_smooth_u:              doGetSmoothU();             break;  // 0x33   57
      case cmd_set_smooth_u:              doSetSmoothU();             break;  // 0x34   57
      case cmd_get_offset_u:              doGetOffsetU();             break;  // 0x35   57
      case cmd_set_offset_u:              doSetOffsetU();             break;  // 0x36   57
      
        // Команды работы с измерителем тока
      case cmd_get_factor_i:              doGetFactorI();             break;  // 0x38   57
      case cmd_set_factor_i:              doSetFactorI();             break;  // 0x39   57
      case cmd_set_factor_default_i:      doSetFactorDefaultI();      break;  // 0x3A   57
      case cmd_get_smooth_i:              doGetSmoothI();             break;  // 0x3B   57
      case cmd_set_smooth_i:              doSetSmoothI();             break;  // 0x3C   57
      case cmd_get_offset_i:              doGetOffsetI();             break;  // 0x3D   57
      case cmd_set_offset_i:              doSetOffsetI();             break;  // 0x3E   57

        // Команды работы с регуляторами
      case cmd_pid_configure:             doPidConfigure();           break;  // 0x40   57
      case cmd_pid_set_coefficients:      doPidSetCoefficients();     break;  // 0x41     *
      case cmd_pid_output_range:          doPidOutputRange();         break;  // 0x42   57
      case cmd_pid_reconfigure:           doPidReconfigure();         break;  // 0x43     *
      case cmd_pid_clear:                 doPidClear();               break;  // 0x44   57
      case cmd_pid_test:                  doPidTest();                break;  // 0x46     *
      case cmd_pwm_configure:             doPwmConfigure();           break;  // 0x47     *
      case cmd_pid_get_configure:         doPidGetConfigure();        break;  // 0x48     *
      case cmd_pid_set_max_sum:           doPidSetMaxSum();           break;  // 0x49    

        // Команды работы с АЦП
      case cmd_adc_read_probes:           doReadProbes();             break;  // 0x50   57
      case cmd_adc_get_offset:            doAdcGetOffset();           break;  // 0x51   57
      case cmd_adc_set_offset:            doAdcSetOffset();           break;  // 0x52   57

        // Команды управления портами управления (в основном тестовые)
      case cmd_set_switch_pin:            doSwPin();                  break;  // 0x54   57
        // Команды тестовые
      case cmd_set_power:                 setPower();                 break;  // 0x56   57
      case cmd_set_discharge:             setDischg();                break;  // 0x57
      case cmd_set_voltage:               doSetVoltage();             break;  // 0x58   .
      case cmd_set_current:               doSetCurrent();             break;  // 0x59   .
      case cmd_set_discurrent:            doSetDiscurrent();          break;  // 0x5A 
      case cmd_set_surge_compensation:    doSurgeCompensation();      break;  // 0x5B   57 na
      case cmd_set_idle_load:             doIdleLoad();               break;  // 0x5C   57

  // Команды задания порогов отключения
      case cmd_get_win_less_u:            doGetWinLtU();              break;  // 0x60; 
      case cmd_set_win_less_u:            doSetWinLtU();              break;  // 0x61; 
      case cmd_set_win_less_default_u:    doSetWinLtDefaultU();       break;  // 0x62; 
      case cmd_get_win_up_u:              doGetWinUpU();              break;  // 0x63; 
      case cmd_set_win_up_u:              doSetWinUpU();              break;  // 0x64; 
      case cmd_set_win_up_default_u:      doSetWinUpDefaultU();       break;  // 0x65; 

      case cmd_get_win_less_i:            doGetWinLtI();              break;  // 0x68; 
      case cmd_set_win_less_i:            doSetWinLtI();              break;  // 0x69;  
      case cmd_set_win_less_default_i:    doSetWinLtDefaultI();       break;  // 0x6A; 
      case cmd_get_win_up_i:              doGetWinUpI();              break;  // 0x6B; 
      case cmd_set_win_up_i:              doSetWinUpI();              break;  // 0x6C;
      case cmd_set_win_up_default_i:      doSetWinUpDefaultI();       break;  // 0x6D; 





        // Команды универсальные
      case cmd_err:                       doErr();                    break;  // 0x01
      case cmd_echo:                      doEcho();                   break;  // 0x02
      case cmd_info:                      doInfo();                   break;  // 0x03

      default: break;
    }
    cmd = cmd_nop;
  }
}

  // передать информацию об устройстве
void doInfo()
{
  char ch = 1;
  int i = 0;

  for( i = 0; i < frame && ch; i++ )
  {
  ch = txDat[i] = Info[i];

  #ifdef DEBUG_WAKE
    Serial.print( ch );
  #endif
  }
  
  txReplay( i, txDat[0] );        // Искусственный прием, об ошибках не сообщается
}

  // передать эхо
void doEcho()
{
  for( int i = 0; i < rxNbt && i < frame; i++ )
  txDat[i] = rxDat[i];
  txReplay( rxNbt, txDat[0] );
  #ifdef DEBUG_WAKE
    Serial.print("команда эхо = "); Serial.print( rxNbt );
  #endif
}

  // ошибка приема пакета
void doErr()
{
  txReplay(1, err_tx);
  #ifdef DEBUG_WAKE
    Serial.println("обработка ошибки");
  #endif
}

  // Формирование регистра состояния 1
void doState1()
{
  switchStatus         ? state1 |= 0b10000000 : state1 &= 0b01111111; 
  powerStatus          ? state1 |= 0b01000000 : state1 &= 0b10111111; 
  currentControlStatus ? state1 |= 0b00100000 : state1 &= 0b11011111; 
  voltageControlStatus ? state1 |= 0b00010000 : state1 &= 0b11101111; 
  chargeStatus         ? state1 |= 0b00001000 : state1 &= 0b11110111; 
  dischargeStatus      ? state1 |= 0b00000100 : state1 &= 0b11111011; 
  pauseStatus          ? state1 |= 0b00000010 : state1 &= 0b11111101; 
  pidStatus            ? state1 |= 0b00000001 : state1 &= 0b11111110;
}

// Формирование регистра состояния 2 
void doState2()
{
  overHeatingStatus     ? state2 |= 0b10000000 : state2 &= 0b01111111; 
  overloadStatus        ? state2 |= 0b01000000 : state2 &= 0b10111111; 
  powerLimitationStatus ? state2 |= 0b00100000 : state2 &= 0b11011111; 
  reversePolarityStatus ? state2 |= 0b00010000 : state2 &= 0b11101111; 
  shortCircuitStatus    ? state2 |= 0b00001000 : state2 &= 0b11110111; 
  calibrationStatus     ? state2 |= 0b00000100 : state2 &= 0b11111011; 
  upgradeStatus         ? state2 |= 0b00000010 : state2 &= 0b11111101; 
  reserve2Status        ? state2 |= 0b00000001 : state2 &= 0b11111110; 
}

void initState1()
{
    // state1
  switchStatus          = false;  // коммутатор ( foff_pin 21 D21 PA23 )
  powerStatus           = false;  // преобразователь
  currentControlStatus  = false;  // регулирование по току
  voltageControlStatus  = false;  // регулирование по напряжению
  chargeStatus          = false;  // заряд
  dischargeStatus       = false;  // разряд
  pauseStatus           = false;  // пауза
  pidStatus             = false;  // управление регулятором
}

void initState2()
{
    // state2
  overHeatingStatus     = false;  // перегрев
  overloadStatus        = false;  // перегрузка
  powerLimitationStatus = false;  // ограничение мощности
  reversePolarityStatus = false;  // обратная полярность
  shortCircuitStatus    = false;  // короткое замыкание
  calibrationStatus     = false;  // калибровка
  upgradeStatus         = false;  // обновление
  reserve2Status        = false;  // резерв 2
}
