#ifndef _MBOARD_H_
#define _MBOARD_H_

/*
 *
 * Версия от 23 октября 2020г.
 */

#include "stdint.h"

  // Управление дискретными портами
void swPinOn();                 // Включение нагрузки
void swPinOff();                // Отключение нагрузки

void p15PinOn();                // Включение (для осциллографирования)
void p15PinOff();               // Отключение

void p14PinOn();                // Включение (для осциллографирования)
void p14PinOff();               // Отключение

void tstPinOn();                // Включение (для осциллографирования)
void tstPinOff();               // Отключение

void portsInit();               // Инициализация дискретных портов

void configMode(uint8_t mode);  // Конфигурация режимов

#endif  //_MBOARD_H_
