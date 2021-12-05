/*
  Версия для платы v57 с подачей ШИМ на вывод PA16
*/

#ifndef _POWER_PWM_H_
#define _POWER_PWM_H_

#include "stdint.h"

void initPwm();
void writePwm(uint16_t value);

#endif  //!_POWER_PWM_H_
