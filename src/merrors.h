#ifndef _MERRORS_H_
#define _MERRORS_H_
/*
* Коды ошибок
*/

#include "stdint.h"

namespace MErrors
{
  constexpr uint8_t pow_out_of_win        = 0x80;
  constexpr uint8_t voltage_out_of_win    = 0x81;
  constexpr uint8_t voltage_negative      = 0x82;
  constexpr uint8_t current_out_of_win    = 0x83;
  constexpr uint8_t disCurrent_out_of_win = 0x84;
           

// Not used

};

#endif // !_MERRORS_H_
