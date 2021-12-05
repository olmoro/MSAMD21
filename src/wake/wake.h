/*
 *
 *
 */

#ifndef _WAKE_H_
#define _WAKE_H_

#include "stdint.h"

// Константы протокола
const uint8_t fend     = 0xC0; // Frame END
const uint8_t fesc     = 0xDB; // Frame ESCape
const uint8_t tfend    = 0xDC; // Transposed Frame END
const uint8_t tfesc    = 0xDD; // Transposed Frame ESCape
const uint8_t crc_init = 0xDE; // Initial CRC value
const uint8_t frame    = 0xFF; // Максимальная длина пакета 255 (полезных данных)

// Коды ошибок:
const uint8_t err_no   = 0x00; // no error
const uint8_t err_tx   = 0x01; // Rx/Tx error
const uint8_t err_bu   = 0x02; // device busy error
const uint8_t err_re   = 0x03; // device not ready error
const uint8_t err_pa   = 0x04; // parameters value error
const uint8_t err_nr   = 0x05; // no replay
const uint8_t err_nc   = 0x06; // no carrier

  // Коды универсальных команд:
const uint8_t cmd_nop  = 0x00; // нет операции
const uint8_t cmd_err  = 0x01; // ошибка приема пакета
const uint8_t cmd_echo = 0x02; // передать эхо
const uint8_t cmd_info = 0x03; // передать информацию об устройстве

void wakeInit( uint8_t addr, long time );
void wakeRead();
void txReplay(uint8_t n, uint8_t err);          // передача ответа на команду


int replyU08(int id, uint8_t  value);
int replyU16(int id, uint16_t value);
int replyU32(int id, uint32_t value);


uint16_t get08(int id);
uint16_t get16(int id);

float getF16(int i);
int32_t getI32(int i);
void testReply( int n );                        // тест отправить n байт из буфера приемника

#endif //!_WAKE_H_