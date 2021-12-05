/*
  EEPROM like API that uses Arduino Zero's flash memory. 
  Requirements: cmaglie's Flash Storage: https://github.com/cmaglie/FlashStorage
  (c) 2016, A. Christian
*/

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <FlashStorage.h>

#ifndef EEPROM_EMULATION_SIZE
  #define EEPROM_EMULATION_SIZE 1024
#endif

typedef struct 
{
  byte data[EEPROM_EMULATION_SIZE];
  boolean valid;  
} EEPROM_EMULATION;


class EEPROMClass 
{
  public:
    EEPROMClass(void);
    uint8_t read(int);
    void write(int, uint8_t);
    void update(int, uint8_t);
    bool isValid();
    void init();
    void commit();

  protected:
    EEPROM_EMULATION _eeprom;
    bool _dirty;
    FlashStorageClass<EEPROM_EMULATION> *_flash;
};

extern EEPROMClass EEPROM;

#endif  //!_EEPROM_H_
