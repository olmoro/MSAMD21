/*
  EEPROM like API that uses Arduino Zero's flash memory. 
  Requirements: cmaglie's Flash Storage: https://github.com/cmaglie/FlashStorage
  (c) 2016, A. Christian
*/

#include "Arduino.h"
#include "eeprom.h"

FlashStorage(my_flash_ptr, EEPROM_EMULATION); 

EEPROMClass::EEPROMClass(void) {
  _dirty = false;
  _eeprom.valid = false;  


  _flash = &my_flash_ptr;
}

uint8_t EEPROMClass::read(int address)
{
  return _eeprom.data[address]; 
}

void EEPROMClass::write(int address, uint8_t value)
{
  _dirty = true;
  _eeprom.data[address] = value; 
}

void EEPROMClass::update(int address, uint8_t value)
{
  _eeprom.data[address] = value; 
}

void EEPROMClass::init()
{
  _eeprom = _flash->read();
}

bool EEPROMClass::isValid()
{
  return _eeprom.valid;
}

void EEPROMClass::commit()
{
  if (_dirty) {
    _eeprom.valid=true;
    _flash->write(_eeprom);
  }
}

EEPROMClass EEPROM;
