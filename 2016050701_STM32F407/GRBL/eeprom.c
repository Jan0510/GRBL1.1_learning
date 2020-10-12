#include "grbl.h"

// ��EEPROM��ȡһ�ֽ����� (checked)
unsigned char eeprom_get_char( unsigned int addr )
{
	return AT24CXX_ReadOneByte(addr);
}

// ��EEPROMдһ�ֽ����� (checked)
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	AT24CXX_WriteOneByte(addr,new_value);
}

// ��EEPROMдһ�����ȵ�����(checked)
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size)
{
  unsigned char checksum = 0;
  for(; size > 0; size--) 
	{ 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

// ��EEPROM��ȡһ�����ȵ�����(checked)
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) 
	{ 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}
