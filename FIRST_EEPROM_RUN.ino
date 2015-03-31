//Set every EEPROM temperature value to 375 deg C

#include <EEPROM.h>

int addr = 0;

void setup()
{
}

void loop()
{
  int val1 = 1;
  int val2 = 119;

  if (addr % 2 == 1) {
    EEPROM.write(addr, val1);
  }
  else {
    EEPROM.write(addr, val2);
  }

  addr = addr + 1;
  if (addr == 512)
    addr = 0;

  delay(100);
}
