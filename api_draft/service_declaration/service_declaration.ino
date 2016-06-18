
#include "ble.h"

BLE_Char heartRateChar =
{
  2, {0x37, 0x2A}, // UUID for heart rate measurement, 16 bits
  BLE_READABLE,
  "Heart rate measurement",
  BLE_NOTIFY,
  BLE_INT, 0 // signed 16 bit integer (Energia int), multiplied by 10^0 = 1
};

BLE_Char *allChars = {&heartRateChar}; // Alternative to this would be to fix the nu

BLE_Service heartRateService = 
{
  2, {0x0D, 0x18}, // UUID for heart rate service. 16 bits
  allChars
};

int addService(BLE_Service* service)
{
  return 0xFFFF; // Fake handle
}

int registerService(int handle)
{
  return 0;
}

void setup()
{
  registerService(addService(&heartRateService));
}

void loop()
{
}
