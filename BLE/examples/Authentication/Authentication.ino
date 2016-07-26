
#include <BLE.h>

int heartRateMeasurement = 0;

BLE_Char heartRateChar =
{
  {0x37, 0x2A}, // UUID for heart rate measurement, 16 bits
  BLE_READABLE,
  "Heart rate measurement"
};

BLE_Char *heartRateChars[] = {&heartRateChar};

BLE_Service heartRateService =
{
  {0x0D, 0x18}, // UUID for heart rate service. 16 bits
  1, heartRateChars
};

void setup() {
  Serial.begin(115200);
  ble.begin();
  ble.addService(&heartRateService);
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  ble.setAdvertName("Energia BLE");
  ble.startAdvert();
  ble.setPairingMode(BLE_SECURITY_WAIT_FOR_REQUEST);
  ble.setIoCapabilities(BLE_DISPLAY_YES_NO);
  ble.useBonding(true);
}

void loop() {
  ble.handleEvents();
  heartRateMeasurement += 1;
  ble.writeValue(&heartRateChar, heartRateMeasurement);
}
