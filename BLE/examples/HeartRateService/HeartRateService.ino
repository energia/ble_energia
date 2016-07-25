
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
  ble.setAdvertName("HeartRateService");
  ble.startAdvert();
}

void loop() {
  /* Handle any messages received from the communication layer. */
  ble.handleEvents();

  /* Update the measurement every second. */
  if (millis() % 1000 == 0)
  {
    heartRateMeasurement += 1; // A little unrealistic
  }

  /* Register the new value with the BLE layer. */
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  heartRateMeasurement = ble.readValue_int(&heartRateChar);
  Serial.print("heartRateMeasurement=");Serial.println(heartRateMeasurement);
}
