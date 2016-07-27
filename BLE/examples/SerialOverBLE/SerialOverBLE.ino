
#include <BLE.h>

void setup() {
  Serial.begin(115200);
  ble.setLogLevel(BLE_LOG_ALL);
  ble.begin();
  ble.serial();
  ble.setAdvertName("Energia Serial");
  ble.startAdvert();
}

void loop() {
  ble.handleEvents();
  if (Serial.available())
  {
    ble.print(Serial.readString());
  }
  if (ble.available())
  {
    /* All the Serial functions are available with BLE serial. */
    Serial.println(ble.readString());
  }
}
