
#include <BLE.h>

void setup() {
  Serial.begin(115200);
  ble.begin();
  ble.serial();
  ble.setAdvertName("Energia Serial");
  ble.startAdvert();
}

void loop() {
  Serial.println(1);
  if (Serial.available())
  {
    ble.print(Serial.readString());
  }
  if (ble.available())
  {
    Serial.println(ble.readString());
  }
}
