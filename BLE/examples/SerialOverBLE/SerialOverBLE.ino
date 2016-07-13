
#include <BLE.h>
#define LED RED_LED

void setup() {
  Serial.begin(115200);
  ble.begin();
  ble.serial();
  ble.setAdvertName("Energia Serial");
  ble.startAdvert();
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  if (Serial.available())
  {
    ble.print(Serial.readString());
  }
  if (ble.available())
  {
    Serial.println(ble.readString());
  }
}
