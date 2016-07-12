
#include <BLE.h>
#define LED RED_LED

BLE ble;
unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  ble = BLE();
  ble.begin();
  ble.serial();
  ble.setAdvertName("Energia Serial");
  ble.startAdvert();
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  // Write ASCII in hex from 0x21 through 0x7E
  ble.print((timer % (0x7E - 0x21 + 1)) + 0x21);
  Serial.println(ble.readString())
}
