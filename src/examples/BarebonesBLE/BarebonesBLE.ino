
#include <ble.h>
#define LED RED_LED

void setup() {
  BLE ble = BLE();
  ble.begin();
  pinMode(LED, OUTPUT);     
}

// the loop routine runs over and over again forever as a task.
void loop() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);               // wait for 100 ms
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(2000);               // wait for 100 ms
}
