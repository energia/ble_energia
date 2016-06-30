
#include <BLE.h>
#define LED RED_LED

BLE_Char heartRateChar =
{
  2, {0x37, 0x2A}, // UUID for heart rate measurement, 16 bits
  BLE_READABLE,
  "Heart rate measurement",
};

BLE_Char allChars[] = {heartRateChar}; // Alternative to this would be to fix the nu

BLE_Service heartRateService =
{
  2, {0x0D, 0x18}, // UUID for heart rate service. 16 bits
  1, allChars
};

BLE_Advert_Settings advertSettings =
{

};

BLE ble;
int numBlinks;
int cnt = 0;

void setup() {
  Serial.begin(115200);
  ble = BLE();
  Serial.println("begin");
  Serial.println(ble.begin());
  Serial.println("done");
  Serial.println("add service");
  Serial.println(ble.addService(&heartRateService));
  Serial.println("done");
  pinMode(LED, OUTPUT);
}

// the loop routine runs over and over again forever as a task.
void loop() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for 100 ms
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for 100 ms
}
