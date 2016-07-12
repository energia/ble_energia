
#include <BLE.h>
#define LED RED_LED

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

BLE ble;

void setup() {
  Serial.begin(115200);
  ble = BLE();
  ble.begin();
  ble.addService(&heartRateService);
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  ble.setAdvertName("HeartRateService");
  ble.startAdvert();
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for 100 ms
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for 100 ms
  heartRateMeasurement += 1;
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  heartRateMeasurement = ble.readValue_int(&heartRateChar);
  Serial.print("heartRateMeasurement=");Serial.println(heartRateMeasurement);
}
