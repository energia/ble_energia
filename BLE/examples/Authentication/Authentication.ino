
#include <BLE.h>
#include "Flags.h"
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

unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("begin ");
  Serial.println(ble.begin());
  Serial.println(ble.error);
  ble.error = BLE_SUCCESS;
  Serial.println("add services:");
  Serial.println(ble.addService(&heartRateService));
  Serial.println(ble.error);
  ble.error = BLE_SUCCESS;
  Serial.println("Initializing values.");
  Serial.println(ble.writeValue(&heartRateChar, heartRateMeasurement));
  Serial.println(ble.error);
  ble.error = BLE_SUCCESS;
  Serial.println("set adv name");
  Serial.println(ble.setAdvertName("Energia BLE"));
  Serial.println(ble.error);
  ble.error = BLE_SUCCESS;
  Serial.println("start adv ");
  Serial.println(ble.startAdvert());
  Serial.println(ble.error);
  ble.error = BLE_SUCCESS;
  Serial.println("Setting security params");
  Serial.println(ble.setPairingMode(BLE_SECURITY_WAIT_FOR_REQUEST));
  Serial.println(ble.error);
  Serial.println(ble.setIoCapabilities(BLE_DISPLAY_ONLY));
  Serial.println(ble.error);
  Serial.println(ble.useBonding(true));
  Serial.println(ble.error);
  ble.error = BLE_SUCCESS;
  Serial.println("Done");
  pinMode(LED, OUTPUT);
}

// the loop routine runs over and over again forever as a task.
void loop() {
  flag0 = 0; flag1 = 0; flag2 = 0; flag3 = 0; flag4 = 0; flag5 = 0;
  ble.handleEvents();
  timer++;
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  heartRateMeasurement += 1;
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  Serial.print("Flag 0:");Serial.println(flag0);
  Serial.print("Flag 1:");Serial.println(flag1);
  Serial.print("Flag 2:");Serial.println(flag2);
  Serial.print("Flag 3:");Serial.println(flag3);
  Serial.print("Flag 4:");Serial.println(flag4);
  Serial.print("Flag 5:");Serial.println(flag5);
}
