
#include <BLE.h>

byte char1Value = 0;
int char2Value = 0;
long char3Value = 0;
String char4Value = String("Hello, world!");

BLE_Char char1 =
{
  {0xF1, 0xFF},
  BLE_READABLE | BLE_WRITABLE,
  "Characteristic 1"
};

BLE_Char char2 =
{
  {0xF2, 0xFF},
  BLE_READABLE,
  "Characteristic 2"
};

BLE_Char char3 =
{
  {0xF3, 0xFF},
  BLE_WRITABLE,
  "Characteristic 3"
};

BLE_Char char4 =
{
  {0xF4, 0xFF},
  BLE_NOTIFIABLE,
  "Characteristic 4"
};

BLE_Char char5 =
{
  {0xF5, 0xFF},
  BLE_READABLE,
  "Characteristic 5"
};

BLE_Char *simpleServiceChars[] = {&char1, &char2, &char3, &char4, &char5};

BLE_Service simpleService =
{
  {0xF0, 0xFF},
  5, simpleServiceChars
};

void setup() {
  Serial.begin(115200);
  ble.begin();
  ble.addService(&simpleService);
  ble.writeValue(&char1, char1Value);
  ble.writeValue(&char2, char2Value);
  ble.writeValue(&char3, char3Value);
  ble.writeValue(&char4, char4Value);
  ble.setAdvertName("Simple Profile");
  ble.startAdvert();
}

// the loop routine runs over and over again forever as a task.
void loop() {
  char1Value = ble.readValue_byte(&char1);
  Serial.print("char1Value=");Serial.println(char1Value);
  char2Value = ble.readValue_int(&char2);
  Serial.print("char2Value=");Serial.println(char2Value);
  char3Value = ble.readValue_long(&char3);
  Serial.print("char3Value=");Serial.println(char3Value);
  char4Value = ble.readValue_String(&char4);
  Serial.print("char4Value=");Serial.println(char4Value);
}
