
#include <BLE.h>

int heartRateMeasurement = 0;
char char1Value = 0;
int char2Value = 0;
long char3Value = 0;
int char4Value = 0;
int char6Value = 0;
const char *char7Value = "Hello, World!";
String char8Value = String("The quick brown fox jumped over the lazy dog.");

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

BLE_Char *simpleServiceChars[] = {&char1, &char2, &char3, &char4};

BLE_Service simpleService =
{
  {0xF0, 0xFF},
  4, simpleServiceChars
};

BLE_Char char6 =
{
  {0xE6, 0xF7, 0xA2, 0xA6, 0x7F, 0xEF, 0x47, 0xEF, 0xA2, 0x4B, 0x74, 0xA3, 0x1A, 0x88, 0xEB, 0xD6},
  BLE_INDICATABLE,
  "Characteristic 6"
};

BLE_Char char7 =
{
  {0xE7, 0xF7, 0xA2, 0xA6, 0x7F, 0xEF, 0x47, 0xEF, 0xA2, 0x4B, 0x74, 0xA3, 0x1A, 0x88, 0xEB, 0xD6},
  BLE_READABLE | BLE_WRITABLE | BLE_NOTIFIABLE,
  "Characteristic 7"
};

BLE_Char char8 =
{
  {0xE8, 0xF7, 0xA2, 0xA6, 0x7F, 0xEF, 0x47, 0xEF, 0xA2, 0x4B, 0x74, 0xA3, 0x1A, 0x88, 0xEB, 0xD6},
  BLE_READABLE | BLE_WRITABLE | BLE_NOTIFIABLE,
  "Characteristic 8"
};

BLE_Char *testServiceChars[] = {&char6, &char7, &char8};

BLE_Service testService =
{
  {0xE0, 0xF7, 0xA2, 0xA6, 0x7F, 0xEF, 0x47, 0xEF, 0xA2, 0x4B, 0x74, 0xA3, 0x1A, 0x88, 0xEB, 0xD6},
  3, testServiceChars
};

void setup() {
  Serial.begin(115200);
  ble.setLogLevel(BLE_LOG_ERRORS | BLE_LOG_MSGS);
  ble.begin();
  ble.addService(&heartRateService);
  ble.addService(&simpleService);
  ble.addService(&testService);
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  ble.writeValue(&char1, char1Value);
  ble.writeValue(&char2, char2Value);
  ble.writeValue(&char3, char3Value);
  ble.writeValue(&char4, char4Value);
  ble.writeValue(&char6, char6Value);
  ble.writeValue(&char7, char7Value);
  ble.writeValue(&char8, &char8Value);
  ble.setAdvertName("Energia BLE");
  ble.startAdvert();
}

unsigned int timer = 0;
int innerCounter = 0;

// the loop routine runs over and over again forever as a task.
void loop() {
  ble.handleEvents();
  if (millis() - timer >= 1000)
  {
    timer = millis();
    heartRateMeasurement += 1;
    ble.writeValue(&heartRateChar, heartRateMeasurement);
    ble.writeValue(&char4, heartRateMeasurement*2);
    if (innerCounter == 0)
    {
      char6Value += 1;
      ble.writeValue(&char6, char6Value);
    }
    innerCounter = (innerCounter + 1) % 5;
    char1Value = ble.readValue_char(&char1);
    Serial.print("char1Value=");Serial.println((int) char1Value);
    char2Value = ble.readValue_int(&char2);
    Serial.print("char2Value=");Serial.println(char2Value);
    char3Value = ble.readValue_long(&char3);
    Serial.print("char3Value=");Serial.println(char3Value);
    char4Value = ble.readValue_int(&char4);
    Serial.print("char4Value=");Serial.println(char4Value);
    char6Value = ble.readValue_int(&char6);
    Serial.print("char6Value=");Serial.println(char6Value);
    char7Value = ble.readValue_charArr(&char7);
    Serial.print("char7Value=");Serial.println(char7Value);
    char8Value = ble.readValue_String(&char8);
    Serial.print("char8Value=");Serial.println(char8Value);
  }
}
