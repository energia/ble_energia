
#include <BLE.h>
#include "Flags.h"
#include "BLEServices.h"
#define LED RED_LED

int heartRateMeasurement = 0;
byte char1Value = 0;
int char2Value = 0;
long char3Value = 0;
int char4Value = 0;
int char6Value = 0;
char *char7Value = "Hello, World!";
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

//BLE_Advert_Settings advertSettings =
//{
//  BLE_ADV_MODE_CONN,
//  0, 160,
//  BLE_ADV_RESTART_ON_CONN_TERM
//};

unsigned long timer = 0;
unsigned long start;

void setup() {
  Serial.begin(115200);
  Serial.println("begin ");
  ble.begin();
  Serial.println("add services:");
  ble.addService(&heartRateService);
  ble.addService(&simpleService);
  ble.addService(&testService);
  ble.serial();
  Serial.println("Initializing values.");
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  ble.writeValue(&char1, char1Value);
  ble.writeValue(&char2, char2Value);
  ble.writeValue(&char3, char3Value);
  ble.writeValue(&char4, char4Value);
  ble.writeValue(&char6, char6Value);
  ble.writeValue(&char7, char7Value);
  ble.writeValue(&char8, char8Value);
  Serial.println("set adv data ");
  ble.setAdvertName("Energia BLE");
  Serial.println("start adv ");
//  Serial.println(ble.startAdvert(&advertSettings));
  Serial.println(ble.startAdvert());
  Serial.println("Done");
  pinMode(LED, OUTPUT);
  start = millis();
}

// the loop routine runs over and over again forever as a task.
void loop() {
  flag0 = 0; flag1 = 0; flag2 = 0; flag3 = 0; flag4 = 0; flag5 = 0;
  timer++;
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for 100 ms
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for 100 ms
  heartRateMeasurement += 1;
  ble.writeValue(&heartRateChar, heartRateMeasurement);
  ble.writeValue(&char4, heartRateMeasurement*2);
  if (timer % 5 == 0)
  {
    ble.writeValue(&char6, char6Value + 1);
  }
  // Write ASCII in hex from 0x21 through 0x7E
  ble.print((char) ((timer % (0x7E - 0x21 + 1)) + 0x21));
  if (Serial.available())
  {
    Serial.println("RFS");
    String stringFromSerial = Serial.readString();
    Serial.print("read:");Serial.println(stringFromSerial);
    ble.print(stringFromSerial);
    Serial.println("printed");
  }
  char1Value = ble.readValue_byte(&char1);
  Serial.print(ble.error);Serial.print(" char1Value=");Serial.println(char1Value);
  char2Value = ble.readValue_int(&char2);
  Serial.print(ble.error);Serial.print(" char2Value=");Serial.println(char2Value);
  char3Value = ble.readValue_long(&char3);
  Serial.print(ble.error);Serial.print(" char3Value=");Serial.println(char3Value);
  char4Value = ble.readValue_int(&char4);
  Serial.print(ble.error);Serial.print(" char4Value=");Serial.println(char4Value);
  char6Value = ble.readValue_int(&char6);
  Serial.print(ble.error);Serial.print(" char6Value=");Serial.println(char6Value);
  char7Value = ble.readValue_string(&char7);
  Serial.print(ble.error);Serial.print(" char7Value=");Serial.println(char7Value);
  char8Value = ble.readValue_String(&char8);
  Serial.print(ble.error);Serial.print(" char8Value=");Serial.println(char8Value);
  String uartValue = ble.readValue_String(&txChar);
  Serial.print(ble.error);Serial.print(" txCharValue=");Serial.println(uartValue);
  uartValue = ble.readValue_String(&rxChar);
  Serial.print(ble.error);Serial.print(" rxCharValue=");Serial.println(uartValue);
  uartValue = ble.readString();
  Serial.print(ble.error);Serial.print(" actualRxValue=");Serial.println(uartValue);
  Serial.print("Flag 0:");Serial.println(flag0);
  Serial.print("Flag 1:");Serial.println(flag1);
  Serial.print("Flag 2:");Serial.println(flag2);
  Serial.print("Flag 3:");Serial.println(flag3);
  Serial.print("Flag 4:");Serial.println(flag4);
  Serial.print("Flag 5:");Serial.println(flag5);
}
