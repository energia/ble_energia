
#include <BLE.h>
#define LED RED_LED

int heartRateMeasurement = 0;
byte char1Value = 0;
int char2Value = 0;
long char3Value = 0;
int char4Value = 0;

BLE_Char heartRateChar =
{
  2, {0x37, 0x2A}, // UUID for heart rate measurement, 16 bits
  BLE_READABLE,
  "Heart rate measurement"
};

BLE_Char *heartRateChars[] = {&heartRateChar};

BLE_Service heartRateService =
{
  2, {0x0D, 0x18}, // UUID for heart rate service. 16 bits
  1, heartRateChars
};

BLE_Char char1 =
{
  2, {0xF1, 0xFF},
  BLE_READABLE | BLE_WRITABLE,
  "Characteristic 1"
};

BLE_Char char2 =
{
  2, {0xF2, 0xFF},
  BLE_READABLE,
  "Characteristic 2"
};

BLE_Char char3 =
{
  2, {0xF3, 0xFF},
  BLE_WRITABLE,
  "Characteristic 3"
};

BLE_Char char4 =
{
  2, {0xF4, 0xFF},
  BLE_NOTIFIABLE,
  "Characteristic 4"
};

BLE_Char *simpleServiceChars[] = {&char1, &char2, &char3, &char4};

BLE_Service simpleService =
{
  2, {0xF0, 0xFF},
  4, simpleServiceChars
};


static uint8_t scanRspData[] = {
  // complete name
  0xc,// length of this data
  SAP_GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'E', 'n', 'e', 'r', 'g', 'i', 'a', ' ',
  'B', 'L', 'E',

  // connection interval range
  0x05,   // length of this data
  0x12, //GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( BLE_DEF_DESIRED_MIN_CONN_INT ),
  HI_UINT16( BLE_DEF_DESIRED_MIN_CONN_INT ),
  LO_UINT16( BLE_DEF_DESIRED_MAX_CONN_INT ),
  HI_UINT16( BLE_DEF_DESIRED_MAX_CONN_INT ),

  // Tx power level
  0x02,   // length of this data
  0x0A, //GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

BLE ble;
int numBlinks;
int cnt = 0;

void setup() {
  Serial.begin(115200);
  ble = BLE();
  Serial.println("begin ");
  ble.begin();
  Serial.println("add services:");
  ble.addService(&heartRateService);
  ble.addService(&simpleService);
  Serial.println("Initializing values.");
  ble.writeValue(heartRateChar.handle, heartRateMeasurement);
  ble.writeValue(char1.handle, char1Value);
  ble.writeValue(char2.handle, char2Value);
  ble.writeValue(char3.handle, char3Value);
  ble.writeValue(char4.handle, char4Value);
  Serial.println("set adv data ");
  ble.setAdvertData(BLE_ADV_DATA_SCANRSP, sizeof(scanRspData), scanRspData);
  Serial.println("start adv ");
  ble.startAdvert();
  Serial.println("Done");
  pinMode(LED, OUTPUT);
}

// the loop routine runs over and over again forever as a task.
void loop() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for 100 ms
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for 100 ms
  heartRateMeasurement += 1;
  ble.writeValue(heartRateChar.handle, heartRateMeasurement);
  ble.writeValue(char4.handle, heartRateMeasurement*2);
  char1Value = ble.readValue_byte(char1.handle);
  Serial.print(ble.error);Serial.print(" char1Value=");Serial.println(char1Value);
  char2Value = ble.readValue_int(char2.handle);
  Serial.print(ble.error);Serial.print(" char2Value=");Serial.println(char2Value);
  char3Value = ble.readValue_long(char3.handle);
  Serial.print(ble.error);Serial.print(" char3Value=");Serial.println(char3Value);
  char4Value = ble.readValue_int(char4.handle);
  Serial.print(ble.error);Serial.print(" char4Value=");Serial.println(char4Value);
  Serial.print("Flag 0:");Serial.println(flag0);
  Serial.print("Flag 1:");Serial.println(flag1);
  Serial.print("Flag 2:");Serial.println(flag2);
  Serial.print("Flag 3:");Serial.println(flag3);
}
