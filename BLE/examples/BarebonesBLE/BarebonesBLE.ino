
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

static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  SAP_GAP_ADTYPE_FLAGS,
  SAP_GAP_ADTYPE_FLAGS_GENERAL | SAP_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // Manufacturer specific advertising data
  0x06,
  0xFF, //GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  LO_UINT16(TI_COMPANY_ID),
  HI_UINT16(TI_COMPANY_ID),
  TI_ST_DEVICE_ID,
  TI_ST_KEY_DATA_ID,
  0x00                                    // Key state
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
  Serial.println("set adv data");
  Serial.println(ble.setAdvertData(BLE_ADV_DATA_SCANRSP, sizeof(scanRspData), scanRspData));
  Serial.println(ble.setAdvertData(BLE_ADV_DATA_NOTCONN, sizeof(advertData), advertData));
  Serial.println("done");
  Serial.println("start adv");
  Serial.println(ble.startAdvert());
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
