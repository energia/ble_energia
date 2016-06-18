
typedef struct 
{
  byte              UUIDlen; // in bytes, either 2 (16 bit) or 16 (128 bit)
  byte              UUID[16]; // array of UUID bytes, little-endian 
  byte              properties; // bitwise OR of macros: e.g. BLE_READABLE | BLE_WRITABLE
  // Null terminated; internally set permissions to read only so we don't have to worry about the length changing
  char              *charDesc;
  // Client Characteristic Configuration Descriptor, enables/disables notifcations and indications to the client
  byte              CCCD;
  byte              valueFormat;
  byte              valueExponent; // only used with integer formats, e.g. value = storedValue*10^valueExponent
} BLE_Char;

typedef struct
{
  byte              UUIDlen; // in bytes
  byte              UUID[16]; // array of UUID bytes
  BLE_Char          *chars;
} BLE_Service;

#define BLE_READABLE 0x02
#define BLE_NOTIFY 0x01
#define BLE_INT 14

BLE_Char heartRateChar =
{
  2, {0x37, 0x2A}, // UUID for heart rate measurement, 16 bits
  BLE_READABLE,
  "Heart rate measurement",
  BLE_NOTIFY,
  BLE_INT, 0 // signed 16 bit integer (Energia int), multiplied by 10^0 = 1
};

BLE_Char *allChars = {&heartRateChar}; // Alternative to this would be to fix the nu

BLE_Service heartRateService = 
{
  2, {0x0D, 0x18}, // UUID for heart rate service. 16 bits
  allChars
};


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(heartRateService.UUIDlen);
  Serial.println(heartRateService.UUID[0]);
  Serial.println(heartRateService.UUID[1]);
}

void loop()
{
  // put your main code here, to run repeatedly:
  
}
