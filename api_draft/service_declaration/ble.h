
#include <Arduino.h>

#define BLE_READABLE 0x02
#define BLE_NOTIFY 0x01
#define BLE_INT 14

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
