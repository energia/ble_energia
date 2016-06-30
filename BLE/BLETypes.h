
#ifndef BLE_TYPES_H
#define BLE_TYPES_H

#include <sap.h>

#include <Energia.h>

/* Type of connection between the user application's
   processor and the bluetooth chip */
#define BLE_PORT_LOCAL          SAP_PORT_LOCAL // unsupported
#define BLE_PORT_UART           SAP_PORT_REMOTE_UART
#define BLE_PORT_SPI            SAP_PORT_REMOTE_SPI // unsupported

/* Advertising types */
#define BLE_ADV_DATA_NOTCONN    SAP_ADV_DATA_NOTCONN
#define BLE_ADV_DATA_CONN       SAP_ADV_DATA_CONN
#define BLE_ADV_DATA_SCANRSP    SAP_ADV_DATA_SCANRSP

/* Characteristic value properties */
#define BLE_READABLE            SNP_GATT_PROP_READ
#define BLE_WRITABLE_NORSP      SNP_GATT_PROP_WRITE_NORSP
#define BLE_WRITABLE            SNP_GATT_PROP_WRITE
#define BLE_NOTIFIABLE          SNP_GATT_PROP_NOTIFICATION
#define BLE_INDICATABLE         SNP_GATT_PROP_INDICATION

/*
 * Characteristic value formatting, Bluetooth spec Vol 3: Part G: 3.3.3.5.2
 */
#define BLE_UINT                 0x06

typedef struct
{
  byte              UUIDlen; // in bytes, either 2 (16 bit) or 16 (128 bit)
  byte              UUID[16]; // array of UUID bytes, little-endian
  byte              properties; // bitwise OR of macros: e.g. BLE_READABLE | BLE_WRITABLE
  // Null terminated; internally set permissions to read only so we don't have to worry about the length changing
  char              *charDesc;
  byte              valueFormat;
  byte              valueExponent; // only used with integer formats, e.g. value = storedValue*10^valueExponent
} BLE_Char;

typedef struct
{
  byte              UUIDlen; // in bytes
  byte              UUID[16]; // array of UUID bytes
  int               numChars;
  BLE_Char          *chars;
} BLE_Service;

typedef struct
{
  byte              numServices;
  BLE_Service       **services;
} BLE_Profile;

typedef struct
{
  byte advertType;
  int timeout; // How long to advertise for (in ms), 0 for indefinitely
  int interval; // Advertising Interval (n * 0.625 ms), 0 for 100ms default
  /* 0x00   Advertising is disabled during connection and will not start after.
   * 0x01   Advertising will continue with non-connectable advertising when connection is established
        There are separate GAP parameters for setting connected advertising interval.
   * 0x02   Advertising will restart with connectable advertising when a connection is terminated.
   */
  byte connectedBehavior;
} BLE_Advert_Settings;

typedef struct
{
  int minConnInt;
  int maxConnInt;
  int respLatency;
  int bleTimeout;
} BLE_Conn_Params;

#endif