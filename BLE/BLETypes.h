
#ifndef BLE_TYPES_H
#define BLE_TYPES_H

#include <sap.h>
#include <hal_defs.h>

#include <Energia.h>

/* Energia BLE status codes */
#define BLE_SUCCESS             0
#define BLE_INVALID_HANDLE      1

/* Type of connection between the user application's
   processor and the bluetooth chip */
#define BLE_PORT_LOCAL          SAP_PORT_LOCAL // unsupported
#define BLE_PORT_UART           SAP_PORT_REMOTE_UART
#define BLE_PORT_SPI            SAP_PORT_REMOTE_SPI // unsupported

/*
 * For setAdvertData.
 * Data to advertise when not connected, connected, and when scanned.
 * The first two are passively seen by a client, while scanning is an
 * active action to request more data.
 */
#define BLE_ADV_DATA_NOTCONN    SAP_ADV_DATA_NOTCONN
#define BLE_ADV_DATA_CONN       SAP_ADV_DATA_CONN
#define BLE_ADV_DATA_SCANRSP    SAP_ADV_DATA_SCANRSP

/*
 * For startAdvert.
 * Connectable undirected adverisement.
 * Scannable undirected advertisement.
 * Non-connectable undirected advertisement.
 */
#define BLE_ADV_MODE_CONN       SNP_ADV_TYPE_CONN
#define BLE_ADV_MODE_SCANABLE   SNP_ADV_TYPE_SCANABLE
#define BLE_ADV_MODE_NONCONN    SNP_ADV_TYPE_NONCONN

/* Characteristic value properties */
#define BLE_READABLE            SNP_GATT_PROP_READ
#define BLE_WRITABLE_NORSP      SNP_GATT_PROP_WRITE_NORSP
#define BLE_WRITABLE            SNP_GATT_PROP_WRITE
#define BLE_NOTIFIABLE          SNP_GATT_PROP_NOTIFICATION
#define BLE_INDICATABLE         SNP_GATT_PROP_INDICATION

// Minimum connection interval (units of 1.25ms, 6=7.5ms) if automatic
// parameter update request is enabled
#define BLE_DEF_DESIRED_MIN_CONN_INT     6

// Maximum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define BLE_DEF_DESIRED_MAX_CONN_INT     80

 // Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D
#define TI_ST_DEVICE_ID                       0x03
#define TI_ST_KEY_DATA_ID                     0x00

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
  int               handle;
  void              *_value;
  int               _valueLen;
} BLE_Char;

typedef struct
{
  byte              UUIDlen; // in bytes
  byte              UUID[16]; // array of UUID bytes
  int               numChars;
  BLE_Char          **chars;
  int               handle;
} BLE_Service;

typedef struct
{
  byte              numServices;
  BLE_Service       **services;
} BLE_Profile;

typedef struct
{
  byte advertMode;
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

static uint8_t defScanRspData[] = {
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

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t defAdvertData[] =
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

#endif