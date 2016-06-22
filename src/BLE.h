
#ifndef BLE_h
#define BLE_h

#include <sap/source/ti/sap/sap.h>
#include <Energia.h>
#include "Stream.h"

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

typedef struct
{
  byte              numServices;
  BLE_Service       **services;
} BLE_Profile;

typedef struct
{
  byte advertType; // Same macros as the byte advertType parameter below
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

class BLE : public Stream
{
  private:
  public:
    BLE(void);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    virtual size_t write(uint8_t c);
};


#endif
