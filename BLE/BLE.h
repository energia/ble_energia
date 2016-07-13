
#ifndef BLE_H
#define BLE_H

#include <Energia.h>
#include "Stream.h"
#include "BLETypes.h"

class BLE : public Stream
{
  private:
    uint8_t _portType; // UART or SPI connection with network processor
    uint8_t *nonConnAdvertData = NULL;
    uint8_t *scanRspData = NULL;

    int setAdvertName(int advertStringLen, const char *advertString);
    int writeValue(BLE_Char *bleChar, int len, const char *str);
    void advertDataInit(void);
    bool isSerialEnabled(void);
    int writeValue(BLE_Char *bleChar, const uint8_t *str);

  public:
    int error;

    BLE(byte portType=BLE_PORT_UART);

    int begin(void);
    int end(void);

    int addService(BLE_Service *bleService);

    int startAdvert(void); // Default settings
    int startAdvert(BLE_Advert_Settings *advertSettings);
    int stopAdvert(void);
    int setAdvertData(int advertType, uint8_t len, uint8_t *advertData);
    int setAdvertName(const char *advertString);
    int setAdvertName(String *advertString);

    int setConnParams(BLE_Conn_Params *connParams);
    int setGapParam(int paramId, int Value); // Will probably just copy the network processor docs
    int setMinConnInt(int minConnInt); // Number of 1.25ms time slots
    int setMaxConnInt(int maxConnInt); // Number of 1.25ms time slots
    int setRespLatency(int respLatency); // Measured in number of connection intervals the slave can miss.
    int setBleTimeout(int timeout);

    int terminateConn(void);

    int writeValue(BLE_Char *bleChar, char value);
    int writeValue(BLE_Char *bleChar, unsigned char value);
    int writeValue(BLE_Char *bleChar, int value);
    int writeValue(BLE_Char *bleChar, unsigned int value);
    int writeValue(BLE_Char *bleChar, long value);
    int writeValue(BLE_Char *bleChar, unsigned long value);
    int writeValue(BLE_Char *bleChar, float value);
    int writeValue(BLE_Char *bleChar, double value);
    int writeValue(BLE_Char *bleChar, const char *str); // Char array
    int writeValue(BLE_Char *bleChar, String str); // Object, calls fxn for char array
    boolean readValue_boolean(BLE_Char *bleChar);
    char readValue_char(BLE_Char *bleChar);
    unsigned char readValue_uchar(BLE_Char *bleChar);
    byte readValue_byte(BLE_Char *bleChar);
    int readValue_int(BLE_Char *bleChar);
    unsigned int readValue_uint(BLE_Char *bleChar);
    word readValue_word(BLE_Char *bleChar);
    long readValue_long(BLE_Char *bleChar);
    unsigned long readValue_ulong(BLE_Char *bleChar);
    float readValue_float(BLE_Char *bleChar);
    double readValue_double(BLE_Char *bleChar);
    char* readValue_string(BLE_Char *bleChar);
    String readValue_String(BLE_Char *bleChar);

    int serial(void);
    int iBeacon(void);
    int nordicBeacon(void);
    int uriBeacon(void);
    int eddystone(void);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    virtual size_t write(uint8_t c);
    virtual size_t write(const uint8_t *buffer, size_t size);
};

extern BLE ble;

#endif
