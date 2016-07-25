
#ifndef BLE_H
#define BLE_H

#include <Energia.h>
#include "Stream.h"
#include "BLETypes.h"

class BLE : public Stream
{
  private:
    uint8_t _portType; // UART or SPI connection with network processor
    uint8_t *advertDataArr[MAX_ADVERT_IDX];

    int resetPublicMembers(void);
    uint8_t advertDataInit(void);
    int setAdvertName(uint8_t advertStringLen, const char *advertString);
    int setSingleConnParam(size_t offset, uint16_t value);
    int writeValue(BLE_Char *bleChar, int len, const char *str);
    int writeValue(BLE_Char *bleChar, const uint8_t *str);
    int setSecurityParam(uint16_t paramID, uint16_t len, uint8_t *pData);

  public:
    int error; // Set to BLE_SUCCESS before conditionally setting
    int opcode; // Command that caused an error. Not guaranteed to be set.

    /*
     * The actual connection parameters used. Set by the async event handler
     * in response to a connection establishment event.
     */
    BLE_Conn_Params usedConnParams;
    uint8_t bleAddr[6];

    bool authKeySet;
    uint32_t authKey;
    int securityState;

    /* Maximum transmission unit */
    uint16_t mtu;

    /* Call when using security and a display methd besides serial */
    displayStringFxn_t displayStringFxn;
    displayUIntFxn_t displayUIntFxn;

    BLE(byte portType=BLE_PORT_UART);

    int begin(void);
    int handleEvents(void);
    int end(void);
    int terminateConn(void);

    bool isConnected(void);
    bool isAdvertising(void);

    int addService(BLE_Service *bleService);

    int startAdvert(void); // Default settings
    int startAdvert(BLE_Advert_Settings *advertSettings);
    int stopAdvert(void);
    int setAdvertData(uint8_t advertType, uint8_t len, uint8_t *advertData);
    int setAdvertName(const char *advertString);
    int setAdvertName(String *advertString);

    int setGattParam(uint8_t serviceID, uint8_t charID,
                     uint16_t len, uint8_t *pData);
    int getGattParam(uint8_t serviceID, uint8_t charID,
                     uint16_t *len, uint8_t *pData);
    int setGapParam(uint16_t paramId, uint16_t value);
    int getGapParam(uint16_t paramId, uint16_t *Value);
    uint8_t *hciCommand(uint16_t opcode, uint16_t len, uint8_t *pData);

    int setConnParams(BLE_Conn_Params_Update_Req *connParams);
    int setMinConnInt(uint16_t intervalMin); // Number of 1.25ms time slots
    int setMaxConnInt(uint16_t intervalMax); // Number of 1.25ms time slots
    int setRespLatency(uint16_t slaveLatency); // Measured in number of connection intervals the slave can miss.
    int setBleTimeout(uint16_t supervisionTimeout);

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

    int setPairingMode(uint8_t param);
    int setIoCapabilities(uint8_t param);
    int useBonding(uint8_t param);
    int eraseAllBonds(void);
    int replaceLruBond(uint8_t param);
    int sendSecurityRequest(void);
    int setWhiteListPolicy(uint8_t policy);

    unsigned int getRand(void);
    void getRevision(BLE_Get_Revision_Rsp *getRevisionRsp);
    void getStatus(BLE_Get_Status_Rsp *getStatusRsp);
    int testCommand(BLE_Test_Command_Rsp *testRsp);

    int serial(void);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    virtual size_t write(uint8_t c);
    virtual size_t write(const uint8_t *buffer, size_t size);
};

extern BLE ble;

#endif
