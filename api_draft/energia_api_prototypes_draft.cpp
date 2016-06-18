
/*******************************************************************************
 *******************************************************************************
 * Initializing BLE.
 *******************************************************************************
 ******************************************************************************/

BLE_UART ble(int pin_numbers, int port_numbers);
BLE_SPI ble(int pin_numbers, int port_numbers);

int ble.begin(void);
int ble.end(void);




/*******************************************************************************
 *******************************************************************************
 * Defining the GATT profile.
 * The struct parameters can be macros for pointers to the structs.
 *******************************************************************************
 ******************************************************************************/

int ble.useProfile(BLE_Profile *profile); // For predefined profiles
// The following three return their respective handles
int ble.addService(BLE_Service *service);
int ble.addCharacteristic(BLE_Char *characteristic, int serviceHandle);
int ble.addCharacteristic(BLE_Char *characteristic); // Adds to last service
int ble.registerService(int handle);
int ble.registerService(void); // Registers last service
/*
 * It may actually better to have
 * int ble.registerService(BLE_Service *service);
 * and just have the user define the GATT with structs. This way the services
 * and structs can be statically defined at compile time. 
 * Alternatively, the user could #define the number of services and
 * characteristics. Then, if the user calls addService when the maximum already
 * exist, the add will simply fail. This is ok beacuse the GATT profile should
 * be set only at startup and never change after.
 */
/*
 * Functions for adding common specific profiles
 */
int ble.serial(void);
int ble.iBeacon(void);
int ble.nordicBeacon(void);
int ble.uriBeacon(void);
int ble.eddystone(void);

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

/*
 * Example declaration, compiles in Energia:

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

 */




/*******************************************************************************
 *******************************************************************************
 * Controlling the BLE connection
 *******************************************************************************
 ******************************************************************************/

/*
 * For controlling device advertisement
 * Asynchronous commands in the SNP, but synchronous here for ease of use.
 */
int ble.startAdvert(void);
int ble.stopAdvert(void);
/* 
 * Underlying SAPlib fxn takes uint8_t array but we should also provide an
 * interface for char arrays or String objects, which would set only the
 * variable length payload. Flags would be set only using the uint8_t array,
 * though we could provide more granular named functions for setting them.
 * Possibly use "byte" array and internally cast to uint8_t.
 * advType parameter is chosen from a set of macros.
 */
int ble.setAdvertData(uint8_t advType, uint8_t *advertData);
int ble.setAdvertData(uint8_t advType, char *advertData);
int ble.setAdvertData(uint8_t advType, String *advertData);

/* 
 * For managing the GAP parameters (i.e. the physical link configuration)
 * snp.h already defines a struct we could possible alias.
 * Should look into the connHandle attribute of the struct.
 * Asynchronous commands in the SNP, but synchronous here for ease of use.
 */
int ble.setConnParams(BLE_Conn_Params *connParams);
int ble.setMinConnInt(int minConnInt); // Number of 1.25ms time slots
int ble.setMaxConnInt(int maxConnInt); // Number of 1.25ms time slots
int ble.setRespLatency(int respLatency); // Measured in number of connection intervals the slave can miss.
int ble.setBleTimeout(int timeout);

 /*
  * Disconnect from the other BLE device while maintaining local BLE stack.
  * Asynchronous command in the SNP, but synchronous here for ease of use.
  */
ble.terminateConn(void);




/*******************************************************************************
 *******************************************************************************
 * Reading and writing data.
 *******************************************************************************
 ******************************************************************************/

/*
 * For writing and reading values for all characteristics, whether AP defined
 * or controlled by the GATT server. 
 * Thankfully Energia uses a C++ compiler so we have function overloading.
 */
int ble.writeValue(int handle, int value); // ... boolean, char, byte, word, long, unsigned versions, float, double
int ble.writeValue(int handle, string str); // Char array
int ble.writeValue(int handle, String str); // Object, calls fxn for char array

/*
 * Cannot overload by return type so we need the type in the function name.
 */
int ble.readValue_int(int handle); // etc...




/*******************************************************************************
 *******************************************************************************
 * Using BLE for OTA serial communication.
 *******************************************************************************
 ******************************************************************************/
/*
 * These are only usable when the UART service is defined.
 * The ble class can inherit from the Energia Stream class, which inherits
 * from Print. If a function is already defined in Print or Stream, this is
 * simply an interface to it.
 * Must define: available, read, peek, flush, write(uint8_t)
 * Caveat: bytes will be written one at a time to the characteristic value
 * using individual calls to Print::write(uint8_t), will need pointer to
 * correct location in characteristic value
 */
// Functions already implemented in Print
size_t ble.print( ... );
size_t ble.println( ... );
size_t ble.write(const uint8_t *buffer, size_t size); // virtual but may be overriden
// Functions already implemented in Stream
// Timeout has weird implications with low BLE data rate. Added Serial to name
// to not conflict with the physical layer setTimeout
void ble.setSerialTimeout(unsigned long timeout);
bool ble.find(char *target);
bool ble.find(char *target, size_t length);
bool ble.findUntil(char *target, char *terminator);
bool ble.findUntil(char *target, size_t targetLen, char *terminate, size_t termLen);
long ble.parseInt(void);
float ble.parseFloat(void);
size_t ble.readBytes( char *buffer, size_t length);
size_t ble.readBytesUntil( char terminator, char *buffer, size_t length);
String ble.readString(void);
String ble.readStringUntil(char terminator);
// Virtual functions in Print, must be implemented
size_t ble.write(uint8_t);
// Virtual functions in Stream, must be implemented
int ble.available(void);
int ble.read(void);
int ble.peek(void);
void ble.flush(void);




/*******************************************************************************
 *******************************************************************************
 * Material below is just notes for my own use, not part of the Energia BLE API.
 *******************************************************************************
 ******************************************************************************/

/*
 * TO-DO: ADD ABILITY FOR AP TO REQUEST THE GATT SERVER SEND A NOTIFICATION.
 * Requests using SNP_SEND_NOTIF_IND_REQ. GATT server replies with
 * SNP_SEND_NOTIF_IND_REQ
 * SOL: Handle internally according to the CCCD flags.
 */


 /*
 * TO-DO: ADD HANDLING OF CCCD READ/WRITE REQUESTS FROM THE GATT CLIENT. 
 * These are forwared from the GATT server to the AP as an SNP_CCCD_UPDATED_IND.
 * AP must reply using SNP_CCCD_UPDATED_CNF if the response needed flag is set.
 * AP is free to allow or deny CCCD updates based on the application.
 * SOL: Handle internally and allow all changes to CCCD. The CCCD only handles 
 * whether notifications and indications are enabled/disabled.
 */


/*
 * TO-DO: ADD HANDLING OF AP DEFINED CHARACTERISTIC READ/WRITE REQUESTS FROM
 * THE GATT CLIENT. These are forwared from the GATT server to the AP as an
 * SNP_CHAR_<READ, WRITE>_IND. AP must reply using SNP_CHAR_<READ, WRITE>_CNF
 * SOL: Handle internally according to the characteristic permissions.
 */
