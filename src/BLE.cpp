
#include <BLE.h>



BLE::BLE(byte portType)
{
  _portType = portType;
}

int BLE::begin(void)
{

}

int BLE::end(void)
{

}

int BLE::useProfile(BLE_Profile *profile)
{

}

int BLE::addService(BLE_Service *service)
{

}

int BLE::addCharacteristic(BLE_Char *characteristic, BLE_Service *service)
{

}

int BLE::addCharacteristic(BLE_Char *characteristic)
{

}

int BLE::registerService(BLE_Service *service)
{

}

int BLE::registerService(void)
{

}

int BLE::startAdvert(void)
{

}

int BLE::startAdvert(BLE_Advert_Settings advertSettings)
{

}

int BLE::stopAdvert(void)
{

}

int BLE::setAdvertData(byte advertType, uint8_t *advertData)
{

}

int BLE::setAdvertData(byte advertType, char *advertData)
{

}

int BLE::setAdvertData(byte advertType, String *advertData)
{

}

int BLE::setConnParams(BLE_Conn_Params *connParams)
{

}

int BLE::setGapParam(int paramId, int Value)
{

}

int BLE::setMinConnInt(int minConnInt)
{

}

int BLE::setMaxConnInt(int maxConnInt)
{

}

int BLE::setRespLatency(int respLatency)
{

}

int BLE::setBleTimeout(int timeout)
{

}

void BLE::terminateConn(void)
{

}

void BLE::terminateConn(byte abruptly)
{

}

int BLE::writeValue(int handle, char value)
{

}

int BLE::writeValue(int handle, unsigned char)
{

}

int BLE::writeValue(int handle, int value)
{

}

int BLE::writeValue(int handle, unsigned int)
{

}


int BLE::writeValue(int handle, long value)
{

}

int BLE::writeValue(int handle, unsigned long)
{

}

int BLE::writeValue(int handle, float value)
{

}

int BLE::writeValue(int handle, double value)
{

}

int BLE::writeValue(int handle, char *str)
{

}

int BLE::writeValue(int handle, String str)
{

}

boolean BLE::readValue_boolean(int handle)
{

}

char BLE::readValue_char(int handle)
{

}

unsigned char BLE::readValue_uchar(int handle)
{

}

byte BLE::readValue_byte(int handle)
{

}

int BLE::readValue_int(int handle)
{

}

unsigned int BLE::readValue_uint(int handle)
{

}

word BLE::readValue_word(int handle)
{

}

long BLE::readValue_long(int handle)
{

}

unsigned long BLE::readValue_ulong(int handle)
{

}

float BLE::readValue_float(int handle)
{

}

double BLE::readValue_double(int handle)
{

}

char* BLE::readValue_string(int handle)
{

}

String BLE::readValue_String(int handle)
{

}


int BLE::serial(void)
{

}

int BLE::iBeacon(void)
{

}

int BLE::nordicBeacon(void)
{

}

int BLE::uriBeacon(void)
{

}

int BLE::eddystone(void)
{

}

int BLE::available(void)
{

}

int BLE::read(void)
{

}

int BLE::peek(void)
{

}

void BLE::flush(void)
{

}

size_t BLE::write(uint8_t c)
{

}

