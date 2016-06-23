
#include <BLE.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Event.h>

// Application Events
#define AP_NONE                              Event_Id_NONE   // No Event
#define AP_EVT_PUI                           Event_Id_00     // Power-Up Indication

Event_Handle apEvent;

BLE::BLE(byte portType)
{
  _portType = portType;
}

int BLE::begin(void)
{
  /* AP_init() in simple_ap.c */
  apEvent = Event_create(NULL, NULL);
  /*
   * Use this to do something at the applicaiton level on a write or
   * config change. In each SAP service struct, we set read, write, and
   * config change callbacks. In simple_ap, the function below
   * registers its parameters as functions called by the callbacks.
   * In other words, this isn't actually necessary for SAPlib or BLE.
   * It's just included for now for completeless with simple_ap.
   */
  // SimpleProfile_RegisterAppCB(AP_SPWriteCB, AP_SPcccdCB);
  /* End AP_init() */

  SAP_Params sapParams;
  SAP_initParams(_portType, &sapParams);
  SAP_open(&sapParams);

  /*
   * Register callback to receive asynchronous requests from the NP.
   * This must be called before using any other calls to SAP, except
   * those above. This function may be called multiple times to
   * register multiple callbacks. Runs in NPI task.
   */
  // SAP_setAsyncCB(AP_asyncCB); // TODO

  //Give the NP time to send a PUIND, if we receive one then
  //we can assume the NP has just started and we don't need to send a reset
  //otherwise, we can assume the NP was running previously and needs to be
  //reset to a known state
  if (0 == Event_pend(apEvent, AP_NONE, AP_EVT_PUI, 100000)) {
    // Assuming that at SAP start up that SNP is already running
    SAP_reset();
    Event_pend(apEvent, AP_NONE, AP_EVT_PUI, BIOS_WAIT_FOREVER);
  }

  /* Requires async event handling callback to actually read response. */
  // Gets device MAC address from network processor
  // SAP_setParam(SAP_PARAM_HCI, SNP_HCI_OPCODE_READ_BDADDR, 0, NULL);
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

