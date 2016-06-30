
#include <string.h>

#include <BLE.h>

#include <ti/drivers/gpio.h>
#include <ti/drivers/gpio/GPIOMSP432.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

#include <sap.h>
#include <snp.h>
/*
 * This is strictly to force the build system to compile npi into
 * its own object files for the linker to link. It isn't used here.
 */
#include <npi_task.h>

#define AP_NONE                              Event_Id_NONE   // No Event
#define AP_EVT_PUI                           Event_Id_00     // Power-Up Indication

#define PIN6_7 35

Event_Handle apEvent;

void AP_asyncCB(uint8_t cmd1, void *pParams);
static void constructService(SAP_Service_t *service, BLE_Service *bleService);
static void constructChar(SAP_Char_t *sapChar, BLE_Char *bleChar);

BLE::BLE(byte portType)
{
  _portType = portType;
}

int BLE::begin(void)
{
  /*
   * When a MSP432 and a CC2650 are stacked, pin 6.7 of the MSP is
   * connected to the reset pin of the CC2650. By default Energia
   * leaves pins unconfigured, so we manually set it to output high.
   * The CC2650's reset pin is active low.
   */
  pinMode(PIN6_7, OUTPUT);
  digitalWrite(PIN6_7, HIGH);

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
  sapParams.port.remote.boardID = 1;
  SAP_open(&sapParams);

  /*
   * Register callback to receive asynchronous requests from the NP.
   * This must be called before using any other calls to SAP, except
   * those above. This function may be called multiple times to
   * register multiple callbacks. Runs in NPI task.
   */
  SAP_setAsyncCB(AP_asyncCB);

  //Give the NP time to send a PUIND, if we receive one then
  //we can assume the NP has just started and we don't need to send a reset
  //otherwise, we can assume the NP was running previously and needs to be
  //reset to a known state
  if (0 == Event_pend(apEvent, AP_NONE, AP_EVT_PUI, 1000)) {
    // Assuming that at SAP start up that SNP is already running
    SAP_reset();
    Event_pend(apEvent, AP_NONE, AP_EVT_PUI, BIOS_WAIT_FOREVER);
  }

  /* Requires async event handling callback to actually read response. */
  // Gets device MAC address from network processor
  // SAP_setParam(SAP_PARAM_HCI, SNP_HCI_OPCODE_READ_BDADDR, 0, NULL);
  return 0;
}

int BLE::end(void)
{
  return 0;
}

int BLE::useProfile(BLE_Profile *profile)
{
  return 0;
}

int BLE::addService(BLE_Service *bleService)
{
  SAP_Service_t *service = (SAP_Service_t *) malloc(sizeof(SAP_Service_t));
  constructService(service, bleService);
  int status = SAP_registerService(service);
  if (status == SNP_FAILURE || service->serviceHandle == NULL) {
    return status;
  }
  return service->serviceHandle;
}

static void constructService(SAP_Service_t *service, BLE_Service *bleService)
{
  service->serviceUUID.len    = bleService->UUIDlen;
  service->serviceUUID.pUUID  = bleService->UUID;
  service->serviceType        = SNP_PRIMARY_SERVICE;
  service->charTableLen       = bleService->numChars; // sizeof with static array?
  service->charTable          = (SAP_Char_t *) malloc(service->charTableLen *
                                                    sizeof(SAP_Char_t));
  service->context            = NULL;
  service->charReadCallback   = NULL; // TO DO
  service->charWriteCallback  = NULL; // TO DO
  service->cccdIndCallback    = NULL; // TO DO
  service->charAttrHandles    = (SAP_CharHandle_t *) malloc(service->charTableLen *
                                                            sizeof(SAP_CharHandle_t));
  uint8_t i;
  for (i = 0; i < bleService->numChars; i++)
  {
    constructChar(&service->charTable[i], &bleService->chars[i]);
  }
}

static void constructChar(SAP_Char_t *sapChar, BLE_Char *bleChar)
{
  sapChar->UUID.len    = bleChar->UUIDlen;
  sapChar->UUID.pUUID  = bleChar->UUID;
  sapChar->properties  = bleChar->properties;
  sapChar->permissions = (sapChar->properties & BLE_READABLE)
                        ? SNP_GATT_PERMIT_READ : 0
                   || (sapChar->properties & (BLE_WRITABLE_NORSP || BLE_WRITABLE))
                        ? SNP_GATT_PERMIT_WRITE : 0;
  if (bleChar->charDesc)
  {
    sapChar->pUserDesc = (SAP_UserDescAttr_t *) malloc(sizeof(SAP_UserDescAttr_t));
    sapChar->pUserDesc->perms    = SNP_GATT_PERMIT_READ;
    sapChar->pUserDesc->maxLen   = strlen(bleChar->charDesc);
    sapChar->pUserDesc->initLen  = sapChar->pUserDesc->maxLen;
    sapChar->pUserDesc->pDesc    = (uint8_t *) bleChar->charDesc;
  }
  else
  {
    sapChar->pUserDesc = NULL;
  }
  sapChar->pCccd       = (SAP_UserCCCDAttr_t *) malloc(sizeof(SAP_UserCCCDAttr_t));
  sapChar->pCccd->perms          = SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE;
  if (bleChar->valueFormat)
  {
    sapChar->pFormat   = (SAP_FormatAttr_t *) malloc(sizeof(SAP_FormatAttr_t));
    sapChar->pFormat->format     = bleChar->valueFormat;
    sapChar->pFormat->exponent   = bleChar->valueExponent;
    sapChar->pFormat->unit       = NULL;
    sapChar->pFormat->name_space = NULL;
    sapChar->pFormat->desc       = NULL;
  }
  else
  {
    sapChar->pFormat   = NULL;
  }
  sapChar->pShortUUID  = NULL;
  sapChar->pLongUUID   = NULL;
}

int BLE::startAdvert(void)
{
  return 0;
}

int BLE::startAdvert(BLE_Advert_Settings advertSettings)
{
  return 0;
}

int BLE::stopAdvert(void)
{
  return 0;
}

int BLE::setAdvertData(byte advertType, uint8_t *advertData)
{
  return 0;
}

int BLE::setAdvertData(byte advertType, char *advertData)
{
  return 0;
}

int BLE::setAdvertData(byte advertType, String *advertData)
{
  return 0;
}

int BLE::setConnParams(BLE_Conn_Params *connParams)
{
  return 0;
}

int BLE::setGapParam(int paramId, int Value)
{
  return 0;
}

int BLE::setMinConnInt(int minConnInt)
{
  return 0;
}

int BLE::setMaxConnInt(int maxConnInt)
{
  return 0;
}

int BLE::setRespLatency(int respLatency)
{
  return 0;
}

int BLE::setBleTimeout(int timeout)
{
  return 0;
}

void BLE::terminateConn(void)
{
  return;
}

void BLE::terminateConn(byte abruptly)
{
  return;
}

int BLE::writeValue(int handle, char value)
{
  return 0;
}

int BLE::writeValue(int handle, unsigned char)
{
  return 0;
}

int BLE::writeValue(int handle, int value)
{
  return 0;
}

int BLE::writeValue(int handle, unsigned int)
{
  return 0;
}


int BLE::writeValue(int handle, long value)
{
  return 0;
}

int BLE::writeValue(int handle, unsigned long)
{
  return 0;
}

int BLE::writeValue(int handle, float value)
{
  return 0;
}

int BLE::writeValue(int handle, double value)
{
  return 0;
}

int BLE::writeValue(int handle, char *str)
{
  return 0;
}

int BLE::writeValue(int handle, String str)
{
  return 0;
}

boolean BLE::readValue_boolean(int handle)
{
  return 0;
}

char BLE::readValue_char(int handle)
{
  return 0;
}

unsigned char BLE::readValue_uchar(int handle)
{
  return 0;
}

byte BLE::readValue_byte(int handle)
{
  return 0;
}

int BLE::readValue_int(int handle)
{
  return 0;
}

unsigned int BLE::readValue_uint(int handle)
{
  return 0;
}

word BLE::readValue_word(int handle)
{
  return 0;
}

long BLE::readValue_long(int handle)
{
  return 0;
}

unsigned long BLE::readValue_ulong(int handle)
{
  return 0;
}

float BLE::readValue_float(int handle)
{
  return 0;
}

double BLE::readValue_double(int handle)
{
  return 0;
}

char* BLE::readValue_string(int handle)
{
  return 0;
}

String BLE::readValue_String(int handle)
{
  return String();
}


int BLE::serial(void)
{
  return 0;
}

int BLE::iBeacon(void)
{
  return 0;
}

int BLE::nordicBeacon(void)
{
  return 0;
}

int BLE::uriBeacon(void)
{
  return 0;
}

int BLE::eddystone(void)
{
  return 0;
}

int BLE::available(void)
{
  return 0;
}

int BLE::read(void)
{
  return 0;
}

int BLE::peek(void)
{
  return 0;
}

void BLE::flush(void)
{
  return;
}

size_t BLE::write(uint8_t c)
{
  return 0;
}

void AP_asyncCB(uint8_t cmd1, void *pParams) {
  switch (SNP_GET_OPCODE_HDR_CMD1(cmd1)) {
    case SNP_DEVICE_GRP: {
      switch (cmd1) {
        case SNP_POWER_UP_IND:
          // Notify state machine of Power Up Indication
          // Log_info0("Got PowerUp indication from NP");
          Event_post(apEvent, AP_EVT_PUI);
          break;
        // case SNP_HCI_CMD_RSP: {
        //   snpHciCmdRsp_t *hciRsp = (snpHciCmdRsp_t *) pParams;
        //   switch (hciRsp->opcode) {
        //     case SNP_HCI_OPCODE_READ_BDADDR:
        //       // Update NWP Addr String
        //       AP_convertBdAddr2Str(ownAddressString, hciRsp->pData);
        //       // Log_info1("Got own address: 0x%s", (uintptr_t)ownAddressString);
        //       break;
        //     default:
        //       break;
        //   }
        // }
        //   break;
        // case SNP_EVENT_IND:
          // Log_info0("Got Event indication from NP");
          // Notify state machine of Advertisement Enabled
          // Event_post(apEvent, AP_EVT_ADV_ENB);
          // break;
        default:
          break;
      }
    }
      break;
    default:
      break;
  }
}