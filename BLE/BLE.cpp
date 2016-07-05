
#include <string.h>

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

#include <BLE.h>
#include "BLEServiceList.h"

#define AP_NONE                              Event_Id_NONE   // No Event
#define AP_EVT_PUI                           Event_Id_00     // Power-Up Indication
#define AP_EVT_ADV_ENB                       Event_Id_01     // Advertisement Enable

#define PIN6_7 35

// From bcomdef.h in the BLE SDK
#define B_ADDR_LEN 6

Event_Handle apEvent = NULL;
char ownAddressString[16] = { 0 };

static void AP_asyncCB(uint8_t cmd1, void *pParams);
static void AP_convertBdAddr2Str(char *str, uint8_t *pAddr);
static void constructService(SAP_Service_t *service, BLE_Service *bleService);
static void constructChar(SAP_Char_t *sapChar, BLE_Char *bleChar);
static uint8_t serviceReadAttrCB(void *context,
                                 uint16_t connectionHandle,
                                 uint16_t charHdl, uint16_t offset,
                                 uint16_t size, uint16_t * len,
                                 uint8_t *pData);
static uint8_t serviceWriteAttrCB(void *context,
                                  uint16_t connectionHandle,
                                  uint16_t charHdl, uint16_t len,
                                  uint8_t *pData);

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

  // Gets device MAC address from network processor
  SAP_setParam(SAP_PARAM_HCI, SNP_HCI_OPCODE_READ_BDADDR, 0, NULL);
  return BLE_SUCCESS;
}

int BLE::end(void)
{
  return BLE_SUCCESS;
}

int BLE::useProfile(BLE_Profile *profile)
{
  return BLE_SUCCESS;
}

int BLE::addService(BLE_Service *bleService)
{
  SAP_Service_t *service = (SAP_Service_t *) malloc(sizeof(SAP_Service_t));
  constructService(service, bleService);
  int status = SAP_registerService(service);
  if (status != SNP_FAILURE && service->serviceHandle != NULL) {
    bleService->handle = service->serviceHandle;
    uint8_t i;
    for (i = 0; i < bleService->numChars; i++)
    {
      bleService->chars[i]->handle = service->charAttrHandles[i].valueHandle;
    }
    addServiceNode(bleService);
  }
  return status;
}

static void constructService(SAP_Service_t *service, BLE_Service *bleService)
{
  bleService->handle = NULL;
  service->serviceUUID.len    = bleService->UUIDlen;
  service->serviceUUID.pUUID  = bleService->UUID;
  service->serviceType        = SNP_PRIMARY_SERVICE;
  service->charTableLen       = bleService->numChars; // sizeof with static array?
  service->charTable          = (SAP_Char_t *) malloc(service->charTableLen *
                                                    sizeof(SAP_Char_t));
  service->context            = NULL;
  service->charReadCallback   = serviceReadAttrCB;
  service->charWriteCallback  = serviceWriteAttrCB;
  service->cccdIndCallback    = NULL; // TO DO
  service->charAttrHandles    = (SAP_CharHandle_t *) malloc(service->charTableLen *
                                                            sizeof(SAP_CharHandle_t));
  uint8_t i;
  for (i = 0; i < bleService->numChars; i++)
  {
    constructChar(&service->charTable[i], bleService->chars[i]);
  }
}

static void constructChar(SAP_Char_t *sapChar, BLE_Char *bleChar)
{
  bleChar->valueFormat = 0; // TO DO REMOVE THIS
  bleChar->_value = NULL;
  bleChar->_valueLen = 0;
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
    uint16_t charStrLen = strlen(bleChar->charDesc);
    sapChar->pUserDesc->maxLen   = charStrLen;
    sapChar->pUserDesc->initLen  = charStrLen;
    // sapChar->pUserDesc->pDesc    = (uint8_t *) malloc(charStrLen*sizeof(uint8_t));
    // memcpy(sapChar->pUserDesc->pDesc, bleChar->charDesc, charStrLen);
    sapChar->pUserDesc->pDesc    = (uint8_t *) bleChar->charDesc;
  }
  else
  {
    sapChar->pUserDesc = NULL;
  }
  sapChar->pCccd = (SAP_UserCCCDAttr_t *) malloc(sizeof(SAP_UserCCCDAttr_t));
  sapChar->pCccd->perms          = SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE;
  if (bleChar->valueFormat)
  {
    sapChar->pFormat = (SAP_FormatAttr_t *) malloc(sizeof(SAP_FormatAttr_t));
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
  if (nonConnAdvertData == NULL)
  {
    nonConnAdvertData = defAdvertData;
    setAdvertData(BLE_ADV_DATA_NOTCONN, sizeof(defAdvertData), defAdvertData);
  }
  if (scanRspData == NULL)
  {
    scanRspData = defScanRspData;
    setAdvertData(BLE_ADV_DATA_SCANRSP, sizeof(defScanRspData), defScanRspData);
  }
  uint8_t enableAdv = SAP_ADV_STATE_ENABLE;
  uint8_t status = SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &enableAdv);
  Event_pend(apEvent, AP_NONE, AP_EVT_ADV_ENB, BIOS_WAIT_FOREVER);
  return status;
}

int BLE::startAdvert(BLE_Advert_Settings advertSettings)
{
  return BLE_SUCCESS;
}

int BLE::stopAdvert(void)
{
  return BLE_SUCCESS;
}

int BLE::setAdvertData(int advertType, uint8_t len, uint8_t *advertData)
{
  uint8_t status = SAP_setParam(SAP_PARAM_ADV, advertType, len, advertData);
  if (status == SNP_SUCCESS)
  {
    if (advertType == BLE_ADV_DATA_NOTCONN)
    {
      nonConnAdvertData = advertData;
    }
    else if (advertType == BLE_ADV_DATA_SCANRSP)
    {
      scanRspData = advertData;
    }
  }
  return status;
}

int BLE::setAdvertName(int advertStringLen, char *advertString)
{
  // if (scanRspData == NULL)
  // {
  //   scanRspData = &defScanRspData;
  // }


  // // length + data  +  length + data
  // uint8_t totalSize = (1 + 2) + (1 + advertStringLen);
  // memcpy((void *) &advertData[4], (void *) advertString, advertStringLen);
  // return setAdvertData(BLE_ADV_DATA_SCANRSP, totalSize, scanRspData);
  return BLE_SUCCESS;
}

int BLE::setAdvertName(char *advertString)
{
  return setAdvertName(strlen(advertString), advertString);
}

int BLE::setAdvertName(String *advertString)
{
  int len = (*advertString).length();
  char *buf = (char *) malloc(len);
  (*advertString).toCharArray(buf, len);
  return setAdvertName(len, buf);
}

int BLE::setConnParams(BLE_Conn_Params *connParams)
{
  return BLE_SUCCESS;
}

int BLE::setGapParam(int paramId, int Value)
{
  return BLE_SUCCESS;
}

int BLE::setMinConnInt(int minConnInt)
{
  return BLE_SUCCESS;
}

int BLE::setMaxConnInt(int maxConnInt)
{
  return BLE_SUCCESS;
}

int BLE::setRespLatency(int respLatency)
{
  return BLE_SUCCESS;
}

int BLE::setBleTimeout(int timeout)
{
  return BLE_SUCCESS;
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
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, unsigned char value)
{
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, int value)
{
  BLE_Char *bleChar = getChar(handle);
  if (bleChar == NULL)
  {
    return BLE_INVALID_HANDLE;
  }
  if (bleChar->_value == NULL)
  {
    bleChar->_value = (void *) malloc(sizeof(int));
    bleChar->_valueLen = sizeof(int);
  }
  *(int *) bleChar->_value = value;
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, unsigned int value)
{
  return BLE_SUCCESS;
}


int BLE::writeValue(int handle, long value)
{
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, unsigned long value)
{
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, float value)
{
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, double value)
{
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, char *str)
{
  return BLE_SUCCESS;
}

int BLE::writeValue(int handle, String str)
{
  return BLE_SUCCESS;
}

boolean BLE::readValue_boolean(int handle)
{
  return BLE_SUCCESS;
}

char BLE::readValue_char(int handle)
{
  return BLE_SUCCESS;
}

unsigned char BLE::readValue_uchar(int handle)
{
  return BLE_SUCCESS;
}

byte BLE::readValue_byte(int handle)
{
  return BLE_SUCCESS;
}

int BLE::readValue_int(int handle)
{
  return BLE_SUCCESS;
}

unsigned int BLE::readValue_uint(int handle)
{
  return BLE_SUCCESS;
}

word BLE::readValue_word(int handle)
{
  return BLE_SUCCESS;
}

long BLE::readValue_long(int handle)
{
  return BLE_SUCCESS;
}

unsigned long BLE::readValue_ulong(int handle)
{
  return BLE_SUCCESS;
}

float BLE::readValue_float(int handle)
{
  return BLE_SUCCESS;
}

double BLE::readValue_double(int handle)
{
  return BLE_SUCCESS;
}

char* BLE::readValue_string(int handle)
{
  return BLE_SUCCESS;
}

String BLE::readValue_String(int handle)
{
  return String();
}

int BLE::serial(void)
{
  return BLE_SUCCESS;
}

int BLE::iBeacon(void)
{
  return BLE_SUCCESS;
}

int BLE::nordicBeacon(void)
{
  return BLE_SUCCESS;
}

int BLE::uriBeacon(void)
{
  return BLE_SUCCESS;
}

int BLE::eddystone(void)
{
  return BLE_SUCCESS;
}

int BLE::available(void)
{
  return BLE_SUCCESS;
}

int BLE::read(void)
{
  return BLE_SUCCESS;
}

int BLE::peek(void)
{
  return BLE_SUCCESS;
}

void BLE::flush(void)
{
  return;
}

size_t BLE::write(uint8_t c)
{
  return BLE_SUCCESS;
}

static void AP_asyncCB(uint8_t cmd1, void *pParams) {
  switch (SNP_GET_OPCODE_HDR_CMD1(cmd1)) {
    case SNP_DEVICE_GRP: {
      switch (cmd1) {
        case SNP_POWER_UP_IND:
          // Notify state machine of Power Up Indication
          // Log_info0("Got PowerUp indication from NP");
          Event_post(apEvent, AP_EVT_PUI);
          break;
        case SNP_HCI_CMD_RSP: {
          snpHciCmdRsp_t *hciRsp = (snpHciCmdRsp_t *) pParams;
          switch (hciRsp->opcode) {
            case SNP_HCI_OPCODE_READ_BDADDR:
              // Update NWP Addr String
              AP_convertBdAddr2Str(ownAddressString, hciRsp->pData);
              // Log_info1("Got own address: 0x%s", (uintptr_t)ownAddressString);
              break;
            default:
              break;
          }
        }
        //   break;
        case SNP_EVENT_IND:
          // Log_info0("Got Event indication from NP");
          // Notify state machine of Advertisement Enabled
          Event_post(apEvent, AP_EVT_ADV_ENB);
          break;
        default:
          break;
      }
    }
      break;
    default:
      break;
  }
}

static void AP_convertBdAddr2Str(char *str, uint8_t *pAddr) {
  uint8_t charCnt;
  char hex[] = "0123456789ABCDEF";

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--) {
    *str++ = hex[*--pAddr >> 4];
    *str++ = hex[*pAddr & 0x0F];
  }
  return;
}

static uint8_t serviceReadAttrCB(void *context,
                                 uint16_t connectionHandle,
                                 uint16_t charHdl, uint16_t offset,
                                 uint16_t size, uint16_t * len,
                                 uint8_t *pData)
{
  BLE_Char *bleChar = getChar(charHdl);
  if (bleChar == NULL)
  {
    *len = 0;
    return SNP_UNKNOWN_ATTRIBUTE;
  }
  *len = bleChar->_valueLen;
  memcpy(pData, (uint8_t *) bleChar->_value, *len);
  return SNP_SUCCESS;
}

static uint8_t serviceWriteAttrCB(void *context,
                                  uint16_t connectionHandle,
                                  uint16_t charHdl, uint16_t len,
                                  uint8_t *pData)
{
  BLE_Char *bleChar = getChar(charHdl);
  if (bleChar == NULL)
  {
    return SNP_UNKNOWN_ATTRIBUTE;
  }
  if (len != bleChar->_valueLen)
  {
    return SNP_INVALID_PARAMS;
  }
  memcpy((uint8_t *) bleChar->_value, pData, len);
  return SNP_SUCCESS;
}
