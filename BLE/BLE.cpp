
#include <string.h>

#include <ti/drivers/gpio.h>
#include <ti/drivers/gpio/GPIOMSP432.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

#include <sap.h>
#include <snp.h>
#include <snp_rpc.h>
#include <hal_defs.h>
/*
 * This is strictly to force the build system to compile npi into
 * its own object files for the linker to link. It isn't used here.
 */
#include <npi_task.h>

#include <BLE.h>
#include "BLEServiceList.h"
#include "Flags.h"

#define AP_NONE                              Event_Id_NONE   // No Event
#define AP_EVT_PUI                           Event_Id_00     // Power-Up Indication
#define AP_EVT_ADV_ENB                       Event_Id_01     // Advertisement Enable
#define AP_EVT_ADV_END                       Event_Id_02     // Advertisement Ended
#define AP_EVT_CONN_EST                      Event_Id_03     // Connection Established Event
#define AP_EVT_CONN_TERM                     Event_Id_04     // Connection Terminated Event
#define AP_ERROR                             Event_Id_31     // Error

#define PIN6_7 35

// From bcomdef.h in the BLE SDK
#define B_ADDR_LEN 6

Event_Handle apEvent = NULL;
uint16_t connHandle = 0;
bool serialEnabled = false;

int flag0 = 0;
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
int flag4 = 0;
int flag5 = 0;

static void AP_asyncCB(uint8_t cmd1, void *pParams);
static uint8_t getUUIDLen(uint8_t *UUID);
static void constructService(SAP_Service_t *service, BLE_Service *bleService);
static void constructChar(SAP_Char_t *sapChar, BLE_Char *bleChar);
static void writeNotifInd(BLE_Char *bleChar);
static uint8_t charValueInit(BLE_Char *bleChar, size_t size);
static uint8_t readValueValidateSize(BLE_Char *bleChar, size_t size);
static uint8_t serviceReadAttrCB(void *context,
                                 uint16_t connectionHandle,
                                 uint16_t charHdl, uint16_t offset,
                                 uint16_t maxSize, uint16_t *len,
                                 uint8_t *pData);
static uint8_t serviceWriteAttrCB(void *context,
                                  uint16_t connectionHandle,
                                  uint16_t charHdl, uint16_t len,
                                  uint8_t *pData);
static uint8_t serviceCCCDIndCB(void *context,
                                uint16_t connectionHandle,
                                uint16_t cccdHdl, uint8_t type,
                                uint16_t value);
static void processSNPEventCB(uint16_t event, snpEventParam_t *param);

BLE::BLE(byte portType)
{
  _portType = portType;
  error = BLE_SUCCESS;
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

  /*
   * Register async SNP event handler. Includes connection establisment,
   * termination, advertisement enabling, security events.
   */
  SAP_registerEventCB(processSNPEventCB, 0xFFFF);

  //Give the NP time to send a PUIND, if we receive one then
  //we can assume the NP has just started and we don't need to send a reset
  //otherwise, we can assume the NP was running previously and needs to be
  //reset to a known state
  if (0 == Event_pend(apEvent, AP_NONE, AP_EVT_PUI, 1000)) {
    // Assuming that at SAP start up that SNP is already running
    SAP_reset();
    Event_pend(apEvent, AP_NONE, AP_EVT_PUI, BIOS_WAIT_FOREVER);
  }

  return BLE_SUCCESS;
}

int BLE::end(void)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::addService(BLE_Service *bleService)
{
  SAP_Service_t *service = (SAP_Service_t *) malloc(sizeof(SAP_Service_t));
  constructService(service, bleService);
  int status = SAP_registerService(service);
  if (status != SNP_FAILURE && service->serviceHandle != 0) {
    bleService->handle = service->serviceHandle;
    uint8_t i;
    for (i = 0; i < bleService->numChars; i++)
    {
      bleService->chars[i]->handle = service->charAttrHandles[i].valueHandle;
      bleService->chars[i]->_CCCDHandle = service->charAttrHandles[i].cccdHandle;
      if (service->charTable[i].pUserDesc)
      {
        free(service->charTable[i].pUserDesc);
      }
      if (service->charTable[i].pCccd)
      {
        free(service->charTable[i].pCccd);
      }
      if (service->charTable[i].pFormat)
      {
        free(service->charTable[i].pFormat);
      }
    }
    addServiceNode(bleService);
  }
  free(service->charTable);
  free(service->charAttrHandles);
  free(service);
  return status;
}

static uint8_t getUUIDLen(uint8_t *UUID)
{
  uint8_t i;
  for (i = SNP_16BIT_UUID_SIZE; i < SNP_128BIT_UUID_SIZE; i++)
  {
    if (UUID[i] != 0)
    {
      return SNP_128BIT_UUID_SIZE;
    }
  }
  return SNP_16BIT_UUID_SIZE;
}

static void constructService(SAP_Service_t *service, BLE_Service *bleService)
{
  bleService->handle = 0;
  service->serviceUUID.len    = getUUIDLen(bleService->UUID);
  service->serviceUUID.pUUID  = bleService->UUID;
  service->serviceType        = SNP_PRIMARY_SERVICE;
  service->charTableLen       = bleService->numChars; // sizeof with static array?
  service->charTable          = (SAP_Char_t *) malloc(service->charTableLen *
                                                    sizeof(SAP_Char_t));
  service->context            = NULL;
  service->charReadCallback   = serviceReadAttrCB;
  service->charWriteCallback  = serviceWriteAttrCB;
  service->cccdIndCallback    = serviceCCCDIndCB;
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
  bleChar->_CCCD = 0;
  bleChar->_CCCDHandle = 0;
  bleChar->_resizable = false;
  sapChar->UUID.len    = getUUIDLen(bleChar->UUID);
  sapChar->UUID.pUUID  = bleChar->UUID;
  sapChar->properties  = bleChar->properties;
  sapChar->permissions = ((sapChar->properties & BLE_READABLE)
                            ? SNP_GATT_PERMIT_READ : 0)
                       | ((sapChar->properties & (BLE_WRITABLE_NORSP | BLE_WRITABLE))
                            ? SNP_GATT_PERMIT_WRITE : 0);
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
    sapChar->pFormat->unit       = 0;
    sapChar->pFormat->name_space = 0;
    sapChar->pFormat->desc       = 0;
  }
  else
  {
    sapChar->pFormat   = NULL;
  }
  sapChar->pShortUUID  = NULL;
  sapChar->pLongUUID   = NULL;
}

void BLE::advertDataInit(void)
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
}

int BLE::startAdvert(void)
{
  return startAdvert((BLE_Advert_Settings *) NULL);
}

int BLE::startAdvert(BLE_Advert_Settings *advertSettings)
{
  advertDataInit();

  uint8_t status;
  if (advertSettings == NULL)
  {
    uint8_t enableAdv = SAP_ADV_STATE_ENABLE;
    status = SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &enableAdv);
  }
  else
  {
    snpStartAdvReq_t lReq;
    lReq.type = advertSettings->advertMode;
    lReq.timeout = advertSettings->timeout;
    lReq.interval = advertSettings->interval;
    lReq.behavior = advertSettings->connectedBehavior;
    status = SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE,
                          sizeof(snpStartAdvReq_t), (uint8_t *) &lReq);
  }
  Event_pend(apEvent, AP_NONE, AP_EVT_ADV_ENB, BIOS_WAIT_FOREVER);
  return status;
}

int BLE::stopAdvert(void)
{
  uint8_t disableAdv = SAP_ADV_STATE_DISABLE;
  SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &disableAdv);
  Event_pend(apEvent, AP_NONE, AP_EVT_ADV_END, BIOS_WAIT_FOREVER);
  return BLE_SUCCESS;
}

int BLE::setAdvertData(int advertType, uint8_t len, uint8_t *advertData)
{
  uint8_t status = SAP_setParam(SAP_PARAM_ADV, advertType, len, advertData);
  if (status == SNP_SUCCESS)
  {
    if (advertType == BLE_ADV_DATA_NOTCONN)
    {
      if (nonConnAdvertData && nonConnAdvertData != defAdvertData)
      {
        free(nonConnAdvertData);
      }
      nonConnAdvertData = advertData;
    }
    else if (advertType == BLE_ADV_DATA_SCANRSP)
    {
      if (scanRspData && scanRspData != defScanRspData)
      {
        free(scanRspData);
      }
      scanRspData = advertData;
    }
  }
  return status;
}

/*
 * Uses the default scan response data defScanRspData in BLETypes.h. The first
 * byte is one plus the length of the name. The second should be
 * SAP_GAP_ADTYPE_LOCAL_NAME_COMPLETE, and the third and so on are the
 * characters of the name.
 */
int BLE::setAdvertName(int advertStringLen, const char *advertString)
{
  uint8_t newSize = sizeof(defScanRspData) - defScanRspData[0]
                  + 1 + advertStringLen;
  uint8_t *newData = (uint8_t *) malloc(newSize);
  newData[0] = 1 + advertStringLen;
  newData[1] = defScanRspData[1];
  strcpy((char *) &newData[2], advertString);
  uint8_t *destAfterStr = newData + 2 + advertStringLen;
  uint8_t *srcAfterStr = defScanRspData + 1 + defScanRspData[0];
  uint8_t afterStrLen = sizeof(defScanRspData) - 1 - defScanRspData[0];
  memcpy(destAfterStr, srcAfterStr, afterStrLen);
  return setAdvertData(BLE_ADV_DATA_SCANRSP, newSize, newData);
}

int BLE::setAdvertName(const char *advertString)
{
  return setAdvertName(strlen(advertString), advertString);
}

int BLE::setAdvertName(String *advertString)
{
  int len = (*advertString).length();
  char *buf = (char *) malloc((len+1)*sizeof(char));
  (*advertString).toCharArray(buf, len);
  int status = setAdvertName(len, buf);
  free(buf);
  return status;
}

int BLE::setConnParams(BLE_Conn_Params *connParams)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::setGapParam(int paramId, int Value)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::setMinConnInt(int minConnInt)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::setMaxConnInt(int maxConnInt)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::setRespLatency(int respLatency)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::setBleTimeout(int timeout)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::terminateConn(void)
{
  int status = SAP_setParam(SAP_PARAM_CONN, SAP_CONN_STATE,
                            sizeof(connHandle), (uint8_t *) &connHandle);
  Event_pend(apEvent, AP_NONE, AP_EVT_CONN_TERM, BIOS_WAIT_FOREVER);
  return status;
}

static void writeNotifInd(BLE_Char *bleChar)
{
  if (bleChar->_CCCD)
  {
    snpNotifIndReq_t localReq;
    localReq.connHandle = connHandle;
    localReq.attrHandle = bleChar->handle;
    localReq.authenticate = 0;
    localReq.pData = (uint8_t *) bleChar->_value;
    if (bleChar->_CCCD & SNP_GATT_CLIENT_CFG_NOTIFY)
    {
      localReq.type = SNP_SEND_NOTIFICATION;
    }
    else if (bleChar->_CCCD & SNP_GATT_CLIENT_CFG_INDICATE)
    {
      localReq.type = SNP_SEND_INDICATION;
    }
    SNP_RPC_sendNotifInd(&localReq, bleChar->_valueLen);
  }
}

static uint8_t charValueInit(BLE_Char *bleChar, size_t size)
{
  if (bleChar->_valueLen != size)
  {
    if (bleChar->_resizable && bleChar->_value)
    {
      free(bleChar->_value);
      bleChar->_value == NULL;
    }
    else
    {
      return BLE_INVALID_PARAMETERS;
    }
  }
  if (bleChar->_value == NULL)
  {
    bleChar->_value = (void *) malloc(size);
    bleChar->_valueLen = size;
  }
  return BLE_SUCCESS;
}

int BLE::writeValue(BLE_Char *bleChar, char value)
{
  int status = charValueInit(bleChar, sizeof(char));
  if (status == BLE_SUCCESS)
  {
    *(char *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, unsigned char value)
{
  int status = charValueInit(bleChar, sizeof(unsigned char));
  if (status == BLE_SUCCESS)
  {
    *(unsigned char *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, int value)
{
  int status = charValueInit(bleChar, sizeof(int));
  if (status == BLE_SUCCESS)
  {
    *(int *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, unsigned int value)
{
  int status = charValueInit(bleChar, sizeof(unsigned int));
  if (status == BLE_SUCCESS)
  {
    *(unsigned int *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, long value)
{
  int status = charValueInit(bleChar, sizeof(long));
  if (status == BLE_SUCCESS)
  {
    *(long *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, unsigned long value)
{
  int status = charValueInit(bleChar, sizeof(unsigned long));
  if (status == BLE_SUCCESS)
  {
    *(unsigned long *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, float value)
{
  int status = charValueInit(bleChar, sizeof(float));
  if (status == BLE_SUCCESS)
  {
    *(float *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, double value)
{
  int status = charValueInit(bleChar, sizeof(double));
  if (status == BLE_SUCCESS)
  {
    *(double *) bleChar->_value = value;
    writeNotifInd(bleChar);
  }
  return status;
}

/*
 * Use buffer of size len+1 so the null-termination is stored. This way the
 * stored strings match the functionality of strcpy, which copies it.
 */
int BLE::writeValue(BLE_Char *bleChar, int len, const char *str)
{
  bleChar->_resizable = true;
  int status = charValueInit(bleChar, (len+1)*sizeof(char));
  if (status == BLE_SUCCESS)
  {
    strcpy((char *) bleChar->_value, str);
    writeNotifInd(bleChar);
  }
  return status;
}

int BLE::writeValue(BLE_Char *bleChar, const char *str)
{
  return writeValue(bleChar, strlen(str), str);
}

int BLE::writeValue(BLE_Char *bleChar, const uint8_t *str)
{
  return writeValue(bleChar, (char *) str);
}

int BLE::writeValue(BLE_Char *bleChar, String str)
{
  int len = str.length();
  char *buf = (char *) malloc((len+1)*sizeof(char));
  str.toCharArray(buf, len+1);
  int status = writeValue(bleChar, len, buf);
  free(buf);
  return status;
}

static uint8_t readValueValidateSize(BLE_Char *bleChar, size_t size)
{
  if (bleChar->_value == NULL || bleChar->_valueLen != size)
  {
    return BLE_UNDEFINED_VALUE;
  }
  return BLE_SUCCESS;
}

boolean BLE::readValue_boolean(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(boolean));
  if (error == BLE_SUCCESS)
  {
    return *(boolean *) bleChar->_value;
  }
  return 0;
}

char BLE::readValue_char(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(char));
  if (error == BLE_SUCCESS)
  {
    return *(char *) bleChar->_value;
  }
  return 0;
}

unsigned char BLE::readValue_uchar(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(unsigned char));
  if (error == BLE_SUCCESS)
  {
    return *(unsigned char *) bleChar->_value;
  }
  return 0;
}

byte BLE::readValue_byte(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(byte));
  if (error == BLE_SUCCESS)
  {
    return *(byte *) bleChar->_value;
  }
  return 0;
}

int BLE::readValue_int(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(int));
  if (error == BLE_SUCCESS)
  {
    return *(int *) bleChar->_value;
  }
  return 0;
}

unsigned int BLE::readValue_uint(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(unsigned int));
  if (error == BLE_SUCCESS)
  {
    return *(unsigned int *) bleChar->_value;
  }
  return 0;
}

word BLE::readValue_word(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(word));
  if (error == BLE_SUCCESS)
  {
    return *(word *) bleChar->_value;
  }
  return 0;
}

long BLE::readValue_long(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(long));
  if (error == BLE_SUCCESS)
  {
    return *(long *) bleChar->_value;
  }
  return 0;
}

unsigned long BLE::readValue_ulong(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(unsigned long));
  if (error == BLE_SUCCESS)
  {
    return *(unsigned long *) bleChar->_value;
  }
  return 0;
}

float BLE::readValue_float(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(float));
  if (error == BLE_SUCCESS)
  {
    return *(float *) bleChar->_value;
  }
  return 0;
}

double BLE::readValue_double(BLE_Char *bleChar)
{
  error = readValueValidateSize(bleChar, sizeof(double));
  if (error == BLE_SUCCESS)
  {
    return *(double *) bleChar->_value;
  }
  return 0;
}

char* BLE::readValue_string(BLE_Char *bleChar)
{
  // readValueHelper but with different length handling
  error = BLE_SUCCESS;
  if (bleChar->_value == NULL)
  {
    error = BLE_UNDEFINED_VALUE;
  }
  if (error == BLE_SUCCESS)
  {
    char *buf = (char *) malloc((bleChar->_valueLen)*sizeof(char));
    strcpy(buf, (char *) bleChar->_value);
    return buf;
  }
  return NULL;
}

// TO-DO: Return pointer instead? Not sure if ok for Energia.
String BLE::readValue_String(BLE_Char *bleChar)
{
  char *buf = readValue_string(bleChar);
  String str;
  if (buf)
  {
    str = String(buf);
    free(buf);
  }
  else
  {
    str = String();
  }
  return str;
}

int BLE::serial(void)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::iBeacon(void)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::nordicBeacon(void)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::uriBeacon(void)
{
  return BLE_NOT_IMPLEMENTED;
}

int BLE::eddystone(void)
{
  return BLE_NOT_IMPLEMENTED;
}

inline bool BLE::isSerialEnabled(void)
{
  return serialEnabled;
}

int BLE::available(void)
{
  if (!isSerialEnabled())
  {
    return BLE_SERIAL_DISABLED;
  }
}

int BLE::read(void)
{
  if (!isSerialEnabled())
  {
    return BLE_SERIAL_DISABLED;
  }
}

int BLE::peek(void)
{
  if (!isSerialEnabled())
  {
    return BLE_SERIAL_DISABLED;
  }
}

void BLE::flush(void)
{
  if (!isSerialEnabled())
  {
    return;
  }
}

size_t BLE::write(uint8_t c)
{
  if (!isSerialEnabled())
  {
    return BLE_SERIAL_DISABLED;
  }
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
            default:
              break;
          }
        } break;
        default:
          break;
      }
    } break;
    default:
      break;
  }
}

static uint8_t serviceReadAttrCB(void *context,
                                 uint16_t connectionHandle,
                                 uint16_t charHdl, uint16_t offset,
                                 uint16_t maxSize, uint16_t *len,
                                 uint8_t *pData)
{
  uint8_t status = SNP_SUCCESS;
  connHandle = connectionHandle;
  BLE_Char *bleChar = getChar(charHdl);
  if (bleChar == NULL)
  {
    *len = 0;
    status = SNP_UNKNOWN_ATTRIBUTE;
  }
  else if (bleChar->_valueLen <= offset)
  {
    *len = 0;
  }
  else
  {
    uint8_t *src = (uint8_t *) bleChar->_value + offset;
    uint16_t remaining = bleChar->_valueLen - offset;
    *len = MIN(remaining, maxSize);
    memcpy(pData, src, *len);
  }
  return status;
}

static uint8_t serviceWriteAttrCB(void *context,
                                  uint16_t connectionHandle,
                                  uint16_t charHdl, uint16_t len,
                                  uint8_t *pData)
{
  connHandle = connectionHandle;
  BLE_Char *bleChar = getChar(charHdl);
  uint8_t status = SNP_SUCCESS;
  if (bleChar == NULL)
  {
    status = SNP_UNKNOWN_ATTRIBUTE;
  }
  status = charValueInit(bleChar, len);
  if (status == SNP_SUCCESS)
  {
    memcpy((uint8_t *) bleChar->_value, pData, len);
  }
  return status;
}

static uint8_t serviceCCCDIndCB(void *context,
                                uint16_t connectionHandle,
                                uint16_t cccdHdl, uint8_t type,
                                uint16_t value)
{
  uint8_t status = SNP_SUCCESS;
  connHandle = connectionHandle;
  bool notify = (value == SNP_GATT_CLIENT_CFG_NOTIFY);
  bool indicate = (value == SNP_GATT_CLIENT_CFG_INDICATE);
  BLE_Char *bleChar = getCCCD(cccdHdl);
  if (bleChar == NULL)
  {
    status = SNP_UNKNOWN_ATTRIBUTE;
  }
  else if (!(value == 0 || notify || indicate))
  {
    status = SNP_INVALID_PARAMS;
  }
  // Attempting to set to mode not allowed by char properties
  else if ((notify && bleChar->properties != BLE_NOTIFIABLE)
        || (indicate && bleChar->properties != BLE_INDICATABLE))
  {
    status = SNP_NOTIF_IND_NOT_ALLOWED;
  }
  else
  {
    bleChar->_CCCD = (byte) value;
  }
  return status;
}

static void processSNPEventCB(uint16_t event, snpEventParam_t *param)
{
  switch (event)
  {
    case SNP_CONN_EST_EVT: {
      Event_post(apEvent, AP_EVT_CONN_EST);
    } break;
    case SNP_CONN_TERM_EVT: {
      Event_post(apEvent, AP_EVT_CONN_TERM);
      resetCCCD();
    } break;
    // case SNP_CONN_PARAM_UPDATED_EVT: {
    // } break;
    case SNP_ADV_STARTED_EVT: {
      snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *) param;
      advEvt->status = SNP_SUCCESS;
      flag1 = advEvt->status;
      if (advEvt->status == SNP_SUCCESS)
      {
        flag0 = 100;
        Event_post(apEvent, AP_EVT_ADV_ENB);
      }
      else
      {
        flag0 = 101;
        Event_post(apEvent, AP_ERROR);
      }
    } break;
    case SNP_ADV_ENDED_EVT: {
      snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *) param;
      if (advEvt->status == SNP_SUCCESS) {
        Event_post(apEvent, AP_EVT_ADV_END);
      }
      else
      {
        Event_post(apEvent, AP_ERROR);
      }
    } break;
    // case SNP_ATT_MTU_EVT: {
    // } break;
    // case SNP_SECURITY_EVT: {
    // } break;
    // case SNP_AUTHENTICATION_EVT: {
    // } break;
    // case SNP_ERROR_EVT: {
    // } break;
  }
}