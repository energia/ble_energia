
#include <string.h>

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
#include "BLESerial.h"
#include "BLEServiceList.h"
#include "BLEServices.h"
#include "Debug.h"

/*
 * Event_pend timeout set in units of ticks. Tick period is microseconds,
 * so this evaluates to 1 second.
 */
#define AP_EVENT_PEND_TIMEOUT                (1000000/Clock_tickPeriod)

// Event ID's are integers
#define AP_NONE                              Event_Id_NONE   // No Event
#define AP_EVT_PUI                           Event_Id_00     // Power-Up Indication
#define AP_EVT_ADV_ENB                       Event_Id_01     // Advertisement Enabled
#define AP_EVT_ADV_END                       Event_Id_02     // Advertisement Ended
#define AP_EVT_ADV_DATA_RSP                  Event_Id_03     // Advertisement Data Set Response
#define AP_EVT_CONN_EST                      Event_Id_04     // Connection Established
#define AP_EVT_CONN_TERM                     Event_Id_05     // Connection Terminated
#define AP_EVT_HCI_RSP                       Event_Id_06     // HCI Command Response
#define AP_EVT_TEST_RSP                      Event_Id_07     // Test Command Response
#define AP_EVT_CONN_PARAMS_UPDATED           Event_Id_08     // Connection Parameters Updated
#define AP_EVT_CONN_PARAMS_CNF               Event_Id_09     // Connection Parameters Request Confirmation
#define AP_EVT_NOTIF_IND_RSP                 Event_Id_10     // Notification/Indication Response
#define AP_EVT_HANDLE_AUTH_EVT               Event_Id_11     // Authentication Required Event
#define AP_EVT_AUTH_RSP                      Event_Id_12     // Set Authentication Data Response
#define AP_EVT_SECURITY_STATE                Event_Id_13     // Security State Changed
#define AP_EVT_SECURITY_PARAM_RSP            Event_Id_14     // Set Security Param Response
#define AP_EVT_WHITE_LIST_RSP                Event_Id_15     // Set White List Policy Response
#define AP_EVT_COPIED_ASYNC_DATA             Event_Id_30     // Copied Data From asyncRspData
#define AP_ERROR                             Event_Id_31     // Error

#define PIN6_7 35
#define BLE_Board_MRDY 2  // Pin 6.0
#define BLE_Board_SRDY 19 // Pin 2.5

// From bcomdef.h in the BLE SDK
#define B_ADDR_LEN 6


/* Macro abuse for setting up many pins for debug */
#define DEBUG_FXN(pin, pinNum) \
int isOutput##pin = 0; \
void ping##pin(void) \
{ \
 if (!isOutput##pin) \
 { \
   pinMode(pinNum, OUTPUT); \
   isOutput##pin = 1; \
 } \
 ping(pinNum); \
}
static void ping(int pin)
{
 digitalWrite(pin, HIGH);
 digitalWrite(pin, LOW);
}
DEBUG_PINS_LIST
#undef DEBUG_FXN
/* End abuse */

Event_Handle apEvent = NULL;
snp_msg_t *asyncRspData = NULL;
snpEventParam_t eventHandlerData;
uint16_t _connHandle = -1;
bool connected;
bool advertising;

BLE ble = BLE();

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t defNotConnAD[] =
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

/*
 * The SNP will automatically use the non-connectable advertisement data
 * if the connectable advertisement data is not set.
 */
static uint8_t defConnAD[0] = {};

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

static uint8_t *defADArr[] =
{
  defNotConnAD,
  defConnAD,
  defScanRspData
};

static size_t defADSizes[] =
{
  sizeof(defNotConnAD),
  sizeof(defConnAD),
  sizeof(defScanRspData)
};

static uint8_t aDIdxToType[] =
{
  BLE_ADV_DATA_NOTCONN,
  BLE_ADV_DATA_CONN,
  BLE_ADV_DATA_SCANRSP
};

static uint8_t advertIndex(uint8_t advertType);
static void AP_asyncCB(uint8_t cmd1, void *pParams);
static uint8_t writeNotifInd(BLE_Char *bleChar);
static uint8_t readValueValidateSize(BLE_Char *bleChar, size_t size);
static void processSNPEventCB(uint16_t event, snpEventParam_t *param);
static bool apEventPend(uint32_t event);
static inline void apPostError(uint8_t status);
static bool isError(uint8_t status);

BLE::BLE(byte portType)
{
  _portType = portType;
  for (uint8_t idx = 0; idx < MAX_ADVERT_IDX; idx++) {advertDataArr[idx] = NULL;}
  displayStringFxn = NULL;
  displayUIntFxn = NULL;
  resetPublicMembers();
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
   * Use this to do something at the application level on a write or
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
  sapParams.port.remote.boardID = 1; // USB Serial is index 0
  sapParams.port.remote.mrdyPinID = BLE_Board_MRDY;
  sapParams.port.remote.srdyPinID = BLE_Board_SRDY;
  if (isError(SAP_open(&sapParams)))
  {
    SAP_close();
    return BLE_CHECK_ERROR;
  }

  /*
   * Register callback to receive asynchronous requests from the NP.
   * This must be called before using any other calls to SAP, except
   * those above. This function may be called multiple times to
   * register multiple callbacks. Runs in NPI task.
   */
  if (isError(SAP_setAsyncCB(AP_asyncCB)))
  {
    SAP_close();
    return BLE_CHECK_ERROR;
  }

  /*
   * Register async SNP event handler. Includes connection establisment,
   * termination, advertisement enabling, security events.
   */
  if (isError(SAP_registerEventCB(processSNPEventCB, 0xFFFF)))
  {
    SAP_close();
    return BLE_CHECK_ERROR;
  }

  /*
   * Give the NP time to send a power up indicator, if we receive one then
   * we can assume the NP has just started and we don't need to send a reset
   * otherwise, we can assume the NP was running previously and needs to be
   * reset to a known state
   */
  if (!apEventPend(AP_EVT_PUI)) {
    // Assuming that at SAP start up that SNP is already running
    if (isError(SAP_reset()))
    {
      SAP_close();
      return BLE_CHECK_ERROR;
    }
    if (!apEventPend(AP_EVT_PUI))
    {
      return BLE_CHECK_ERROR;
    }
  }

  /*
   * Events XORed with full mask are not sent from the SNP to the AP.
   * This SAP API always returns success.
   */
  SAP_setSNPEventMask(SNP_ATT_MTU_EVT);

  return BLE_SUCCESS;
}

int BLE::end(void)
{
  /* Reset private members of BLE.h */
  for (uint8_t idx = 0; idx < MAX_ADVERT_IDX; idx++) {advertDataArr[idx] = NULL;}

  /* Reset public members of BLE.h */
  resetPublicMembers();


  // _connHandle = -1;
  // txChar;
  // rxChar;
  // serialService;
  // bleServiceListHead;
  // bleServiceListTail;
  // rxBuffer;
  // rxWriteIndex;
  // rxReadIndex;
  // SAP_reset();
  // SAP_close();
  return BLE_NOT_IMPLEMENTED;
}

int BLE::terminateConn(void)
{
  if ((!connected && isError(BLE_NOT_CONNECTED)) ||
      isError(SAP_setParam(SAP_PARAM_CONN, SAP_CONN_STATE,
                           sizeof(_connHandle), (uint8_t *) &_connHandle)) ||
      !apEventPend(AP_EVT_CONN_TERM))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

bool BLE::isConnected(void)
{
  return connected;
}

bool BLE::isAdvertising(void)
{
  return advertising;
}

int BLE::resetPublicMembers(void)
{
  error = BLE_SUCCESS;
  opcode = 0;
  memset(&usedConnParams, 0, sizeof(usedConnParams));
  memset(&bleAddr, 0, sizeof(bleAddr));
  authKeySet = false;
  authKey = 0;
  return BLE_SUCCESS;
}

int BLE::addService(BLE_Service *bleService)
{
  if (isError(BLE_registerService(bleService)))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

static uint8_t advertIndex(uint8_t advertType)
{
  switch (advertType)
  {
    case BLE_ADV_DATA_NOTCONN:
      return 0;
    case BLE_ADV_DATA_CONN:
      return 1;
    case BLE_ADV_DATA_SCANRSP:
      return 2;
  }
  return BLE_INVALID_PARAMETERS;
}

uint8_t BLE::advertDataInit(void)
{
  for (uint8_t idx = 0; idx < MAX_ADVERT_IDX; idx++)
  {
    if (advertDataArr[idx] == NULL && defADArr[idx] != NULL && defADSizes[idx])
    {
      if (isError(setAdvertData(aDIdxToType[idx], defADSizes[idx],
                                defADArr[idx])))
      {
        return BLE_CHECK_ERROR;
      }
    }
  }
  return BLE_SUCCESS;
}

int BLE::startAdvert(void)
{
  return startAdvert((BLE_Advert_Settings *) NULL);
}

int BLE::startAdvert(BLE_Advert_Settings *advertSettings)
{
  if ((advertising && isError(BLE_ALREADY_ADVERTISING)) ||
      isError(advertDataInit()))
  {
    return BLE_CHECK_ERROR;
  }

  uint16_t reqSize;
  uint8_t *pData;
  /* Declare these outside the if statements so they're in-scope
     for SAP_setParam. */
  uint8_t enableAdv = SAP_ADV_STATE_ENABLE;
  snpStartAdvReq_t lReq;
  if (advertSettings == NULL)
  {
    reqSize = 1;
    pData = &enableAdv;
  }
  else
  {
    lReq.type = advertSettings->advertMode;
    lReq.timeout = advertSettings->timeout;
    lReq.interval = advertSettings->interval;
    lReq.behavior = advertSettings->connectedBehavior;
    reqSize = (uint16_t) sizeof(lReq);
    pData = (uint8_t *) &lReq;
  }
  if (isError(SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, reqSize, pData)) ||
      !apEventPend(AP_EVT_ADV_ENB))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

int BLE::stopAdvert(void)
{
  uint8_t disableAdv = SAP_ADV_STATE_DISABLE;
  if ((!advertising && isError(BLE_NOT_ADVERTISING)) ||
      isError(SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &disableAdv)) ||
      !apEventPend(AP_EVT_ADV_END))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

int BLE::setAdvertData(uint8_t advertType, uint8_t len, uint8_t *advertData)
{
  if (isError(SAP_setParam(SAP_PARAM_ADV, advertType, len, advertData)) ||
      !apEventPend(AP_EVT_ADV_DATA_RSP))
  {
    return BLE_CHECK_ERROR;
  }
  // advertType validated by SAP_setParam
  uint8_t idx = advertIndex(advertType);
  if (advertDataArr[idx] && advertDataArr[idx] != defADArr[idx])
  {
    free(advertDataArr[idx]);
  }
  advertDataArr[idx] = advertData;
  return BLE_SUCCESS;
}

/*
 * Uses the default scan response data defScanRspData. The first
 * byte is one plus the length of the name. The second should be
 * SAP_GAP_ADTYPE_LOCAL_NAME_COMPLETE, and the third and so on are the
 * characters of the name.
 */
int BLE::setAdvertName(uint8_t advertStringLen, const char *advertString)
{
  uint8_t newSize = sizeof(defScanRspData) - defScanRspData[0]
                  + 1 + advertStringLen;
  uint8_t *newData = (uint8_t *) malloc(newSize * sizeof(*newData));
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
  uint8_t len = (*advertString).length();
  char *buf = (char *) malloc((len+1)*sizeof(*buf));
  (*advertString).toCharArray(buf, len);
  uint8_t status = setAdvertName(len, buf);
  free(buf);
  return status;
}

int BLE::setGattParam(uint8_t serviceId, uint8_t charId,
                      uint16_t len, uint8_t *pData)
{
  return SAP_setServiceParam(serviceId, charId, len, pData);
}

int BLE::getGattParam(uint8_t serviceId, uint8_t charId,
                      uint16_t *len, uint8_t *pData)
{
  return SAP_getServiceParam(serviceId, charId, len, pData);
}

int BLE::setGapParam(uint16_t paramId, uint16_t value)
{
  return SAP_setParam(SAP_PARAM_GAP, paramId,
                      sizeof(value), (uint8_t *) &value);
}

int BLE::getGapParam(uint16_t paramId, uint16_t *value)
{
  return SAP_setParam(SAP_PARAM_GAP, paramId,
                      sizeof(*value), (uint8_t *) value);
}

uint8_t *hciCommand(uint16_t opcode, uint16_t len, uint8_t *pData)
{
  if (isError(SAP_getParam(SAP_PARAM_HCI, opcode, len, pData)) ||
      !apEventPend(AP_EVT_HCI_RSP))
  {
    return NULL;
  }
  snpHciCmdRsp_t hciCmdRsp;
  memcpy(&hciCmdRsp, asyncRspData, sizeof(hciCmdRsp));
  Event_post(apEvent, AP_EVT_COPIED_ASYNC_DATA);
  return hciCmdRsp.pData; // TODO: does this get deallocated in NPI task?
}

/*
 * Must be currently in a connection.
 */
int BLE::setConnParams(BLE_Conn_Params_Update_Req *connParams)
{
  if ((!connected && isError(BLE_NOT_CONNECTED)) ||
      isError(SAP_setParam(SAP_PARAM_CONN, SAP_CONN_PARAM,
                           sizeof(*connParams), (uint8_t *) connParams)) ||
      !apEventPend(AP_EVT_CONN_PARAMS_CNF))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

int BLE::setSingleConnParam(size_t offset, uint16_t value)
{
  BLE_Conn_Params_Update_Req paramsReq;
  paramsReq.connHandle         = _connHandle;
  paramsReq.intervalMin        = SNP_CONN_INT_MIN;
  paramsReq.intervalMax        = SNP_CONN_INT_MAX;
  paramsReq.slaveLatency       = usedConnParams.slaveLatency;
  paramsReq.supervisionTimeout = usedConnParams.supervisionTimeout;
  *(uint16_t *)(((char *) &paramsReq) + offset) = value;
  return setConnParams(&paramsReq);
}

int BLE::setMinConnInt(uint16_t intervalMin)
{
  return setSingleConnParam(offsetof(BLE_Conn_Params_Update_Req, intervalMin),
                            intervalMin);
}

int BLE::setMaxConnInt(uint16_t intervalMax)
{
  return setSingleConnParam(offsetof(BLE_Conn_Params_Update_Req, intervalMax),
                            intervalMax);
}

int BLE::setRespLatency(uint16_t slaveLatency)
{
  return setSingleConnParam(offsetof(BLE_Conn_Params_Update_Req, slaveLatency),
                            slaveLatency);
}

int BLE::setBleTimeout(uint16_t supervisionTimeout)
{
  return setSingleConnParam(offsetof(BLE_Conn_Params_Update_Req, supervisionTimeout),
                            supervisionTimeout);
}

static uint8_t writeNotifInd(BLE_Char *bleChar)
{
  if (bleChar->_CCCD)
  {
    snpNotifIndReq_t localReq;
    localReq.connHandle = _connHandle;
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
    if (isError(SNP_RPC_sendNotifInd(&localReq, bleChar->_valueLen)))
    {
      return BLE_CHECK_ERROR;
    }
    // Only pend for confirmation of indication
    else if ((bleChar->_CCCD & SNP_GATT_CLIENT_CFG_INDICATE) &&
             !apEventPend(AP_EVT_NOTIF_IND_RSP))

    {
      return BLE_CHECK_ERROR;
    }
  }
  return BLE_SUCCESS;
}

int BLE::writeValue(BLE_Char *bleChar, char value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(char *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

int BLE::writeValue(BLE_Char *bleChar, unsigned char value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(unsigned char *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

int BLE::writeValue(BLE_Char *bleChar, int value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(int *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

int BLE::writeValue(BLE_Char *bleChar, unsigned int value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(unsigned int *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

int BLE::writeValue(BLE_Char *bleChar, long value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(long *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

int BLE::writeValue(BLE_Char *bleChar, unsigned long value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(unsigned long *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

int BLE::writeValue(BLE_Char *bleChar, float value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(float *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

int BLE::writeValue(BLE_Char *bleChar, double value)
{
  if (isError(BLE_charValueInit(bleChar, sizeof(value))))
  {
    return BLE_CHECK_ERROR;
  }
  *(double *) bleChar->_value = value;
  return writeNotifInd(bleChar);
}

/*
 * Use buffer of size len+1 so the null-termination is stored. This way the
 * stored strings match the functionality of strcpy, which copies it.
 */
int BLE::writeValue(BLE_Char *bleChar, int len, const char *str)
{
  bleChar->_resizable = true;
  if (isError(BLE_charValueInit(bleChar, (len+1)*sizeof(*str))))
  {
    return BLE_CHECK_ERROR;
  }
  strcpy((char *) bleChar->_value, str);
  return writeNotifInd(bleChar);
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
  char *buf = (char *) malloc((len+1)*sizeof(*buf));
  str.toCharArray(buf, len+1);
  uint8_t status = writeValue(bleChar, len, buf);
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
  error = BLE_SUCCESS;
  if (bleChar->_value == NULL)
  {
    error = BLE_UNDEFINED_VALUE;
  }
  else if (error == BLE_SUCCESS)
  {
    int len = bleChar->_valueLen;
    /* Convert value to null-termiated string, if not already */
    if (((char *) bleChar->_value)[len-1] != '\0')
    {
      bleChar->_value = realloc(bleChar->_value, (len+1)*sizeof(char));
      ((char *) bleChar->_value)[len] = '\0';
    }
    return (char *) bleChar->_value;
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
  }
  else
  {
    str = String();
  }
  return str;
}

int BLE::setSecurityParam(uint16_t paramID, uint16_t len, uint8_t *pData)
{
  if (isError(SAP_setParam(SAP_PARAM_SECURITY, paramID, len, pData)) ||
      !apEventPend(AP_EVT_SECURITY_PARAM_RSP))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

int BLE::setPairingMode(uint8_t param)
{
  return setSecurityParam(SAP_SECURITY_BEHAVIOR, 1, &param);
}

int BLE::setIoCapabilities(uint8_t param)
{
  return setSecurityParam(SAP_SECURITY_IOCAPS, 1, &param);
}

int BLE::useBonding(uint8_t param)
{
  return setSecurityParam(SAP_SECURITY_BONDING, 1, &param);
}

int BLE::eraseAllBonds(void)
{
  return setSecurityParam(SAP_ERASE_ALL_BONDS, 0, NULL);
}

int BLE::replaceLruBond(uint8_t param)
{
  return setSecurityParam(SAP_ERASE_LRU_BOND, 1, &param);
}

int BLE::sendSecurityRequest(void)
{
  return SAP_sendSecurityRequest();
}

int BLE::setWhiteListPolicy(uint8_t policy)
{
  if (isError(SAP_setParam(SAP_PARAM_WHITELIST, 0, 0, &policy)) ||
      !apEventPend(AP_EVT_WHITE_LIST_RSP))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

unsigned int BLE::getRand(void)
{
  return SAP_getRand();
}

/*
 * Rarely used and advanced-use calls so we won't provide a framework for this.
 */
void BLE::getRevision(BLE_Get_Revision_Rsp *getRevisionRsp)
{
  SAP_getRevision(getRevisionRsp);
}

void BLE::getStatus(BLE_Get_Status_Rsp *getStatusRsp)
{
  SAP_getStatus(getStatusRsp);
}

int BLE::testCommand(BLE_Test_Command_Rsp *testRsp)
{
  SAP_testCommand(); // void function
  if (!apEventPend(AP_EVT_TEST_RSP))
  {
    return BLE_CHECK_ERROR;
  }
  memcpy(testRsp, asyncRspData, sizeof(*testRsp));
  Event_post(apEvent, AP_EVT_COPIED_ASYNC_DATA);
  return BLE_SUCCESS;
}

int BLE::serial(void)
{
  if (isError(addService(&serialService)) ||
      isError(writeValue(&txChar, "")) ||
      isError(writeValue(&rxChar, "")))
  {
    return BLE_CHECK_ERROR;
  }
  return BLE_SUCCESS;
}

int BLE::available(void)
{
  return BLESerial_available();
}

int BLE::read(void)
{
  return BLESerial_read();
}

int BLE::peek(void)
{
  return BLESerial_peek();
}

void BLE::flush(void)
{
  return BLESerial_flush();
}

size_t BLE::write(uint8_t c)
{
  if (writeValue(&txChar, c) == BLE_SUCCESS)
  {
    return 1;
  }
  return 0;
}

size_t BLE::write(const uint8_t *buffer, size_t size)
{
  if (writeValue(&txChar, buffer) == BLE_SUCCESS)
  {
    return size;
  }
  return 0;
}

int BLE::handleEvents(void)
{
  uint32_t postedEvent = Event_pend(apEvent, AP_NONE, AP_EVT_HANDLE_AUTH_EVT, 1);
  if (postedEvent & AP_EVT_HANDLE_AUTH_EVT)
  {
    snpAuthenticationEvt_t *evt = (snpAuthenticationEvt_t *) &eventHandlerData;
    if (evt->display || evt->input)
    {
      authKey = getRand() % 1000000;
      authKeySet = true;
      if (evt->display)
      {
        if (displayStringFxn && displayUIntFxn)
        {
          displayStringFxn("Auth key:");
          displayUIntFxn(authKey);
        }
        else if (Serial)
        {
          Serial.print("Auth key:");
          Serial.println(authKey);
        }
      }
    }
    else if (evt->numCmp)
    {
      // TODO
      if (displayStringFxn && displayUIntFxn)
      {
        displayStringFxn("Left btn if eql; right else");
        displayUIntFxn(evt->numCmp);
      }
      else if (Serial)
      {
        Serial.print("Left btn if eql; right else");
        Serial.println(evt->numCmp);
      }
    }
    if (isError(SAP_setAuthenticationRsp(authKey)) ||
        !apEventPend(AP_EVT_AUTH_RSP))
    {
      return BLE_CHECK_ERROR;
    }
  }
  return BLE_SUCCESS;
}

/*
 * Even though many events and resposes are asynchronous, we still handle them
 * synchronously. Any request that generates an asynchronous response should
 * Event_pend on the corresponding Event_post here.
 */
static void AP_asyncCB(uint8_t cmd1, void *pParams)
{
  ble.error = BLE_SUCCESS;
  switch (SNP_GET_OPCODE_HDR_CMD1(cmd1))
  {
    case SNP_DEVICE_GRP:
    {
      switch (cmd1)
      {
        case SNP_POWER_UP_IND:
        {
          // Notify state machine of Power Up Indication
          // Log_info0("Got PowerUp indication from NP");
          Event_post(apEvent, AP_EVT_PUI);
        } break;
        case SNP_HCI_CMD_RSP:
        {
          snpHciCmdRsp_t *hciRsp = (snpHciCmdRsp_t *) pParams;
          ble.opcode = hciRsp->opcode;
          if (hciRsp->status == SNP_SUCCESS)
          {
            asyncRspData = (snp_msg_t *) hciRsp;
            Event_post(apEvent, AP_EVT_HCI_RSP);
            Event_pend(apEvent, AP_NONE, AP_EVT_COPIED_ASYNC_DATA,
                       AP_EVENT_PEND_TIMEOUT);
          }
          else
          {
            apPostError(hciRsp->status);
          }
        } break;
        case SNP_TEST_RSP:
        {
          snpTestCmdRsp_t *testRsp = (snpTestCmdRsp_t *) pParams;
          asyncRspData = (snp_msg_t *) testRsp;
          Event_post(apEvent, AP_EVT_TEST_RSP);
          Event_pend(apEvent, AP_NONE, AP_EVT_COPIED_ASYNC_DATA,
                     AP_EVENT_PEND_TIMEOUT);
          // No status code in response
        } break;
        default:
          break;
      }
    } break;
    case SNP_GAP_GRP:
    {
      switch (cmd1)
      {
        case SNP_SET_ADV_DATA_CNF:
        {
          snpSetAdvDataCnf_t *advDataRsp = (snpSetAdvDataCnf_t *) pParams;
          if (advDataRsp->status == SNP_SUCCESS)
          {
            Event_post(apEvent, AP_EVT_ADV_DATA_RSP);
          }
          else
          {
            apPostError(advDataRsp->status);
          }
        } break;
        // Just a confirmation that the request update was sent.
        case SNP_UPDATE_CONN_PARAM_CNF:
        {
          snpUpdateConnParamCnf_t *connRsp =
            (snpUpdateConnParamCnf_t *) pParams;
          if (connRsp->status == SNP_SUCCESS)
          {
            Event_post(apEvent, AP_EVT_CONN_PARAMS_CNF);
          }
          else
          {
            apPostError(connRsp->status);
          }
        } break;
        case SNP_SET_SECURITY_PARAM_RSP:
        {
          snpSetSecParamRsp_t *securityParamRsp = (snpSetSecParamRsp_t *) pParams;
          if (securityParamRsp->status == SNP_SUCCESS)
          {
            Event_post(apEvent, AP_EVT_SECURITY_PARAM_RSP);
          }
          else
          {
            apPostError(securityParamRsp->status);
          }
        } break;
        case SNP_SEND_AUTHENTICATION_DATA_RSP:
        {
          snpSetAuthDataRsp_t *authRsp = (snpSetAuthDataRsp_t *) pParams;
          if (authRsp->status == SNP_SUCCESS)
          {
            Event_post(apEvent, AP_EVT_AUTH_RSP);
          }
          else
          {
            apPostError(authRsp->status);
          }
        }
        case SNP_SET_WHITE_LIST_POLICY_RSP:
        {
          snpSetWhiteListRsp_t *whiteListRsp = (snpSetWhiteListRsp_t *) pParams;
          if (whiteListRsp->status == SNP_SUCCESS)
          {
            Event_post(apEvent, AP_EVT_WHITE_LIST_RSP);
          }
          else
          {
            apPostError(whiteListRsp->status);
          }
        }
        default:
          break;
      }
    }
    case SNP_GATT_GRP:
    {
      switch (cmd1)
      {
        case SNP_SEND_NOTIF_IND_CNF:
        {
          snpNotifIndCnf_t *notifIndRsp = (snpNotifIndCnf_t *) pParams;
          if (notifIndRsp->status == SNP_SUCCESS)
          {
            Event_post(apEvent, AP_EVT_NOTIF_IND_RSP);
          }
          else
          {
            apPostError(notifIndRsp->status);
          }
        } break;
        default:
          break;
      }
    }
    default:
      break;
  }
}

static void processSNPEventCB(uint16_t event, snpEventParam_t *param)
{
  ble.error = BLE_SUCCESS;
  switch (event)
  {
    case SNP_CONN_EST_EVT:
    {
      snpConnEstEvt_t *evt = (snpConnEstEvt_t *) param;
      _connHandle                           = evt->connHandle;
      ble.usedConnParams.connInterval       = evt->connInterval;
      ble.usedConnParams.slaveLatency       = evt->slaveLatency;
      ble.usedConnParams.supervisionTimeout = evt->supervisionTimeout;
      memcpy(&ble.bleAddr, &(evt->pAddr), sizeof(evt->pAddr));
      connected = true;
      Event_post(apEvent, AP_EVT_CONN_EST);
    } break;
    case SNP_CONN_TERM_EVT:
    {
      connected = false;
      Event_post(apEvent, AP_EVT_CONN_TERM);
      BLE_resetCCCD();
    } break;
    case SNP_CONN_PARAM_UPDATED_EVT:
    {
      snpUpdateConnParamEvt_t *evt = (snpUpdateConnParamEvt_t *) param;
      ble.usedConnParams.connInterval       = evt->connInterval;
      ble.usedConnParams.slaveLatency       = evt->slaveLatency;
      ble.usedConnParams.supervisionTimeout = evt->supervisionTimeout;
      Event_post(apEvent, AP_EVT_CONN_PARAMS_UPDATED);
    } break;
    case SNP_ADV_STARTED_EVT:
    {
      snpAdvStatusEvt_t *evt = (snpAdvStatusEvt_t *) param;
      if (evt->status == SNP_SUCCESS)
      {
        advertising = true;
        Event_post(apEvent, AP_EVT_ADV_ENB);
      }
      else
      {
        apPostError(evt->status);
      }
    } break;
    case SNP_ADV_ENDED_EVT:
    {
      snpAdvStatusEvt_t *evt = (snpAdvStatusEvt_t *) param;
      if (evt->status == SNP_SUCCESS) {
        advertising = false;
        Event_post(apEvent, AP_EVT_ADV_END);
      }
      else
      {
        apPostError(evt->status);
      }
    } break;
    /*
     * Unused because value size handling in this library is agnostic of
     * MTU (maximum transmission unit) size.
     */
    // case SNP_ATT_MTU_EVT:
    // {
    // } break;
    case SNP_SECURITY_EVT:
    {
      snpSecurityEvt_t *evt = (snpSecurityEvt_t *) param;
      ble.securityState = evt->state;
      if (evt->status == SNP_SUCCESS) {
        Event_post(apEvent, AP_EVT_SECURITY_STATE);
      }
      else
      {
        apPostError(evt->status);
      }
    } break;
    case SNP_AUTHENTICATION_EVT:
    {
      memcpy(&eventHandlerData, param, sizeof(eventHandlerData));
      Event_post(apEvent, AP_EVT_HANDLE_AUTH_EVT);
    } break;
    case SNP_ERROR_EVT:
    {
      snpErrorEvt_t *evt = (snpErrorEvt_t *) param;
      ble.opcode = evt->opcode;
      apPostError(evt->status);
    } break;
  }
}

/*
 * Can't return specific error beacuse that's only in the async handler,
 * so we return true/false and let caller check ble.error.
 */
static bool apEventPend(uint32_t event)
{
  ble.error = BLE_SUCCESS;
  uint32_t postedEvent = Event_pend(apEvent, AP_NONE, event | AP_ERROR,
                                    AP_EVENT_PEND_TIMEOUT);
  /* Bug in NPI causes this specific event to get posted twice */
  if (event == AP_EVT_ADV_DATA_RSP)
  {
    Event_pend(apEvent, AP_NONE, event | AP_ERROR, 1);
  }
  bool status = false;
  if (postedEvent & event)
  {
    status = true;
  }
  else if (postedEvent == 0)
  {
    ble.error = BLE_TIMEOUT;
    status = false;
  }
  else if (postedEvent & AP_ERROR)
  {
    // Function that posts AP_ERROR should set ble.error
    status = false;
  }
  return status;
}

static inline void apPostError(uint8_t status)
{
  ble.error = status;
  Event_post(apEvent, AP_ERROR);
}

/*
 * Handles propogating errors through stack to Energia sketch. Use when
 * failure of the checked call requires immediate return (e.g. if the
 * next statements depend on it).
 */
static bool isError(uint8_t status)
{
  if (status == BLE_CHECK_ERROR)
  {
    return true;
  }
  else if ((ble.error = status) != SNP_SUCCESS)
  {
    return true;
  }
  return false;
}
