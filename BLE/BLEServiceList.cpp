
#include <sap.h>
#include <snp.h>

#include "BLESerial.h"
#include "BLEServiceList.h"

BLE_Service_Node *bleServiceListHead = NULL;
BLE_Service_Node *bleServiceListTail = NULL;

static void addServiceNode(BLE_Service *service);
static BLE_Char* getChar(uint16_t handle);
static BLE_Char* getCCCD(uint16_t handle);
static BLE_Service* getServiceWithChar(uint16_t handle);
static void constructService(SAP_Service_t *service, BLE_Service *bleService);
static void constructChar(SAP_Char_t *sapChar, BLE_Char *bleChar);
static uint8_t getUUIDLen(uint8_t *UUID);
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

int BLE_registerService(BLE_Service *bleService)
{
  SAP_Service_t *service = (SAP_Service_t *) malloc(sizeof(*service));
  constructService(service, bleService);
  int status = SAP_registerService(service);
  if (status != SNP_FAILURE) {
    bleService->_handle = service->serviceHandle;
    for (uint8_t i = 0; i < bleService->numChars; i++)
    {
      bleService->chars[i]->_handle = service->charAttrHandles[i].valueHandle;
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

void BLE_resetCCCD(void)
{
  BLE_Service_Node *curr = bleServiceListHead;
  while (curr)
  {
    for (uint8_t i = 0; i < curr->service->numChars; i++)
    {
      curr->service->chars[i]->_CCCD = 0;
    }
    curr = curr->next;
  }
}

uint8_t BLE_charValueInit(BLE_Char *bleChar, size_t size)
{
  if (bleChar->_valueLen != size && bleChar->_value)
  {
    free(bleChar->_value);
    bleChar->_value = NULL;
  }
  if (bleChar->_value == NULL)
  {
    bleChar->_value = (void *) malloc(size);
    bleChar->_valueLen = size;
  }
  return BLE_SUCCESS;
}

static void addServiceNode(BLE_Service *service)
{
  BLE_Service_Node *newNode = (BLE_Service_Node *) malloc(sizeof(*newNode));
  newNode->next = NULL;
  newNode->service = service;
  if (bleServiceListHead == NULL)
  {
    bleServiceListHead = newNode;
    bleServiceListTail = newNode;
  }
  else
  {
    bleServiceListTail->next = newNode;
    bleServiceListTail = newNode;
  }
}

static BLE_Char* getChar(uint16_t handle)
{
  BLE_Service *service = getServiceWithChar(handle);
  for (uint8_t i = 0; i < service->numChars; i++)
  {
    if (service->chars[i]->_handle == handle)
    {
      return service->chars[i];
    }
  }
  return NULL;
}

static BLE_Char* getCCCD(uint16_t handle)
{
  BLE_Service *service = getServiceWithChar(handle);
  for (uint8_t i = 0; i < service->numChars; i++)
  {
    if (service->chars[i]->_CCCDHandle == handle)
    {
      return service->chars[i];
    }
  }
  return NULL;
}

static BLE_Service* getServiceWithChar(uint16_t handle)
{
  BLE_Service_Node *curr = bleServiceListHead;
  while (curr->next && curr->next->service->_handle <= handle)
  {
    curr = curr->next;
  } // At end of loop, curr->service->_handle <= handle < curr-next->service->_handle
  return curr->service;
}

static void constructService(SAP_Service_t *service, BLE_Service *bleService)
{
  bleService->_handle = 0;
  service->serviceUUID.len    = getUUIDLen(bleService->UUID);
  service->serviceUUID.pUUID  = bleService->UUID;
  service->serviceType        = SNP_PRIMARY_SERVICE;
  service->charTableLen       = bleService->numChars; // sizeof with static array?
  service->charTable          = (SAP_Char_t *) malloc(service->charTableLen *
                                                    sizeof(*service->charTable));
  service->context            = NULL;
  service->charReadCallback   = serviceReadAttrCB;
  service->charWriteCallback  = serviceWriteAttrCB;
  service->cccdIndCallback    = serviceCCCDIndCB;
  service->charAttrHandles    = (SAP_CharHandle_t *) malloc(service->charTableLen *
                                                            sizeof(*service->charAttrHandles));
  for (uint8_t i = 0; i < bleService->numChars; i++)
  {
    constructChar(&service->charTable[i], bleService->chars[i]);
  }
}

static void constructChar(SAP_Char_t *sapChar, BLE_Char *bleChar)
{
  /* TODO remove this. Bug in BLE stack makes this fail otherwise. */
  bleChar->valueFormat = 0;

  /* Initialize characteristic to have one byte with a value of 0.
     Override by calling writeValue in the main sketch. */
  bleChar->_value = calloc(1, 1);
  bleChar->_valueLen = 1;

  /* Default to no notifications or indications. */
  bleChar->_CCCD = 0;

  bleChar->_CCCDHandle = 0;

  sapChar->UUID.len    = getUUIDLen(bleChar->UUID);
  sapChar->UUID.pUUID  = bleChar->UUID;

  sapChar->properties  = bleChar->properties;
  sapChar->permissions = ((sapChar->properties & BLE_READABLE)
                            ? SNP_GATT_PERMIT_READ : 0)
                       | ((sapChar->properties & (BLE_WRITABLE_NORSP | BLE_WRITABLE))
                            ? SNP_GATT_PERMIT_WRITE : 0);

  if (bleChar->charDesc)
  {
    sapChar->pUserDesc = (SAP_UserDescAttr_t *) malloc(sizeof(*sapChar->pUserDesc));
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
  sapChar->pCccd = (SAP_UserCCCDAttr_t *) malloc(sizeof(*sapChar->pCccd));
  sapChar->pCccd->perms          = SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE;
  if (bleChar->valueFormat)
  {
    sapChar->pFormat = (SAP_FormatAttr_t *) malloc(sizeof(*sapChar->pFormat));
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

static uint8_t getUUIDLen(uint8_t *UUID)
{
  for (uint8_t i = SNP_16BIT_UUID_SIZE; i < SNP_128BIT_UUID_SIZE; i++)
  {
    if (UUID[i] != 0)
    {
      return SNP_128BIT_UUID_SIZE;
    }
  }
  return SNP_16BIT_UUID_SIZE;
}

static uint8_t serviceReadAttrCB(void *context,
                                 uint16_t connectionHandle,
                                 uint16_t charHdl, uint16_t offset,
                                 uint16_t maxSize, uint16_t *len,
                                 uint8_t *pData)
{
  (void) context;
  (void) connectionHandle;
  uint8_t status = SNP_SUCCESS;
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
  (void) context;
  (void) connectionHandle;
  BLE_Char *bleChar = getChar(charHdl);
  uint8_t status = SNP_SUCCESS;
  if (bleChar == NULL)
  {
    status = SNP_UNKNOWN_ATTRIBUTE;
  }
  status = BLE_charValueInit(bleChar, len);
  if (status == SNP_SUCCESS)
  {
    memcpy((uint8_t *) bleChar->_value, pData, len);
  }
  if (bleChar == &rxChar)
  {
    BLESerial_clientWrite(len, pData);
  }
  return status;
}

static uint8_t serviceCCCDIndCB(void *context,
                                uint16_t connectionHandle,
                                uint16_t cccdHdl, uint8_t type,
                                uint16_t value)
{
  (void) context;
  (void) type;
  (void) connectionHandle;
  uint8_t status = SNP_SUCCESS;
  bool notify = (value == SNP_GATT_CLIENT_CFG_NOTIFY);
  bool indicate = (value == SNP_GATT_CLIENT_CFG_INDICATE);
  BLE_Char *bleChar = getCCCD(cccdHdl);
  if (bleChar == NULL)
  {
    status = SNP_UNKNOWN_ATTRIBUTE;
  }
  // Only 0, or either notify/indicate but not both, is valid.
  else if (!(value == 0 || (!notify || !indicate)))
  {
    status = SNP_INVALID_PARAMS;
  }
  // Attempting to set to mode not allowed by char properties
  else if ((notify && !(bleChar->properties & BLE_NOTIFIABLE))
        || (indicate && !(bleChar->properties & BLE_INDICATABLE)))
  {
    status = SNP_NOTIF_IND_NOT_ALLOWED;
  }
  else
  {
    bleChar->_CCCD = (byte) value;
  }
  return status;
}
