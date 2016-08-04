
#ifndef BLE_SERVICE_LIST_H
#define BLE_SERVICE_LIST_H

#include "BLETypes.h"

typedef struct BLE_Service_Node
{
  BLE_Service_Node *next;
  BLE_Service *service;
} BLE_Service_Node;

int BLE_registerService(BLE_Service *bleService);
void BLE_resetCCCD(void);
void BLE_charWriteValue(BLE_Char *bleChar, void *pData, size_t size, bool isBigEnd);
void BLE_clearServices(void);

#endif
