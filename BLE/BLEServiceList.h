
#ifndef BLE_SERVICE_LIST_H
#define BLE_SERVICE_LIST_H

#include "BLETypes.h"

typedef struct BLE_Service_Node
{
  BLE_Service_Node *next;
  BLE_Service *service;
} BLE_Service_Node;

extern uint16_t _connHandle;

int BLE_registerService(BLE_Service *bleService);
BLE_Char* BLE_getChar(int handle);
BLE_Char* BLE_getCCCD(int handle);
void BLE_resetCCCD(void);
uint8_t BLE_charValueInit(BLE_Char *bleChar, size_t size);

#endif
