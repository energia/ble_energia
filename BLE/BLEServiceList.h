
#ifndef BLE_SERVICE_LIST_H
#define BLE_SERVICE_LIST_H

#include "BLETypes.h"

typedef struct BLE_Service_Node
{
  BLE_Service_Node *next;
  BLE_Service *service;
} BLE_Service_Node;

void addServiceNode(BLE_Service *service);
BLE_Char* getChar(int handle);
BLE_Char* getCCCD(int handle);
BLE_Service* getService(int handle);
BLE_Service* getServiceWithChar(int handle);

#endif