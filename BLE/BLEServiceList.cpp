
#include "BLEServiceList.h"
#include "BLETypes.h"

BLE_Service_Node *bleServiceListHead = NULL;
BLE_Service_Node *bleServiceListTail = NULL;

void addServiceNode(BLE_Service *service)
{
  BLE_Service_Node *newNode = (BLE_Service_Node *) malloc(sizeof(BLE_Service_Node));
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

BLE_Char* getChar(int handle)
{
  BLE_Service *service = getServiceWithChar(handle);
  uint8_t i;
  for (i = 0; i < service->numChars; i++)
  {
    if (service->chars[i]->handle == handle)
    {
      return service->chars[i];
    }
  }
  return NULL;
}

BLE_Service* getService(int handle)
{
  BLE_Service *service = getServiceWithChar(handle);
  if (service->handle == handle)
  {
    return service;
  }
  return NULL; // bad handle or handle of a characteristic
}

BLE_Service* getServiceWithChar(int handle)
{
  BLE_Service_Node *curr = bleServiceListHead;
  while (curr->next && curr->next->service->handle <= handle)
  {
    curr = curr->next;
  } // At end of loop, curr->service->handle <= handle < curr-next->service->handle
  return curr->service;
}
