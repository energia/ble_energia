
#include "BLELog.h"

#include <ti/sysbios/knl/Task.h>

#define ACQUIRE_TIMEOUT 100 // ms

uint8_t logLevel = 0;
uint8_t logLast = 0;

uint8_t logLock = 0;

static void logAcquire(void);
static void logRelease(void);

void logParam(const char name[], int value, int base)
{
  logAcquire();
  if (logLevel & logLast)
  {
    Serial.print("  ");
    Serial.print(name);
    Serial.print(":");
    Serial.println(value, base);
  }
  logRelease();
}

void logParam(const char name[], int value)
{
  logAcquire();
  logParam(name, value, DEC);
  logRelease();
}

void logError(const char msg[])
{
  logAcquire();
  logLast = BLE_LOG_ERRORS;
  if (logLevel & BLE_LOG_ERRORS)
  {

  }
  logRelease();
}

void logRPC(const char msg[])
{
  logAcquire();
  logLast = BLE_LOG_SENT_MSGS;
  if (logLevel & BLE_LOG_SENT_MSGS)
  {
    Serial.print("RPC:");
    Serial.println(msg);
  }
  logRelease();
}

void logAsync(const char name[], uint8_t cmd1)
{
  logAcquire();
  logLast = BLE_LOG_REC_MSGS;
  if (logLevel & BLE_LOG_REC_MSGS)
  {
    Serial.print("Rec msg 0x");
    if (cmd1 < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(cmd1, HEX);
    Serial.print(":");
    Serial.println(name);
    if (cmd1 == SNP_SET_ADV_DATA_CNF)
    {
      Serial.println("Bug->double evt");
    }
  }
  logRelease();
}

void logSync(uint8_t cmd1)
{
  logAcquire();
  logLast = BLE_LOG_REC_MSGS;
  if (logLevel & BLE_LOG_REC_MSGS)
  {
  }
  logRelease();
}

void logChar(const char msg[])
{
  logAcquire();
  logLast = BLE_LOG_CHARACTERISTICS;
  if (logLevel & BLE_LOG_CHARACTERISTICS)
  {

  }
  logRelease();
}

void logState(const char msg[])
{
  logAcquire();
  logLast = BLE_LOG_STATE;
  if (logLevel & BLE_LOG_STATE)
  {

  }
  logRelease();
}

/*
 * Dijkstra is rolling in his grave. Not safe for real applications,
 * but here we just need to prevent overlapping serial writes.
 * After ACQUIRE_TIMEOUT ms it just prints anyway. We increment and
 * decrement logLock instead of setting equal to 1 and 0 in case of a
 * timeout. This way logLock will track the number of tasks and won't
 * be set to 0 while a task is still logging.
 */
static void logAcquire(void)
{
  uint32_t startTime = millis();
  while (logLock && (millis() - startTime < ACQUIRE_TIMEOUT))
  {
    Task_yield();
  }
  logLock++;
}

static void logRelease(void)
{
  logLock--;
}
