
#include "BLELog.h"

#define ACQUIRE_TIMEOUT 50 // ms

/* Bit mask that determines what is logged. */
uint8_t logLevel = 0;

/* Prevents competition with the Energia user's Serial calls */
bool apLogLock = false;

/* Used to determine if caller is the Energia sketch task. */
Task_Handle apTask = NULL;

/* Prevents simultaneous logging, but not other Serial calls. */
uint8_t logLock = 0;

/* Indicates when another task wants to log. If set when logRelease
   is called, yields to another task. */
bool logLockReq = false;

/* The last log mode called; used for followup calls. */
uint8_t apLogLast = 0x00;
uint8_t otherLogLast = 0x00;
#define GET_LOG_LAST ((apTask == Task_self()) ? apLogLast : otherLogLast)

static bool logAllowed(uint8_t mode);
static void logAcquire(void);
static void logRelease(void);

void logSetMainTask(Task_Handle mainTask)
{
  apTask = mainTask;
}

void logParam(const char name[], int value, int base)
{
  if (logLevel & GET_LOG_LAST)
  {
    logAcquire();
    Serial.print("  ");
    Serial.print(name);
    Serial.print(":");
    Serial.println(value, base);
    logRelease();
  }
}

void logParam(const char name[], int value)
{
  logParam(name, value, DEC);
}

void logParam(const char value[])
{
  if (logLevel & GET_LOG_LAST)
  {
    logAcquire();
    Serial.print("  ");
    Serial.println(value);
    logRelease();
  }
}

void logError(const char msg[])
{
  if (logAllowed(BLE_LOG_ERRORS))
  {
    logAcquire();

    logRelease();
  }
}

void logRPC(const char msg[])
{
  if (logAllowed(BLE_LOG_RPCS))
  {
    logAcquire();
    Serial.print("RPC:");
    Serial.println(msg);
    logRelease();
  }
}

void logAsync(const char name[], uint8_t cmd1)
{
  if (logAllowed(BLE_LOG_REC_MSGS))
  {
    logAcquire();
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
    logRelease();
  }
}

void logChar(const char msg[])
{
  if (logAllowed(BLE_LOG_CHARACTERISTICS))
  {
    logAcquire();

    logRelease();
  }
}

void logState(const char msg[])
{
  if (logAllowed(BLE_LOG_STATE))
  {
    logAcquire();

    logRelease();
  }
}

static bool logAllowed(uint8_t mode)
{
  if (Task_self() == apTask)
  {
    apLogLast = mode;
  }
  else
  {
    otherLogLast = mode;
  }
  return logLevel & mode;
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
  if (Task_self() == apTask)
  {
    while ((logLock) &&
           (millis() - startTime < ACQUIRE_TIMEOUT))
    {
      Task_yield();
    }
  }
  else // NPI Task
  {
    while ((apLogLock || logLock) &&
           (millis() - startTime < ACQUIRE_TIMEOUT))
    {
      logLockReq = true;
      Task_yield();
      logLockReq = false;
    }
  }
  logLock++;
}

static void logRelease(void)
{
  logLock--;
  uint32_t startTime = millis();
  while (logLockReq && (millis() - startTime < ACQUIRE_TIMEOUT))
  {
    bool apLogLockSaved = apLogLock;
    apLogLock = false;
    Task_yield();
    apLogLock = apLogLockSaved;
  }
}
