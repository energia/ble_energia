
#include "BLELog.h"

#define ACQUIRE_TIMEOUT 50 // ms

/* Bit mask that determines what is logged. */
uint8_t logLevel = 0;

/* Prevents competition with the Energia user's Serial calls */
volatile bool apLogLock = false;

/* Used to determine if caller is the Energia sketch task. */
Task_Handle apTask = NULL;

/* Prevents simultaneous logging, but not other Serial calls. */
volatile uint8_t logLock = 0;

/* Indicates when another task wants to log. If set when logRelease
   is called, yields to another task. */
volatile bool logLockReq = false;

/* The last log mode called; used for followup calls. */
uint8_t apLogLast = 0x00;
uint8_t otherLogLast = 0x00;
#define SHOULD_LOG_PARAM (logLevel & ((apTask == Task_self()) ? apLogLast : otherLogLast))

static void hexPrint(int num);
static void hexPrintBigEnd(const uint8_t buf[], uint16_t len);
static void hexPrintLitEnd(const uint8_t buf[], uint16_t len);
static bool logAllowed(uint8_t mode);
static void logAcquire(void);
static void logRelease(void);

void logSetMainTask(Task_Handle mainTask)
{
  apTask = mainTask;
}

void logParam(const char name[], const uint8_t buf[],
              uint16_t len, bool isBigEnd)
{
  if (SHOULD_LOG_PARAM)
  {
    logAcquire();
    Serial.print("  ");
    Serial.print(name);
    Serial.print(":0x");
    if (isBigEnd)
    {
      hexPrintBigEnd(buf, len);
    }
    else
    {
      hexPrintLitEnd(buf, len);
    }
    Serial.println();
    logRelease();
  }
}

void logParam(const char name[], int value, int base)
{
  if (SHOULD_LOG_PARAM)
  {
    logAcquire();
    Serial.print("  ");
    Serial.print(name);
    Serial.print(":");
    if (base == HEX)
    {
      hexPrint(value);
      Serial.println();
    }
    else if (base == BIN)
    {
      Serial.print("0b");
      Serial.println(value, base);
    }
    else
    {
      Serial.println(value);
    }
    logRelease();
  }
}

void logParam(const char name[], const char value[])
{
  if (SHOULD_LOG_PARAM)
  {
    logAcquire();
    Serial.print("  ");
    Serial.print(name);
    Serial.print(":");
    Serial.println(value);
    logRelease();
  }
}

void logParam(const char value[])
{
  if (SHOULD_LOG_PARAM)
  {
    logAcquire();
    Serial.print("  ");
    Serial.println(value);
    logRelease();
  }
}

/* Array guaranteed to have 16 bytes. */
void logUUID(const uint8_t UUID[], uint8_t UUIDlen)
{
  if (SHOULD_LOG_PARAM)
  {
    logAcquire();
    Serial.print("  UUID:0x");
    if (UUIDlen == SNP_128BIT_UUID_SIZE)
    {
      hexPrintLitEnd(&UUID[12], 4);
      Serial.print("-");
      hexPrintLitEnd(&UUID[10], 2);
      Serial.print("-");
      hexPrintLitEnd(&UUID[8], 2);
      Serial.print("-");
      hexPrintLitEnd(&UUID[6], 2);
      Serial.print("-");
      hexPrintLitEnd(&UUID[0], 6);
    }
    else if (UUIDlen == SNP_16BIT_UUID_SIZE)
    {
      hexPrintLitEnd(&UUID[0], 2);
    }
    Serial.println();
    logRelease();
  }
}

void logError(uint8_t status)
{
  if (logAllowed(BLE_LOG_ERRORS))
  {
    logAcquire();
    Serial.print("ERR ");
    hexPrint(status);
    Serial.println();
    logRelease();
  }
}

void logError(const char msg[], uint8_t status)
{
  if (logAllowed(BLE_LOG_ERRORS))
  {
    logAcquire();
    Serial.print("ERR ");
    hexPrint(status);
    Serial.print(":");
    Serial.println(msg);
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
    Serial.print("Rec msg ");
    hexPrint(cmd1);
    Serial.print(":");
    Serial.println(name);
    if (cmd1 == SNP_SET_ADV_DATA_CNF)
    {
      Serial.println("Bug->double evt");
    }
    logRelease();
  }
}

void logChar(const char action[])
{
  if (logAllowed(BLE_LOG_CHARACTERISTICS))
  {
    logAcquire();
    Serial.print(action);
    Serial.println(" char value");
    logRelease();
  }
}

static void hexPrint(int num)
{
  Serial.print("0x");
  if (num < 0x10)
  {
    Serial.print("0");
  }
  Serial.print(num, HEX);
}

static void hexPrintBigEnd(const uint8_t buf[], uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    if (buf[i] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(buf[i], HEX);
  }
}

static void hexPrintLitEnd(const uint8_t buf[], uint16_t len)
{
  /* Overflows from 0 to 65535 */
  for (uint16_t i = len-1; i < len; i--)
  {
    if (buf[i] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(buf[i], HEX);
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
