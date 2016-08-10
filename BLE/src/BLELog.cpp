
#include "BLELog.h"

#define ACQUIRE_TIMEOUT 50 // ms

/* Bit mask that determines what is logged. */
uint8_t logLevel = BLE_LOG_NONE;

/* Prevents competition with the Energia user's Serial calls */
volatile bool apLogLock = false;

/* Used to determine if caller is the Energia sketch task. */
Task_Handle apTask = NULL;
Task_Handle owner = NULL;

/* Prevents simultaneous logging, but not other Serial calls. */
volatile uint8_t logLock = 0;

/* Indicates when another task wants to log. If set when release
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

void logSetMainTask(Task_Handle mainTask)
{
  apTask = mainTask;
}

void logParam(const char name[], const uint8_t buf[],
              uint16_t len, bool isBigEnd)
{
  if (SHOULD_LOG_PARAM)
  {
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
  }
}

void logParam(const char name[], int value, int base)
{
  if (SHOULD_LOG_PARAM)
  {
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
  }
}

void logParam(const char name[], const char value[])
{
  if (SHOULD_LOG_PARAM)
  {
    Serial.print("  ");
    Serial.print(name);
    Serial.print(":");
    Serial.println(value);
  }
}

void logParam(const char value[])
{
  if (SHOULD_LOG_PARAM)
  {
    Serial.print("  ");
    Serial.println(value);
  }
}

/* Array guaranteed to have 16 bytes. */
void logUUID(const uint8_t UUID[], uint8_t UUIDlen)
{
  if (SHOULD_LOG_PARAM)
  {
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
  }
}

void logError(uint8_t status)
{
  if (logAllowed(BLE_LOG_ERRORS))
  {
    Serial.print("ERR ");
    hexPrint(status);
    Serial.println();
  }
}

void logError(const char msg[], uint8_t status)
{
  if (logAllowed(BLE_LOG_ERRORS))
  {
    Serial.print("ERR ");
    hexPrint(status);
    Serial.print(":");
    Serial.println(msg);
  }
}

void logRPC(const char msg[])
{
  if (logAllowed(BLE_LOG_RPCS))
  {
    Serial.print("RPC:");
    Serial.println(msg);
  }
}

bool advDataCnfPendedWorkaround = false;
void logAsync(const char name[], uint8_t cmd1)
{
  if (cmd1 == SNP_SET_ADV_DATA_CNF)
  {
    advDataCnfPendedWorkaround = !advDataCnfPendedWorkaround;
    // Log every other one of this type because it gets posted twice
    if (!advDataCnfPendedWorkaround)
    {
      return;
    }
  }
  if (logAllowed(BLE_LOG_REC_MSGS))
  {
    Serial.print("Rec msg ");
    hexPrint(cmd1);
    Serial.print(":");
    Serial.println(name);
  }
}

void logChar(const char action[])
{
  if (logAllowed(BLE_LOG_CHARACTERISTICS))
  {
    Serial.print(action);
    Serial.println(" char value");
  }
}

void logReset(void)
{
  logLevel = BLE_LOG_NONE;
  apLogLock = false;
  apTask = NULL;
  logLock = 0;
  logLockReq = false;
  apLogLast = 0x00;
  otherLogLast = 0x00;
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
 * Not safe for real applications,
 * but here we just need to prevent overlapping serial writes.
 * After ACQUIRE_TIMEOUT ms it just prints anyway. We increment and
 * decrement logLock instead of setting equal to 1 and 0 in case of a
 * timeout. This way logLock will track the number of tasks and won't
 * be set to 0 while a task is still logging.
 */
void logAcquire(void)
{
  uint32_t startTime = millis();
  if (Task_self() == owner)
  {
    // Task already owns. Nested log call.
  }
  else if (Task_self() == apTask)
  {
    while ((logLock) &&
           (millis() - startTime < ACQUIRE_TIMEOUT))
    {
      Task_yield();
    }
  }
  else // NPI Task
  {
    logLockReq = true;
    while ((apLogLock || logLock) &&
           (millis() - startTime < ACQUIRE_TIMEOUT))
    {
      Task_yield();
    }
  }
  logLock++;
  owner = Task_self();
}

void logRelease(void)
{
  logLock--;
  if (Task_self() == owner)
  {
    owner = NULL;
  }
  if (Task_self() == apTask)
  {
    uint32_t startTime = millis();
    while (logLockReq && (millis() - startTime < ACQUIRE_TIMEOUT))
    {
      bool apLogLockSaved = apLogLock;
      apLogLock = false;
      Task_yield();
      apLogLock = apLogLockSaved;
    }
  }
  else // NPI Task
  {
    logLockReq = false;
  }
}
