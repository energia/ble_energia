
#include <BLE.h>
#include "BLESerial.h"

uint8_t rxBuffer[BLE_SERIAL_BUFFER_SIZE] = {0};
volatile uint16_t rxWriteIndex = 0;
volatile uint16_t rxReadIndex = 0;

/* Nordic Semiconductor's UART Service */

BLE_Char rxChar =
{
  {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
   0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E},
  BLE_WRITABLE,
  "Client TX"
};

BLE_Char txChar =
{
  {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
   0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E},
  BLE_READABLE | BLE_NOTIFIABLE,
  "Client RX"
};

BLE_Char *serialServiceChars[] = {&rxChar, &txChar};

BLE_Service serialService =
{
  {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
   0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E},
  2, serialServiceChars
};

int BLE::available(void)
{
  int numChars = 0;
  numChars = (rxWriteIndex >= rxReadIndex) ?
    (rxWriteIndex - rxReadIndex)
    : BLE_SERIAL_BUFFER_SIZE - (rxReadIndex - rxWriteIndex);
  return numChars;
}

int BLE::peek(void)
{
  int iChar = -1;
  if (available())
  {
    iChar = (int) rxBuffer[rxReadIndex];
  }
  return iChar;
}

int BLE::read(void)
{
  int iChar = -1;
  iChar = peek();
  if (0 <= iChar)
  {
    rxReadIndex = (rxReadIndex + 1) % BLE_SERIAL_BUFFER_SIZE;
  }
  return iChar;
}

/*
 * Serial.flush is supposed to wait for outgoing data to transmit, but since
 * a write call operates in the same task all the way throught the RPC, we
 * don't need to worry.
 */
void BLE::flush(void)
{
  /*
   * Essentially fast-forward the read index to the write index.
   * When rxReadIndex == rxWriteIndex, available() returns 0.
   */
  rxReadIndex = rxWriteIndex;
  return;
}

size_t BLE::write(uint8_t c)
{
  if (writeValue(&txChar, c) == BLE_SUCCESS)
  {
    return 1;
  }
  return 0;
}

size_t BLE::write(const uint8_t buffer[], size_t size)
{
  if (writeValue(&txChar, buffer, size) == BLE_SUCCESS)
  {
    return size;
  }
  return 0;
}


/* Called in the NPI task when the GATT client writes data. */
void BLESerial_clientWrite(uint16_t len, uint8_t *pData)
{
  /* If larger than buffer only write the last part. */
  if (BLE_SERIAL_BUFFER_SIZE - 1 < len)
  {
    pData += len - BLE_SERIAL_BUFFER_SIZE - 1;
    len = BLE_SERIAL_BUFFER_SIZE - 1;
  }

  bool overwritesReadIdx = false;
  /* Fits in buffer without wrapping. */
  if (len < BLE_SERIAL_BUFFER_SIZE - rxWriteIndex)
  {
    memcpy(&rxBuffer[rxWriteIndex], pData, len);
    /* If the read index was overwritten, move past end of
       write to earliest valid data. */
    if (rxWriteIndex < rxReadIndex && rxReadIndex < rxWriteIndex + len)
    {
      overwritesReadIdx = true;
    }
    rxWriteIndex += len;
  }
  /* Wraps end of buffer */
  else
  {
    uint16_t firstLen = BLE_SERIAL_BUFFER_SIZE - rxWriteIndex;
    uint16_t lastLen = len - firstLen;
    memcpy(&rxBuffer[rxWriteIndex], pData, firstLen);
    memcpy(&rxBuffer[0], pData + firstLen, lastLen);
    /* If the read index was overwritten, move past end of
       write to earliest valid data. */
    if (rxReadIndex < lastLen || rxWriteIndex < rxReadIndex)
    {
      overwritesReadIdx = true;
    }
    rxWriteIndex = lastLen;
  }
  if (overwritesReadIdx)
  {
    rxReadIndex = (rxWriteIndex + 1) % BLE_SERIAL_BUFFER_SIZE;
  }
}
