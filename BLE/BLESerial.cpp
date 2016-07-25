
#include "BLESerial.h"

#define BLE_SERIAL_BUFFER_SIZE 128

uint8_t rxBuffer[BLE_SERIAL_BUFFER_SIZE] = {0};
volatile uint16_t rxWriteIndex = 0;
volatile uint16_t rxReadIndex = 0;

/* Nordic Semiconductor's UART Service */

BLE_Char txChar =
{
  {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
   0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E},
  BLE_READABLE | BLE_NOTIFIABLE,
  "Client RX"
};

BLE_Char rxChar =
{
  {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
   0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E},
  BLE_WRITABLE,
  "Client TX"
};

BLE_Char *serialServiceChars[] = {&txChar, &rxChar};

BLE_Service serialService =
{
  {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
   0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E},
  2, serialServiceChars
};

int BLESerial_available(void)
{
  int numChars = 0;
  numChars = (rxWriteIndex >= rxReadIndex) ?
    (rxWriteIndex - rxReadIndex)
    : BLE_SERIAL_BUFFER_SIZE - (rxReadIndex - rxWriteIndex);
  return numChars;
}

int BLESerial_peek(void)
{
  int iChar = -1;
  if (BLESerial_available())
  {
    iChar = (int) rxBuffer[rxReadIndex];
  }
  return iChar;
}

int BLESerial_read(void)
{
  int iChar = -1;
  iChar = BLESerial_peek();
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
void BLESerial_flush(void)
{
  /*
   * Essentially fast-forward the read index to the write index.
   * When rxReadIndex == rxWriteIndex, available() returns 0.
   */
  rxReadIndex = rxWriteIndex;
  return;
}

/* Called in the NPI task when the GATT client writes data. */
void BLESerial_clientWrite(uint16_t len, uint8_t *pData)
{
  /* If larger than buffer only write the last part. */
  if (BLE_SERIAL_BUFFER_SIZE < len)
  {
    pData += len - BLE_SERIAL_BUFFER_SIZE;
    len = BLE_SERIAL_BUFFER_SIZE;
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
    uint16_t lastLen = (len - BLE_SERIAL_BUFFER_SIZE);
    memcpy(&rxBuffer[rxWriteIndex], pData, firstLen);
    memcpy(&rxBuffer[0], pData + firstLen, lastLen);
    /* If the read index was overwritten, move past end of
       write to earliest valid data. */
    if (rxReadIndex < lastLen || rxWriteIndex < rxReadIndex)
    {
      overwritesReadIdx = true;
    }
  }
  if (overwritesReadIdx)
  {
    rxReadIndex = (rxWriteIndex + len + 1) % BLE_SERIAL_BUFFER_SIZE;
  }
}
