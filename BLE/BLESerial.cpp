
#include "BLESerial.h"

uint8_t rxBuffer[BLE_SERIAL_BUFFER_SIZE] = {0};
volatile uint16_t rxWriteIndex = 0;
volatile uint16_t rxReadIndex = 0;

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
