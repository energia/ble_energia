
#ifndef BLESERIAL_H
#define BLESERIAL_H

#include <stddef.h>
#include <stdint.h>

#define BLE_SERIAL_BUFFER_SIZE 128

extern uint8_t rxBuffer[BLE_SERIAL_BUFFER_SIZE];
extern volatile uint16_t rxWriteIndex;
extern volatile uint16_t rxReadIndex;

int BLESerial_available(void);
int BLESerial_read(void);
int BLESerial_peek(void);
void BLESerial_flush(void);

#endif
