
#ifndef BLE_LOG_H
#define BLE_LOG_H

#include "BLETypes.h"
#include <ti/sysbios/knl/Task.h>

extern uint8_t logLevel;

/* Set the task given ability to lock out the other. */
void logSetAPTask(Task_Handle apTask);

/* Main logging functions that will call logAcquire(). */
void logError(uint8_t status);
void logError(const char msg[], uint8_t status);
void logRPC(const char msg[]);
void logAsync(const char name[], uint8_t cmd1);
void logChar(const char action[]);

/* Secondary logging functions that only log if the last main function logs. */
void logParam(const char name[], const uint8_t buf[], uint16_t len, bool isBigEnd=true);
void logParam(const char name[], int value, int base=DEC);
void logParam(const char name[], const char value[]);
void logParam(const char value[]);
void logUUID(const uint8_t UUID[], uint8_t len);

/*
 * Wrap a set of logging calls in logAcquire and logRelease to ensure they
 * are not interleaved with other logging calls. All of the main logging
 * functions call these by default
 */
void logAcquire(void);
void logRelease(void);

/* Used in ble.end(). */
void logReset(void);

#endif
