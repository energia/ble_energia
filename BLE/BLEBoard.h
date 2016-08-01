
#ifndef BLE_BOARD_H
#define BLE_BOARD_H

#ifdef __MSP432P401R__
#ifdef Board_WATCHDOG
#undef Board_WATCHDOG
#endif //Board_WATCHDOG
#ifdef Board_WIFI
#undef Board_WIFI
#endif //Board_WIFI
#include "variants/MSP_EXP432P401R/Board.h"

#define PIN2_5 19
#define PIN6_0 2
#define PIN6_1 23
#define PIN6_7 35

#define CC2650_RESET_PIN   PIN6_7
#define BLE_Board_MRDY     PIN6_0
#define BLE_Board_SRDY     PIN2_5
#define BLE_SBL_BL_PIN     PIN6_0

#define BLE_UART_ID Board_UARTA2 // =1, USB Serial is Board_UARTA0=0
#endif //__MSP432P401R__

void initBoard(void);

#endif
