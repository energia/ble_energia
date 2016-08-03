
#ifndef BLE_DEBUG_H
#define BLE_DEBUG_H

extern int flag0;
extern int flag1;
extern int flag2;
extern int flag3;
extern int flag4;
extern int flag5;

#define DEBUG_PINS_LIST /*
*/ DEBUG_FXN(3_7, 31); /*
*/ DEBUG_FXN(3_5, 32); /*
*/ DEBUG_FXN(5_1, 33); /*
*/ DEBUG_FXN(2_3, 34)

#define DEBUG_FXN(pin, pinNum) void ping##pin(void)

// DEBUG_PINS_LIST;
void ping3_5(void); void ping5_1(void); void ping2_3(void);

#undef DEBUG_FXN

#endif
