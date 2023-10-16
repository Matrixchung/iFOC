#ifndef _KEY_H
#define _KEY_H

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "user_task.h"

typedef struct
{
  uint8_t state : 2; // 0 - not pressed, 1 - debounced, 2 - long pressed
  uint8_t dbCnt : 4;
  uint16_t lpCnt : 10;
} keyState;

#define KEY_SCAN_INTERVAL_MS          10
#define KEY_DEBOUNCE_MS               100
#define KEY_LONG_PRESS_MS             1000

#define BIT_KEY_PRESSED             (1<<0)
#define BIT_KEY_LONG_PRESSED        (1<<1)
#define BIT_KEY_LONG_PRESS_RELEASED (1<<2)

extern EventGroupHandle_t mainEventGroup;
void key_scan(void);

#endif