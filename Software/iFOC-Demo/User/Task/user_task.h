#ifndef __USER_TASK_H__
#define __USER_TASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "key.h"
#include "vofa.h"
#include "math.h"
#include "foc.h"

extern EventGroupHandle_t mainEventGroup;
extern FOC_TypeDef foc_driver;

void StartUserTask(void);
void FOC_Loop_IRQHandler_LL(void);

#endif