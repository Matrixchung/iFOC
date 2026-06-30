#ifndef _GLOBAL_INCLUDE_H
#define _GLOBAL_INCLUDE_H

#ifdef __cplusplus
extern "C" {
#endif
/* ---------------- C Scope --------------- */ // For executed by C file
#include "stdint-gcc.h"
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "rtc.h"
#include "usart.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "fdcan.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

void app_main(void);
void ADC1_2_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void vofa_output();

extern unsigned long run_time_stats;

#if defined(configUSE_MALLOC_FAILED_HOOK)
void vApplicationMallocFailedHook(void);
#endif
#if defined(configCHECK_FOR_STACK_OVERFLOW)
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
#endif

#ifdef __cplusplus
}
#endif
#endif
