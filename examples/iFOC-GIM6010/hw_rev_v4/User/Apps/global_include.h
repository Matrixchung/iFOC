#ifndef _GLOBAL_INCLUDE_H
#define _GLOBAL_INCLUDE_H

#ifdef __cplusplus
extern "C" {
#endif
/* ---------------- C Scope --------------- */ // For executed by C file
#include "stdint.h"
#include "at32f403a_407_wk_config.h"
#include "wk_acc.h"
#include "wk_debug.h"
#include "wk_usbfs.h"
#include "wk_gpio.h"
// #include "wk_system.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usbd_core.h"
#include "usbd_int.h"
#include "hid_class.h"
#include "hid_desc.h"

void app_main(void);

void adc_1_2_irq(void);

void tmr2_irq(void);

void drv_fault_irq(void);

void usart1_irq(void);
void usart1_rx_dma_irq(void);
void usart1_tx_dma_irq(void);

void usart3_irq(void);
void usart3_rx_dma_irq(void);
void usart3_tx_dma_irq(void);

void usbfs_irq(void);

void can1_rx0_irq(void);

void hardfault_irq(void);

extern unsigned long run_time_stats;

#if defined(configUSE_MALLOC_FAILED_HOOK)
void vApplicationMallocFailedHook(void);
#endif
#if defined(configCHECK_FOR_STACK_OVERFLOW)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
#endif

#ifdef __cplusplus
}
#endif
#endif
