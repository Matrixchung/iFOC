#ifndef __UART_HANDLER_H
#define __UART_HANDLER_H

#include "main.h"
#include "usart.h"
// using FIFO Interrupt in FreeRTOS: https://blog.csdn.net/luliplus/article/details/132383525
#include "fifo.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "utils.h"

// Used with USART RX&TX DMA Enabled
#define RX_BUF_SIZE 128
extern char UART_RX_BUF[RX_BUF_SIZE];

#define TX_BUF_SIZE 128
extern char UART_TX_BUF[TX_BUF_SIZE];

#define TX_IT 2
#define TX_BLOCK 1
#define TX_DMA 0

// extern fifo_s_t uart_rx_fifo;
// extern uint8_t UART_RxCpltFlag;
// extern uint16_t tx_fifo_len;
extern osSemaphoreId uart_rx_finished_sem;

HAL_StatusTypeDef UART_HandlerInit(UART_HandleTypeDef *huart);
uint16_t UART_GetRxLen(void);
char UART_PeekRxFifo(uint16_t ptr);
void UART_GetRxFifo(char *dest, uint16_t len);
void UART_PutTxFifo(char *data, uint16_t len);
HAL_StatusTypeDef UART_Transmit(uint8_t blocking);
void UART_printf(char *fmt, ...);
extern void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
#endif