#include "uart_handler.h"

char UART_RX_BUF[RX_BUF_SIZE];
char UART_TX_BUF[TX_BUF_SIZE];
uint16_t tx_fifo_len = 0;
fifo_s_t uart_rx_fifo, uart_tx_fifo;
// uint8_t UART_RxCpltFlag = 0;

UART_HandleTypeDef *gHuart;
osSemaphoreId uart_rx_finished_sem;

static int inHandlerMode(void)
{
	return __get_IPSR();
}

HAL_StatusTypeDef UART_HandlerInit(UART_HandleTypeDef *huart)
{
    uint32_t u32wk0;
    osSemaphoreDef(_uart_rx_finished_sem);
    uart_rx_finished_sem = osSemaphoreCreate(osSemaphore(_uart_rx_finished_sem), 1);
    osSemaphoreWait(uart_rx_finished_sem, osWaitForever); // First take the sem
    gHuart = huart;
    uart_rx_fifo = *fifo_s_create(RX_BUF_SIZE);
    uart_tx_fifo = *fifo_s_create(TX_BUF_SIZE);
    u32wk0 = gHuart->Instance->SR; // Clear UART status register
    u32wk0 = gHuart->Instance->DR; // Clear UART data register
    UNUSED(u32wk0);
    return HAL_UARTEx_ReceiveToIdle_DMA(gHuart, (uint8_t *)UART_RX_BUF, RX_BUF_SIZE);
}

uint16_t UART_GetRxLen(void)
{
    return fifo_s_used(&uart_rx_fifo);
}

char UART_PeekRxFifo(uint16_t ptr)
{
    return fifo_s_preread(&uart_rx_fifo, ptr);
}

void UART_GetRxFifo(char *dest, uint16_t len)
{
    fifo_s_gets(&uart_rx_fifo, dest, len);
}

void UART_PutTxFifo(char *data, uint16_t len)
{
    fifo_s_puts(&uart_tx_fifo, data, len);
}

HAL_StatusTypeDef UART_Transmit(uint8_t blocking)
{
    // vPortEnterCritical(); // Enter critical area
    if(inHandlerMode() != 0) vPortEnterCritical();
    else
    {
        while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) if(blocking != TX_BLOCK) osThreadYield();
    }
    HAL_StatusTypeDef ret = HAL_ERROR;
    tx_fifo_len = fifo_s_used(&uart_tx_fifo);
    fifo_s_gets(&uart_tx_fifo, (char *)UART_TX_BUF, tx_fifo_len);
    if(blocking == TX_DMA)
    {
        // gHuart->gState = HAL_UART_STATE_READY; // Set UART state to ready to prevent HAL_UART_Transmit_DMA() from returning HAL_BUSY
        ret = HAL_UART_Transmit_DMA(gHuart, (uint8_t *)UART_TX_BUF, tx_fifo_len);
    }
    else if(blocking == TX_BLOCK) ret = HAL_UART_Transmit(gHuart, (uint8_t *)UART_TX_BUF, tx_fifo_len, 0xFF);
    else if(blocking == TX_IT) ret = HAL_UART_Transmit_IT(gHuart, (uint8_t *)UART_TX_BUF, tx_fifo_len);
    // vPortExitCritical();
    if(inHandlerMode() != 0) vPortExitCritical();
    return ret;
}

void UART_printf(char *fmt, ...)
{
    // while(gHuart->gState != HAL_UART_STATE_READY && fifo_s_used(&uart_tx_fifo) == 0); // wait till previous DMA transmit finished
	va_list args;
    uint16_t buf_len = 0;
	va_start(args, fmt);
    // vPortEnterCritical(); // Accessing global variable 'UART_TX_BUF', Enter critical area
	vsnprintf(UART_TX_BUF, TX_BUF_SIZE, fmt, args);
	buf_len = strlen(UART_TX_BUF);
	if(buf_len>TX_BUF_SIZE) buf_len = TX_BUF_SIZE;
	// HAL_UART_Transmit(&huart1, (uint8_t *)buffer, txLen, HAL_MAX_DELAY);
    UART_PutTxFifo(UART_TX_BUF, buf_len);
    UART_Transmit(TX_BLOCK);
    // vPortExitCritical();
	va_end(args);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == gHuart)
    {
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) == SET) __HAL_UART_CLEAR_OREFLAG(huart);
        static uint16_t buffer_pos = 0;
        static uint16_t buffer_length = 0;
        buffer_length = Size - buffer_pos;
        fifo_s_puts(&uart_rx_fifo, &UART_RX_BUF[buffer_pos], buffer_length);
        buffer_pos += buffer_length;
        if(buffer_pos >= RX_BUF_SIZE) buffer_pos = 0; // reset the buffer
        // HAL_UARTEx_GetRxEventType
        if(huart->RxEventType == HAL_UART_RXEVENT_TC || huart->RxEventType == HAL_UART_RXEVENT_IDLE) // Transfer Complete
        {
            // Process or set flag
            // tx_fifo_len = fifo_s_used(&uart_rx_fifo);
            // fifo_s_gets(&uart_rx_fifo, (char *)UART_TX_BUF, tx_fifo_len);
            // HAL_UART_Transmit_DMA(&USART_Idx, UART_TX_BUF, tx_fifo_len);
            xSemaphoreGiveFromISR(uart_rx_finished_sem, pdFALSE);
            // osSemaphoreRelease(uart_rx_finished_sem);
        }
        // else if(huart->RxEventType == HAL_UART_RXEVENT_HT)
        // {
        //     HAL_UART_Transmit(&USART_Idx, "1\n", 2, 0xFFFF);
        // }
        // else if(huart->RxEventType == HAL_UART_RXEVENT_IDLE)
        // {
        //     HAL_UART_Transmit(&USART_Idx, "3\n", 1, 0xFFFF);
        // }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // If the DMA mode is in circular mode, we should restart the DMA transfer as previous circular transmission was blocked
    if(huart == gHuart)
    {
        if(huart->hdmarx->Init.Mode == DMA_CIRCULAR)
        {
            __HAL_UART_CLEAR_OREFLAG(huart); 
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            // HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)UART_RX_BUF, RX_BUF_SIZE);
        }
    }

}