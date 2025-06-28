#include "stm32_uart.hpp"

#if defined(HAL_UART_MODULE_ENABLED)

namespace iFOC::HAL
{
STM32UART::STM32UART(UART_HandleTypeDef *_huart) : UARTBase(), huart(_huart) {}

FuncRetCode STM32UART::Init(DataType::Comm::UARTBaudrate baud)
{
    FuncRetCode ret = tx_fifo.init(TX_FIFO_BUFFER_SIZE);
    if(ret != FuncRetCode::OK) return ret;
    ret = rx_fifo.init(RX_FIFO_BUFFER_SIZE);
    if(ret != FuncRetCode::OK) return ret;
    xSemaphoreGive(tx_sem);
    uint32_t baudrate = 115200;
    switch(baud)
    {
        case DataType::Comm::UARTBaudrate::BAUD_9600: baudrate = 9600; break;
        case DataType::Comm::UARTBaudrate::BAUD_230400: baudrate = 230400; break;
        case DataType::Comm::UARTBaudrate::BAUD_460800: baudrate = 460800; break;
        case DataType::Comm::UARTBaudrate::BAUD_921600: baudrate = 921600; break;
        case DataType::Comm::UARTBaudrate::BAUD_1843200: baudrate = 1843200; break;
        default: break;
    }
    // DMA Settings
    if(huart->hdmarx)
    {
        huart->hdmarx->Init.PeriphInc = DMA_PINC_DISABLE;
        huart->hdmarx->Init.MemInc = DMA_MINC_ENABLE;
        huart->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        huart->hdmarx->Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
        huart->hdmarx->Init.Mode = DMA_CIRCULAR;
        if(HAL_DMA_Init(huart->hdmarx) != HAL_OK) return FuncRetCode::HARDWARE_ERROR;
    }
    if(huart->hdmatx)
    {
        huart->hdmatx->Init.PeriphInc = DMA_PINC_DISABLE;
        huart->hdmatx->Init.MemInc = DMA_MINC_ENABLE;
        huart->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        huart->hdmatx->Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
        huart->hdmatx->Init.Mode = DMA_NORMAL;
        if(HAL_DMA_Init(huart->hdmatx) != HAL_OK) return FuncRetCode::HARDWARE_ERROR;
    }
    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    if(HAL_UART_Init(huart) != HAL_OK) return FuncRetCode::HARDWARE_ERROR;
    tx_buffer.fill(0);
    rx_buffer.fill(0);
    event_handler.Start();
    /// If we enable UART RX DMA Channel interrupt, the IDLE event will be called on DMA HALF FULL & DMA TC.
    /// See: UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
    /// Also: HAL_DMA_Start_IT().
    if(HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer.data(), rx_buffer.max_size()) != HAL_OK) return FuncRetCode::HARDWARE_ERROR;
    return FuncRetCode::OK;
}

FuncRetCode STM32UART::StartTransmit(bool blocked)
{
    if(!blocked)
    {
        if(xSemaphoreTake(tx_sem, READ_WRITE_TIMEOUT_MS) == pdTRUE)
        {
            auto len = tx_fifo.get(tx_buffer.data(), tx_buffer.max_size());
            if(len == 0) xSemaphoreGive(tx_sem);
            else if(HAL_UART_Transmit_DMA(huart, (const uint8_t*)tx_buffer.data(), len) != HAL_OK)
            {
                // BUSY, give back semaphore
                xSemaphoreGive(tx_sem);
                return FuncRetCode::BUSY;
            }
            return FuncRetCode::OK;
        }
        return FuncRetCode::REMOTE_TIMEOUT;
    }
    else
    {
        // xSemaphoreTake(tx_fifo_mutex, portMAX_DELAY);
        auto len = tx_fifo.get(tx_buffer.data(), tx_buffer.max_size());
        // xSemaphoreGive(tx_fifo_mutex);
        if(HAL_UART_Transmit(huart, (const uint8_t*)tx_buffer.data(), len, 0xF) != HAL_OK) return FuncRetCode::BUSY;
        // CallTxCpltCallback();
    }
    return FuncRetCode::OK;
}

/// See: https://blog.csdn.net/wallace89/article/details/146610657 \n
/// Also: https://acuity.blog.csdn.net/article/details/108367512 \n
/// Note 1: If DMA is in circular mode, and TC event happened, HAL Library will set IDLE event again with
///         Size == BUFFER_SIZE.\n
/// Note 2: All single data packet size > RX_FIFO_BUFFER_SIZE will probably be truncated or get corrupted. \n
void STM32UART::OnIdleIRQ(UART_HandleTypeDef *_huart, uint16_t Size)
{
    if(_huart == huart)
    {
        switch(huart->RxEventType)
        {
            case HAL_UART_RXEVENT_TC:
            {
                uint16_t recv_size = rx_buffer.max_size() - last_dma_rx_size;
                rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
                last_dma_rx_size = 0;
                break;
            }
            case HAL_UART_RXEVENT_HT:
            {
                uint16_t recv_total_size = rx_buffer.max_size() - __HAL_DMA_GET_COUNTER(huart->hdmarx);
                uint16_t recv_size = recv_total_size - last_dma_rx_size;
                rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
                last_dma_rx_size = recv_total_size;
                // if(recv_size > 0) vTaskNotifyGiveFromISR(event_handler.GetHandle(), nullptr);
                break;
            }
            case HAL_UART_RXEVENT_IDLE:
            {
                uint16_t recv_total_size = rx_buffer.max_size() - __HAL_DMA_GET_COUNTER(huart->hdmarx);
                uint16_t recv_size = recv_total_size - last_dma_rx_size;
                rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
                last_dma_rx_size = recv_total_size;
                vTaskNotifyGiveFromISR(event_handler.GetHandle(), nullptr);
                break;
            }
            default:
            {
                __builtin_unreachable();
            }
        }
    }
}

void STM32UART::OnTxCpltIRQ(UART_HandleTypeDef *_huart)
{
    if(_huart == huart)
    {
        // xSemaphoreGiveFromISR(tx_sem, nullptr);
        if(xSemaphoreGiveFromISR(tx_sem, nullptr) != pdTRUE) xSemaphoreTakeFromISR(tx_sem, nullptr);
        // CallTxCpltCallback();
    }
}

}

#endif