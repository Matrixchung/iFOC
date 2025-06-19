#include "stm32_spi.hpp"
#if defined(HAL_SPI_MODULE_ENABLED)

#include "stm32_gpio.hpp"
#include "hal_impl.hpp"

namespace iFOC::HAL
{
STM32SPI::STM32SPI(SPI_HandleTypeDef* _hspi, GPIOBase& _cs) : hspi(_hspi), cs(_cs) {};
STM32SPI::STM32SPI(SPI_HandleTypeDef* _hspi, GPIOBase* _cs) : hspi(_hspi), cs(*_cs) {};

#if defined(STM32G4)
HAL_StatusTypeDef my_HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout);
#endif

FuncRetCode STM32SPI::Init()
{
    cs.ModeOutPP();
    cs = 1;
    HAL_SPI_DeInit(hspi);
    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    if(HAL_SPI_Init(hspi) != HAL_OK) return FuncRetCode::HARDWARE_ERROR;
    return FuncRetCode::OK;
}

FuncRetCode STM32SPI::WriteBytes(const uint8_t *data, uint16_t size)
{
    cs = 0;
    const auto ret = HAL_SPI_Transmit(hspi, data, size, 0xF);
    cs = 1;
    if(ret == HAL_OK) return FuncRetCode::OK;
    else if(ret == HAL_TIMEOUT) return FuncRetCode::REMOTE_TIMEOUT;
    else if(ret == HAL_BUSY) return FuncRetCode::BUSY;
    return FuncRetCode::HARDWARE_ERROR;
}

FuncRetCode STM32SPI::ReadBytes(uint8_t *data, uint16_t size)
{
    cs = 0;
    const auto ret = HAL_SPI_Receive(hspi, data, size, 0xF);
    cs = 1;
    if(ret == HAL_OK) return FuncRetCode::OK;
    else if(ret == HAL_TIMEOUT) return FuncRetCode::REMOTE_TIMEOUT;
    else if(ret == HAL_BUSY) return FuncRetCode::BUSY;
    return FuncRetCode::HARDWARE_ERROR;
}

FuncRetCode STM32SPI::WriteReadBytes(const uint8_t *write_data, uint8_t *read_data, uint16_t size)
{
    cs = 0;
#if defined(STM32G4) // platform-specific optimization
    const auto ret = my_HAL_SPI_TransmitReceive(hspi, write_data, read_data, size, 0xF);
#else
    const auto ret = HAL_SPI_TransmitReceive(hspi, write_data, read_data, size, 0xF);
#endif
    cs = 1;
    if(ret == HAL_OK) return FuncRetCode::OK;
    else if(ret == HAL_TIMEOUT) return FuncRetCode::REMOTE_TIMEOUT;
    else if(ret == HAL_BUSY) return FuncRetCode::BUSY;
    return FuncRetCode::HARDWARE_ERROR;
}


void STM32SPI::SetDataWidth(SPIBase::DataWidth w)
{
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    if(w == SPIBase::DataWidth::HALF_WORD) hspi->Init.DataSize = SPI_DATASIZE_16BIT;
}

void STM32SPI::SetClock(const uint32_t clock)
{
#if defined(STM32G4)
    const auto spi_base_clock = HAL::GetCoreClockHz();
#endif
    assert_param(clock < spi_base_clock);
    const uint16_t divider = spi_base_clock / clock;
    if(divider <= 2) hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    else if(divider <= 4) hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    else if(divider <= 8) hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    else if(divider <= 16) hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    else if(divider <= 32) hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    else if(divider <= 64) hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    else if(divider <= 128) hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    else hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
}

void STM32SPI::SetCPOLCPHA(const uint8_t cpol, const uint8_t cpha)
{
    if(cpol == 1) hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
    else hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    if(cpha == 1) hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    else hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
}

#if defined(STM32G4)
HAL_StatusTypeDef my_HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout)
{
    // uint32_t             tmp_mode;
    uint32_t             tickstart = HAL_GetTick();

    /* Variable used to alternate Rx and Tx during transfer */
    uint8_t             txallowed = 1;

    /* Init temporary variables */
    // tmp_mode            = hspi->Init.Mode;

    // if (!((hspi->State == HAL_SPI_STATE_READY) || \
    // ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) &&
    //  (hspi->State == HAL_SPI_STATE_BUSY_RX))))
    if(!(hspi->State == HAL_SPI_STATE_READY || hspi->State == HAL_SPI_STATE_BUSY_RX))
    {
        return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX) hspi->State = HAL_SPI_STATE_BUSY_TX_RX;

    /* Set the transaction information */
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferCount = Size;
    hspi->RxXferSize  = Size;
    hspi->pTxBuffPtr  = (const uint8_t *)pTxData;
    hspi->TxXferCount = Size;
    hspi->TxXferSize  = Size;

    /* Set the Rx Fifo threshold */
    if(Size > 1U)
    {
        /* Set fiforxthreshold according the reception data length: 16bit */
        CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
    }
    else
    {
        /* Set fiforxthreshold according the reception data length: 8bit */
        SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
    }

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) __HAL_SPI_ENABLE(hspi); // Enable SPI peripheral

    /* Transmit and Receive data in 8 Bit mode */
    if(Size == 1U)
    {
        if (hspi->TxXferCount > 1U)
        {
            hspi->Instance->DR = *((const uint16_t *)hspi->pTxBuffPtr);
            hspi->pTxBuffPtr += sizeof(uint16_t);
            hspi->TxXferCount -= 2U;
        }
        else
        {
            *(__IO uint8_t *)&hspi->Instance->DR = *((const uint8_t *)hspi->pTxBuffPtr++);
            // hspi->pTxBuffPtr++;
            hspi->TxXferCount--;
        }
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
    {
        /* Check TXE flag */
        if ((txallowed == 1) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U))
        {
            if (hspi->TxXferCount > 1U)
            {
                hspi->Instance->DR = *((const uint16_t *)hspi->pTxBuffPtr);
                hspi->pTxBuffPtr += sizeof(uint16_t);
                hspi->TxXferCount -= 2U;
            }
            else
            {
                *(__IO uint8_t *)&hspi->Instance->DR = *((const uint8_t *)hspi->pTxBuffPtr++);
                // hspi->pTxBuffPtr++;
                hspi->TxXferCount--;
            }
            /* Next Data is a reception (Rx). Tx not allowed */
            txallowed = 0;
        }

        /* Wait until RXNE flag is reset */
        if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U))
        {
            if (hspi->RxXferCount > 1U)
            {
                *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
                hspi->pRxBuffPtr += sizeof(uint16_t);
                hspi->RxXferCount -= 2U;
                if (hspi->RxXferCount <= 1U)
                {
                    /* Set RX Fifo threshold before to switch on 8 bit data size */
                    SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
                }
            }
            else
            {
                (*(uint8_t *)hspi->pRxBuffPtr++) = *(__IO uint8_t *)&hspi->Instance->DR;
                // hspi->pRxBuffPtr++;
                hspi->RxXferCount--;
            }
            /* Next Data is a Transmission (Tx). Tx is allowed */
            txallowed = 1;
        }
        if(HAL_GetTick() - tickstart >= Timeout)
        {
            hspi->State = HAL_SPI_STATE_READY;
            __HAL_UNLOCK(hspi);
            return HAL_TIMEOUT;
        }
    }

    tickstart = HAL_GetTick();
    while(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) != RESET)
    {
        if(HAL_GetTick() - tickstart >= Timeout)
        {
            hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
            __HAL_UNLOCK(hspi);
            return HAL_ERROR;
        }
    }

    // /* Check the end of the transaction */
    // if (SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK)
    // {
    //     hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
    //     __HAL_UNLOCK(hspi);
    //     return HAL_ERROR;
    // }


    hspi->State = HAL_SPI_STATE_READY;
    /* Unlock the process */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}
#endif

}

#endif