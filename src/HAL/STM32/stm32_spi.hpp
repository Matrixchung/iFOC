#pragma once

#include "../../Common/Interface/spi_base.hpp"
#include "../../Common/Interface/gpio_base.hpp"
#include "main.h"

#if defined(HAL_SPI_MODULE_ENABLED)

namespace iFOC::HAL
{
class STM32SPI final : public SPIBase
{
public:
    STM32SPI(SPI_HandleTypeDef* _hspi, GPIOBase& _cs);
    STM32SPI(SPI_HandleTypeDef* _hspi, GPIOBase* _cs);
    FuncRetCode Init() final;
    FuncRetCode WriteBytes(const uint8_t* data, const uint16_t size) final;
    FuncRetCode ReadBytes(uint8_t* data, const uint16_t size) final;
    FuncRetCode WriteReadBytes(const uint8_t* write_data,
                               uint8_t* read_data,
                               const uint16_t size) final;
    void SetDataWidth(DataWidth w) final;
    void SetClock(uint32_t clock) final;
    void SetCPOLCPHA(uint8_t cpol, uint8_t cpha) final;
private:
    SPI_HandleTypeDef *hspi;
    GPIOBase& cs;
};
}

#endif