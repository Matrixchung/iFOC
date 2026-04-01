#pragma once

#include "../../Common/Interface/spi_base.hpp"
#include "../../Common/Interface/gpio_base.hpp"
#include "hal_const.h"

#if defined(GD32_ENV)

namespace iFOC::HAL
{
class SPI final : public SPIBase
{
public:
    SPI(uint32_t spi_periph, GPIOBase& _cs);
    SPI(uint32_t spi_periph, GPIOBase* _cs);
    FuncRetCode Init() override;
    FuncRetCode WriteBytes(const uint8_t* data, uint16_t size) override;
    FuncRetCode ReadBytes(uint8_t* data, uint16_t size) override;
    FuncRetCode WriteReadBytes(const uint8_t* write_data,
                               uint8_t* read_data,
                               uint16_t size) override;
    void SetDataWidth(DataWidth w) override;
    void SetClock(uint32_t clock) override;
    void SetCPOLCPHA(uint8_t cpol, uint8_t cpha) override;
    void SetCS(bool state) override;
private:
    uint32_t hspi;
    GPIOBase& cs;
    spi_parameter_struct spi_init_struct{};
};
}

#endif