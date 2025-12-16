#pragma once

#include "../../Common/Interface/spi_base.hpp"
#include "../../Common/Interface/gpio_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(SPI_MODULE_ENABLED)

namespace iFOC::HAL
{
class SPI final : public SPIBase
{
public:
    SPI(spi_type* _hspi, GPIOBase& _cs);
    SPI(spi_type* _hspi, GPIOBase* _cs);
    FuncRetCode Init() override;
    FuncRetCode WriteBytes(const uint8_t* data, const uint16_t size) override;
    FuncRetCode ReadBytes(uint8_t* data, const uint16_t size) override;
    FuncRetCode WriteReadBytes(const uint8_t* write_data,
                               uint8_t* read_data,
                               const uint16_t size) override;
    void SetDataWidth(DataWidth w) override;
    void SetClock(uint32_t clock) override;
    void SetCPOLCPHA(uint8_t cpol, uint8_t cpha) override;
    void SetCS(bool state) override;
private:
    spi_type* hspi;
    GPIOBase& cs;
    spi_init_type spi_init_struct{};
};
}

#endif