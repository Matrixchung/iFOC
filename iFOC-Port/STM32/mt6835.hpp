#ifndef _ENCODER_MT6835_H
#define _ENCODER_MT6835_H

#include "encoder_mt6835_base.hpp"
#include "global_include.h"

class EncoderMT6835 : public EncoderMT6835Base
{
public:
    EncoderMT6835(SPI_TypeDef *_hspi, GPIO_TypeDef *_cs_port, uint32_t _cs_pin, float _max_vel)
    :hspi(_hspi), CS_Port(_cs_port), CS_Pin(_cs_pin)
    {
        max_velocity = _max_vel;
    };
    bool PortSPIInit() override;
    void PortSetCS(uint8_t state) override;
    uint16_t PortSPIRead16(uint16_t reg, uint16_t *ret) override;
    uint16_t PortSPIRead8(uint8_t reg, uint8_t *ret) override;
private:
    SPI_TypeDef *hspi;
    GPIO_TypeDef *CS_Port;
    uint32_t CS_Pin;
};

bool EncoderMT6835::PortSPIInit()
{
    LL_SPI_Enable(hspi);
    LL_GPIO_SetOutputPin(CS_Port, CS_Pin);
    return true;
}

void EncoderMT6835::PortSetCS(uint8_t state)
{
    if(state) LL_GPIO_SetOutputPin(CS_Port, CS_Pin);
    else LL_GPIO_ResetOutputPin(CS_Port, CS_Pin);
}

uint16_t EncoderMT6835::PortSPIRead16(uint16_t reg, uint16_t *ret)
{
    return SPI_ReadReg16(hspi, reg, ret);
}

uint16_t EncoderMT6835::PortSPIRead8(uint8_t reg, uint8_t *ret)
{
    return SPI_ReadReg8(hspi, reg, ret);
}

#endif