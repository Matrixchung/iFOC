#ifndef _ENCODER_AS5048A_H
#define _ENCODER_AS5048A_H

#include "encoder_as5048a_base.hpp"
#include "global_include.h"

// SPI Setting: 16 Bits, MSB, Prescaler 64(STM32F7), BR = 781.25KBits/s
// CPOL = LOW, CPHA = 2 Edge
// CS Line will interfere SCK Line, all of CS, SCK and MISO should be PULLED UP!
// the SPI line is noisy, considering using PWM output...

class EncoderAS5048A : public EncoderAS5048ABase
{
public:
    EncoderAS5048A(SPI_TypeDef *_hspi, GPIO_TypeDef *_cs_port, uint32_t _cs_pin, float _max_vel)
    :hspi(_hspi), CS_Port(_cs_port), CS_Pin(_cs_pin)
    {
        max_velocity = _max_vel;
    };
    bool PortSPIInit() override;
    void PortSetCS(uint8_t state) override;
    uint16_t PortSPIRead16(uint16_t reg, uint16_t *ret) override;
private:
    SPI_TypeDef *hspi;
    GPIO_TypeDef *CS_Port;
    uint32_t CS_Pin;
};

bool EncoderAS5048A::PortSPIInit()
{
    LL_SPI_Enable(hspi);
    LL_GPIO_SetOutputPin(CS_Port, CS_Pin);
    return true;
}

void EncoderAS5048A::PortSetCS(uint8_t state)
{
    if(state) LL_GPIO_SetOutputPin(CS_Port, CS_Pin);
    else LL_GPIO_ResetOutputPin(CS_Port, CS_Pin);
}

uint16_t EncoderAS5048A::PortSPIRead16(uint16_t reg, uint16_t *ret)
{
    return SPI_ReadReg16(hspi, reg, ret);
}

#endif