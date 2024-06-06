#ifndef _ENCODER_MT6835_H
#define _ENCODER_MT6835_H

#include "encoder_mt6835_base.hpp"
#include "global_include.h"

class EncoderMT6835 : public EncoderMT6835Base
{
public:
    EncoderMT6835(SPI_HandleTypeDef *_hspi, GPIO_TypeDef *_cs_port, uint32_t _cs_pin, float _max_vel)
    :hspi(_hspi), CS_Port(_cs_port), CS_Pin(_cs_pin)
    {
        max_velocity = _max_vel;
    };
    bool PortSPIInit() override;
    void PortSetCS(uint8_t state) override;
    uint16_t PortSPIRead16(uint16_t reg, uint16_t *ret) override;
    uint16_t PortSPIRead8(uint8_t reg, uint8_t *ret) override;
private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *CS_Port;
    uint32_t CS_Pin;
};

bool EncoderMT6835::PortSPIInit()
{
    // LL_SPI_Enable(hspi);
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
    uint8_t data_u8[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, (const uint8_t*)data_u8, data_u8, 2, 0xFFF);
    *ret = (uint16_t)data_u8[0] << 8 | (uint16_t)data_u8[1];
    return status == HAL_OK;
}

uint16_t EncoderMT6835::PortSPIRead8(uint8_t reg, uint8_t *ret)
{
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, (const uint8_t*)&reg, ret, 1, 0xFFF);
    return status == HAL_OK;
}

#endif