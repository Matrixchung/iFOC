#ifndef _HW_I2C_INTERFACE_H
#define _HW_I2C_INTERFACE_H

#include "global_include.h"
#include "i2c_base.h"

class HWI2C : public I2CBase
{
public:
    HWI2C(I2C_HandleTypeDef *_hi2c) : hi2c(_hi2c) {};
    void Init() override;
    uint8_t WriteBytes(uint8_t addr, uint8_t *data, uint8_t len) override; // return 1 if success, else 0
    uint8_t ReadBytes(uint8_t addr, uint8_t *data, uint8_t len) override;
    void DelayMs(uint32_t ms) override { delay_ms(ms); };
private:
    I2C_HandleTypeDef *hi2c;
};

void HWI2C::Init()
{
    
}

uint8_t HWI2C::WriteBytes(uint8_t addr, uint8_t *data, uint8_t len)
{
    // addr &= ~0x01; // R/W bit
    __disable_irq();
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr << 1), data, len, 0xFFF);
    __enable_irq();
    return ret == HAL_OK;
}

uint8_t HWI2C::ReadBytes(uint8_t addr, uint8_t *data, uint8_t len)
{
    __disable_irq();
    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(hi2c, (uint16_t)(addr << 1), data, len, 0xFFF);
    __enable_irq();
    return ret == HAL_OK;
}

#endif