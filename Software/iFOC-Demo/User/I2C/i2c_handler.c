#include "i2c_handler.h"

void I2C_LL_Write(I2C_TypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t data)
{
    LL_I2C_GenerateStartCondition(hi2c);
    while(LL_I2C_IsActiveFlag_SB(hi2c) != SET);
    LL_I2C_TransmitData8(hi2c, addr);
    while(LL_I2C_IsActiveFlag_ADDR(hi2c) != SET);
    LL_I2C_ClearFlag_ADDR(hi2c);
    while(LL_I2C_IsActiveFlag_TXE(hi2c) != SET);
    LL_I2C_TransmitData8(hi2c, reg);
    while(LL_I2C_IsActiveFlag_TXE(hi2c) != SET);
    LL_I2C_TransmitData8(hi2c, data);
    while(LL_I2C_IsActiveFlag_BTF(hi2c) != SET);
    LL_I2C_GenerateStopCondition(hi2c);
}

void _I2C_Soft_WriteByte(uint8_t data)
{
    LL_GPIO_ResetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
    for(uint8_t i = 0; i < 8; i++)
    {
        if(((data&0x80)>>7)) LL_GPIO_SetOutputPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin);
        else LL_GPIO_ResetOutputPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin);
        delay_us(1);
        LL_GPIO_SetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
        delay_us(1);
        LL_GPIO_ResetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
        data <<= 1;
    }
}
static inline void _I2C_Stop()
{
    LL_GPIO_ResetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
    LL_GPIO_ResetOutputPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin);
    delay_us(1);
    LL_GPIO_SetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
    LL_GPIO_SetOutputPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin);
    delay_us(1);
}
uint8_t _I2C_Soft_WaitAck()
{
    LL_GPIO_SetPinMode(OLED_SDA_GPIO_Port, OLED_SDA_Pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
    delay_us(1);
    uint16_t ackTimeout = 0;
    while(LL_GPIO_IsInputPinSet(OLED_SDA_GPIO_Port, OLED_SDA_Pin))
    {
        ackTimeout++;
        delay_us(1);
        if(ackTimeout > 500)
        {
            _I2C_Stop();
            return 1;
        }
    }
    LL_GPIO_ResetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
    LL_GPIO_SetPinMode(OLED_SDA_GPIO_Port, OLED_SDA_Pin, LL_GPIO_MODE_OUTPUT);
    return 0;
}

uint8_t I2C_Soft_Write(uint8_t addr, uint8_t reg, uint8_t data)
{
    // I2C Start
    LL_GPIO_SetOutputPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin);
    LL_GPIO_SetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);
    delay_us(1);
    LL_GPIO_ResetOutputPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin);
    delay_us(1);
    LL_GPIO_ResetOutputPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin);

    // Write address first
    _I2C_Soft_WriteByte(addr);
    if(_I2C_Soft_WaitAck()) return 1;

    // Write cmd/data register
    _I2C_Soft_WriteByte(reg);
    if(_I2C_Soft_WaitAck()) return 1;

    // Write data packet
    _I2C_Soft_WriteByte(data);
    if(_I2C_Soft_WaitAck()) return 1;

    // I2C Stop
    _I2C_Stop();

    return 0;
}

