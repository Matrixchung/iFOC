#ifndef _I2C_HANDLER_H
#define _I2C_HANDLER_H

#include "main.h"
// #include "i2c.h"

#define I2C_WAIT_TIMEOUT 500 // in us

// void I2C_LL_Write(I2C_TypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C_Soft_Write(uint8_t addr, uint8_t reg, uint8_t data);

#endif