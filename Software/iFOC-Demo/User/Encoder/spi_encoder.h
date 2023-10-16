#ifndef _SPI_ENCODER_H
#define _SPI_ENCODER_H
#include "main.h"
#include "delay.h"
// CS Pin should connect to GPIO
// 16 Bit SPI Mode
// use AS5048A Read-only mode (SOFT_SPI)
// Hardware SPI doesn't supply Read-Only mode
// SPI_CS LOW; SPI_READ_DATA; SPI_CS HIGH;
#define AS5048A_CMD_READ_NOP      0xC000
#define AS5048A_CMD_READ_CERRFLAG 0x4001
#define AS5048A_CMD_READ_ANGLE    0xFFFF

#define AS5048A_MAX_TIMEOUT 0xFF
#define AS5048A_DIR -1

#define AS5048A_CS(state) (state ? LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin) : LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin))
#define AS5048A_SCK(state) (state ? LL_GPIO_SetOutputPin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin) : LL_GPIO_ResetOutputPin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin))
#define AS5048A_READ_MISO() (LL_GPIO_IsInputPinSet(SPI1_MISO_GPIO_Port, SPI1_MISO_Pin))

#define _1div16384 0.00006103515625f
#define _2PI 6.283185307179586476925286766559f
#define pi180 57.295779513082320876798154814105f
#define ROTATION_CHECK_THRESHOLD (0.8f * _2PI)

extern uint16_t spi_encoder_raw_angle;
extern uint8_t spi_encoder_crc_bit;
extern uint8_t spi_encoder_error_bit;
extern float spi_encoder_rad;
extern int spi_encoder_full_rotations;

void SPI_Encoder_Init(void);
// uint8_t SPI_Encoder_ReadReg16(); // Used in read-only mode, internally
void SPI_Encoder_UpdateAngle(void);
float SPI_Encoder_GetVelocity(void);
float SPI_Encoder_GetRawRad(void);
float SPI_Encoder_GetRawAngle(void);
float SPI_Encoder_GetAngle(void);
#endif