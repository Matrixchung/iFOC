#include "spi_encoder.h"
uint16_t spi_encoder_raw_angle = 0;
uint8_t spi_encoder_crc_bit = 0;
uint8_t spi_encoder_error_bit = 0;
float spi_encoder_rad = 0.0f;
int spi_encoder_full_rotations = 0;

static float _rad_prev = -1.0f; // used in rotation detection, init -1.0f
static uint32_t _vel_rad_ts = 0;
static float _vel_rad_prev = -1.0f;
static uint32_t _vel_rad_prev_ts = 0;
static int _vel_full_rotations = 0;
static float _rad_delta = 0.0f;

// 0 - success 1 - failed
// CPOL=0, CPHA=2
static uint8_t SPI_Encoder_ReadReg16(void)
{
    uint16_t ret = 0;
    AS5048A_CS(0);
    // delay_us(1);
    for(uint8_t i = 0; i < 16; i++)
    {
        AS5048A_SCK(1);
        delay_us(1);
        AS5048A_SCK(0);
        delay_us(1);
        if(AS5048A_READ_MISO()) ret |= (0x8000 >> i);
        
    }
    // delay_us(1);
    spi_encoder_crc_bit = (uint8_t)(ret & 0x8000); // [15]
    spi_encoder_error_bit = (uint8_t)(ret & 0x4000); // [14]
    spi_encoder_raw_angle = (uint16_t)(ret & 0x3FFF); // [13:0]
    AS5048A_CS(1);
    return 0;

    // Hardware SPI (buggy SCK Clock)
    // uint16_t timeout = 0;
    // // uint16_t temp = 0;
    // *result = 0;
    // AS5048A_CS(0);
    // LL_SPI_Enable(SPI1);
    // while(!LL_SPI_IsActiveFlag_RXNE(SPI1))
    // {
    //     timeout++;
    //     // delay_us(1);
    //     if(timeout > AS5048A_MAX_TIMEOUT)
    //     {
    //         *result = 0;
    //         delay_us(1);
    //         LL_SPI_Disable(SPI1);
    //         while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
    //         AS5048A_CS(1);
    //         return 1;
    //     }
    // }
    // *result = LL_SPI_ReceiveData16(SPI1);
    // // while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
    // delay_us(1);
    // LL_SPI_Disable(SPI1);
    // while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
    // AS5048A_CS(1);
    // return 0;
}


void SPI_Encoder_Init(void)
{
    SPI_Encoder_ReadReg16();
    SPI_Encoder_ReadReg16();
}

void SPI_Encoder_UpdateAngle(void)
{
    portDISABLE_INTERRUPTS();
    SPI_Encoder_ReadReg16();
    portENABLE_INTERRUPTS();
    spi_encoder_rad = AS5048A_DIR == 1 ? ((float)(spi_encoder_raw_angle) * _1div16384 * _2PI) : ((float)(16384.0f-spi_encoder_raw_angle) * _1div16384 * _2PI);
    if(_rad_prev >= 0.0f)
    {
        _rad_delta = spi_encoder_rad - _rad_prev;
        if(_rad_delta > ROTATION_CHECK_THRESHOLD) spi_encoder_full_rotations--;
        else if(_rad_delta < -ROTATION_CHECK_THRESHOLD) spi_encoder_full_rotations++;
    }
    _rad_prev = spi_encoder_rad;
}
float SPI_Encoder_GetVelocity(void)
{
    static float _last_Ts = 0.0f;
    if(_rad_prev < 0.0f) return 0.0f;
    _vel_rad_ts = get_micros();
    float Ts = (float) (_vel_rad_ts - _vel_rad_prev_ts) * 1e-6f;
    if(Ts <= 0) Ts = _last_Ts;
    float vel = ((float)(spi_encoder_full_rotations - _vel_full_rotations) * _2PI + (_rad_prev - _vel_rad_prev)) / Ts;
    _vel_rad_prev = _rad_prev;
    _vel_rad_prev_ts = _vel_rad_ts;
    _vel_full_rotations = spi_encoder_full_rotations;
    _last_Ts = Ts;
    return vel;
}
float SPI_Encoder_GetRawRad(void)
{
    SPI_Encoder_UpdateAngle();
    return spi_encoder_rad;
}

float SPI_Encoder_GetRawAngle(void)
{
    SPI_Encoder_UpdateAngle();
    return (float)(spi_encoder_rad * pi180);
}
float SPI_Encoder_GetAngle(void)
{
    return (float)(spi_encoder_full_rotations * 360.0f) + SPI_Encoder_GetRawAngle();
}
float SPI_Encoder_GetRad(void)
{
    return (float)(spi_encoder_full_rotations * _2PI) + SPI_Encoder_GetRawRad();
}