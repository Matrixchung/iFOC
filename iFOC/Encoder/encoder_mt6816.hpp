#ifndef _FOC_ENCODER_MT6816_H
#define _FOC_ENCODER_MT6816_H

#include "encoder_base.hpp"
#include "spi_base.h"

template<SPIImpl T>
class EncoderMT6816 : public EncoderBase
{
public:
    explicit EncoderMT6816(T& _spi) : spi(_spi) {};
    bool Init() final;
    void Update(float Ts) final;
    void UpdateMidInterval(float Ts) final {};
    bool IsCalibrated() final { return true; };
private:
    T& spi;
};

template<SPIImpl T>
bool EncoderMT6816<T>::Init()
{
    spi.ResetCS();
}

template<SPIImpl T>
void EncoderMT6816<T>::Update(float Ts)
{
    uint8_t tempRet[4] = {0x00};
    uint8_t tempCmd[4] = {0x83, 0xFF, 0x84, 0xFF};
    spi.SetCS();
    spi.ReadWriteBytes(tempCmd, tempRet, 4);
    spi.ResetCS();
    uint8_t PC = (tempRet[3] & 0x01);
    uint16_t raw = (tempRet[1] << 8) | (tempRet[3]);
    uint8_t parity = 0;
    for(uint8_t i = 0; i < 15; i++)
    {
        if(raw & (1 << i)) parity ^= 1;
    }
    if(PC == parity && !(tempRet[3] & (1 << 1))) single_round_angle = (float)(((float)(tempRet[1] * 256 + tempRet[3]) / 4.0f) / 16384.0f) * PI2;
}



#endif