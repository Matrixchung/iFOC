#ifndef _FOC_ENCODER_AS5600_BASE_H
#define _FOC_ENCODER_AS5600_BASE_H

#include "encoder_base.h"

class EncoderAS5600Base : public EncoderBase
{
public:
    bool Init(float max_vel) override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override;
private:
    
};

#endif