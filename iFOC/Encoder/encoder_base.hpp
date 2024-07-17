#ifndef _FOC_ENCODER_BASE_H
#define _FOC_ENCODER_BASE_H

#include "foc_header.h"
#include "foc_math.h"
#include "sliding_filter.h"
#include "lowpass_filter.h"

class EncoderBase
{
public:
    float single_round_angle = 0.0f;
    float raw_angle = 0.0f;
    // float calibrated_angle = 0.0f;
    float velocity = 0.0f;
    int full_rotations = 0;
    // int8_t direction = FOC_DIR_POS;
    virtual bool Init() = 0;
    virtual void Update(float Ts) = 0;
    virtual void UpdateMidInterval(float Ts) = 0;
    virtual bool IsCalibrated() = 0;
    virtual bool IsError() {return false;};
};

#endif