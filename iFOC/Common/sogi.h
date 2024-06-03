#ifndef _FOC_SOGI_H
#define _FOC_SOGI_H

#include "foc_header.h"
#include "foc_math.h"

#define SOGI_COMMON_K 1.4142135623730950488016887242097f

class SOGI
{
public:
    SOGI(float frequency, float _K);
    qd_t* GetOutput(float input, float Ts);
    qd_t output = {.q = 0.0f, .d = 0.0f};
private:
    float K = SOGI_COMMON_K;
    float omega = 0.0f;
    float _2komega = 0.0f;
    float _omegapow2 = 0.0f;
    float _omega0p5 = 0.0f;
    float u_2 = 0.0f;
    float u_1 = 0.0f;
    float q_2 = 0.0f;
    float q_1 = 0.0f;
    float d_2 = 0.0f;
    float d_1 = 0.0f;
};

#endif