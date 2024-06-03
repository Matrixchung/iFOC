#include "sogi.h"
SOGI::SOGI(float frequency, float _K)
{
    K = _K;
    omega = 2.0f * PI * frequency;
    _2komega = 2.0f * K * omega;
    _omegapow2 = omega * omega;
    _omega0p5 = 0.5f * omega;
}

qd_t* SOGI::GetOutput(float input, float Ts)
{
    float x = _2komega * Ts;
    float y = _omegapow2 * Ts * Ts;
    float lambda = _omega0p5 * Ts;
    float a = x / (x + y + 4.0f);
    float b = (8.0f - 2.0f * y) / (x + y + 4.0f);
    float c = (x - y - 4.0f) / (x + y + 4.0f);
    output.d = a * input - a * u_2 + b * d_1 + c * d_2;
    output.q = b * q_1 + c * q_2 + lambda * a * (input + 2.0f * u_1 + u_2);
    u_2 = u_1;
    u_1 = input;
    d_2 = d_1;
    d_1 = output.d;
    q_2 = q_1;
    q_1 = output.q;
    return &output;
}