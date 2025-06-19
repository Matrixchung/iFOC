#pragma once

#include "../foc_motor.hpp"

namespace iFOC::FOC
{
class EncoderArbiterTask final : public Task
{
public:
    EncoderArbiterTask();
    void UpdateRT(float Ts) final;
};
}