#pragma once

#include "../foc_motor.hpp"

namespace iFOC
{
class UpdateSenseTask final : public Task
{
public:
    UpdateSenseTask();
    void UpdateRT(float Ts) final;
    void UpdateNormal() final;
private:
    uint8_t overcurrent_tick = 0;
};
}