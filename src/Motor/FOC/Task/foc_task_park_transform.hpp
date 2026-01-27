#pragma once

#include "../foc_motor.hpp"

namespace iFOC
{
class ParkTransformTask final : public Task
{
public:
    ParkTransformTask();
    void UpdateRT(float Ts) override;
private:
    uint8_t overcurrent_tick = 0;
};
}