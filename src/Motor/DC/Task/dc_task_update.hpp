#pragma once

#include "../dc_motor.hpp"

namespace iFOC::DC
{
class UpdateTask final : public Task
{
public:
    UpdateTask();
    void UpdateRT(float Ts) override;
    void UpdateNormal() override;
};
}