#pragma once

#include "indicator_base.hpp"

namespace iFOC::HAL
{
class IndicatorLEDBase : public IndicatorBase
{
public:
    void Update(uint8_t motor_id, Error error, MotorState state, MotorControlMode control_mode) final;
    virtual void SetBrightness(float pct) = 0;
};
}