#pragma once

#include "../foc_types.hpp"
#include "../../DataType/Headers/Base/motor_error.h"
#include "../../DataType/Headers/Base/motor_state.h"
#include "../../DataType/Headers/Base/motor_control_mode.h"

namespace iFOC::HAL
{
class IndicatorBase
{
    OVERRIDE_NEW();
protected:
    using Error = std::underlying_type_t<MotorError>;
    using MotorState = iFOC::DataType::Base::MotorState;
    using MotorControlMode = iFOC::DataType::Base::MotorControlMode;
public:
    virtual FuncRetCode Init() { return FuncRetCode::OK; };
    virtual void SetRGB(uint8_t r, uint8_t g, uint8_t b) {};
    virtual void Update(uint8_t motor_id, Error error, MotorState state, MotorControlMode control_mode) = 0;
};

template<typename T>
concept IndicatorImpl = std::is_base_of<IndicatorBase, T>::value;
}
