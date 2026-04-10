#pragma once

#include "indicator_base.hpp"

namespace iFOC::HAL
{
class IndicatorRGB : public IndicatorBase
{
    OVERRIDE_NEW();
public:
    void Update(uint8_t motor_id, Error error, MotorState state, MotorControlMode control_mode) final;
    void SetRGB(uint8_t r, uint8_t g, uint8_t b) final
    {
        SetBrightness(0.8f);
        SetColor(0, r, g, b);
        Update();
    }
    virtual void Update() {}
    virtual void SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual void SetBrightness(float _brightness) {}
private:
    uint8_t timer_blink_tick = 0;
    uint8_t timer_tick = 0;
    uint8_t color_wheel_i = 170;
    bool gradient_flag = false;
};
}