#ifndef _LIMITER_H
#define _LIMITER_H

#include "limiter_base.hpp"
#include "global_include.h"

class Limiter : public LimiterBase
{
public:
    Limiter(GPIO_TypeDef *_port, uint32_t _pin, uint8_t _active, int8_t _dir)
    : LimiterBase(_dir), active_state(_active), port(_port), pin(_pin) {};
    bool PortGetLimiterState() override;
private:
    uint8_t active_state = 1;
    GPIO_TypeDef *port;
    uint32_t pin;
};

bool Limiter::PortGetLimiterState()
{
    return LL_GPIO_IsInputPinSet(port, pin) == active_state;
}

#endif