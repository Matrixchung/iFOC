#ifndef _STM32_LIMITER_HPP
#define _STM32_LIMITER_HPP

#include "global_include.h"

#define LIMITER_DEBOUNCE_COUNT 10

class Limiter
{
public:
    Limiter(GPIO_TypeDef *_port, uint32_t _pin) : port(_port), pin(_pin) {};
    void Update(uint16_t tickrate);
    bool IsActivated() { return state; }
    bool HasActivated() { return latched_state; }
private:
    GPIO_TypeDef *port;
    uint32_t pin;
    bool state = false;
    bool latched_state = false;
    uint16_t debounce_counter = 0;
    uint8_t active_state = 0;
};

void Limiter::Update(uint16_t tickrate)
{
    if(LL_GPIO_IsInputPinSet(port, pin) == active_state)
    {
        if(debounce_counter >= LIMITER_DEBOUNCE_COUNT)
        {
            state = true;
            latched_state = true;
        }
        else debounce_counter++;
    }
    else
    {
        debounce_counter = 0;
        state = false;
    }
}

#endif