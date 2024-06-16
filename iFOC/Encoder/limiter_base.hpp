#ifndef _FOC_LIMITER_BASE_H
#define _FOC_LIMITER_BASE_H

#include "foc_header.h"

#define LIMITER_DEBOUNCE_COUNT 5 // 200Hz, w = 2*pi*f = 400pirad/s, rpm = 12000RPM

class LimiterBase
{
public:
    LimiterBase(int8_t _dir): direction(_dir) {};
    bool IsActivated() {return state;};
    bool HasActivated() {return latched_state;};
    bool Update();
    // virtual void PortActiveIRQ() = 0; // use IRQ
    virtual bool PortGetLimiterState() = 0; // use poll
    int8_t direction = FOC_DIR_POS; // direction * home_speed = real_speed, it indicates which direction the motor should spin to meet the limit switch
protected:
    bool state = false;
    bool latched_state = false;
    uint16_t debounce_counter = 0;
};

bool LimiterBase::Update()
{
    if(PortGetLimiterState())
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
    return state;
}

#endif