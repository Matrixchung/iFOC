#ifndef _FOC_LIMITER_H
#define _FOC_LIMITER_H

#include "foc_header.h"
#include "gpio_base.h"

#define LIMITER_DEBOUNCE_COUNT 5 // 200Hz, w = 2*pi*f = 400pirad/s, rpm = 12000RPM

template<class T = GPIOBase>
class Limiter
{
public:
    Limiter(T& _gpio, bool en_state, int8_t _dir): gpio(_gpio), enable_state(en_state), direction(_dir) {};
    bool IsActivated() {return state;};
    bool HasActivated() {return latched_state;};
    bool Update();
    int8_t direction = FOC_DIR_POS; // direction * home_speed = real_speed, it indicates which direction the motor should spin to meet the limit switch
private:
    T& gpio;
    bool state = false;
    bool latched_state = false;
    bool enable_state = true;
    uint16_t debounce_counter = 0;
};

template<class T>
bool Limiter<T>::Update()
{
    if(gpio.ReadState() == enable_state)
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