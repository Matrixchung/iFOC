#ifndef _FOC_MODULE_INDICATOR_BASE_HPP
#define _FOC_MODULE_INDICATOR_BASE_HPP

#include "gpio_base.h"

template<class T = GPIOBase>
class Indicator
{
public:
    Indicator(T& _gpio, bool en_state) : gpio(_gpio), enable_state(en_state) 
    {
        static_assert(std::is_base_of<GPIOBase, T>::value, "GPIO must be derived from GPIOBase");
    };
    Indicator(T& _gpio) : Indicator(_gpio, true) {};
    void SetState(bool state)
    {
        if(state) gpio.SetState(enable_state);
        else gpio.SetState(!enable_state);
    }
    void Toggle()
    {
        gpio.ToggleState();
    }
private:
    T& gpio;
    bool enable_state = true;
};

#endif