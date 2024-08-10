#ifndef _STM32_GPIO_HPP
#define _STM32_GPIO_HPP

#include "global_include.h"
#include "gpio_base.h"

class STM32GPIO : public GPIOBase
{
public:
    STM32GPIO(GPIO_TypeDef *_port, uint32_t _pin) : port(_port), pin(_pin) {};
    void SetState(bool state) override;
    void ToggleState() override;
    bool ReadState() override { return false; }
private:
    GPIO_TypeDef *port;
    uint32_t pin;
};

void STM32GPIO::SetState(bool state)
{
    if(state) LL_GPIO_SetOutputPin(port, pin);
    else LL_GPIO_ResetOutputPin(port, pin);
}

void STM32GPIO::ToggleState()
{
    LL_GPIO_TogglePin(port, pin);
}

#endif