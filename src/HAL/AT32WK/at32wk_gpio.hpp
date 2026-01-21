#pragma once

#include "../../Common/Interface/gpio_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV)

namespace iFOC::HAL
{
class GPIO final : public GPIOBase
{
public:
    GPIO(gpio_type *_port, uint16_t _pin) : port(_port), pin(_pin) {};
    __fast_inline void Set() override;
    __fast_inline void Clear() override;
    __fast_inline void Write(bool val) override;
    [[nodiscard]] __fast_inline bool Read() const override;
    __fast_inline void Toggle() override;
protected:
    __fast_inline void SetMode(GPIOMode mode) override;
    __fast_inline void SetPull(GPIOPull pull) override;
private:
    gpio_type *port;
    uint16_t pin;
};

inline void GPIO::Set()
{
    // gpio_bits_set(port, pin);
    port->scr = pin;
}

inline void GPIO::Clear()
{
    // gpio_bits_reset(port, pin);
    port->clr = pin;
}

inline void GPIO::Write(bool val)
{
    if(val) Set();
    else Clear();
}

inline bool GPIO::Read() const
{
    // return gpio_input_data_bit_read(port, pin);
    return pin == (pin & port->idt);
}

inline void GPIO::Toggle()
{
    gpio_bits_toggle(port, pin);
}

inline void GPIO::SetMode(GPIOMode mode)
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = pin;
    switch(mode)
    {
        case GPIOMode::OUTPUT_PUSHPULL:
        {
            gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
            gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
            gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
            gpio_init(port, &gpio_init_struct);
            break;
        }
        case GPIOMode::OUTPUT_OPENDRAIN:
        {
            gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
            gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
            gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
            gpio_init(port, &gpio_init_struct);
            break;
        }
        case GPIOMode::ALTERNATE_MODE:
        {
            gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
            gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
            gpio_init(port, &gpio_init_struct);
            break;
        }
        case GPIOMode::INPUT:
        {
            gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
            gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
            gpio_init(port, &gpio_init_struct);
            break;
        }
        case GPIOMode::ANALOG:
        {
            gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
            gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
            gpio_init(port, &gpio_init_struct);
            break;
        }
            __builtin_unreachable();
    }
}

inline void GPIO::SetPull(GPIOPull pull)
{

}
}

#endif