#pragma once

#include "../../Common/Interface/gpio_base.hpp"
#include "hal_const.h"

#if defined(GD32_ENV)

#if defined(GD32G5X3)

namespace iFOC::HAL
{
    class GPIO final : public GPIOBase
    {
        OVERRIDE_NEW();
        DELETE_COPY_CONSTRUCTOR(GPIO);
    public:
        GPIO(uint32_t _port, uint16_t _pin) : port(_port), pin(_pin) {};
        __fast_inline void Set() override;
        __fast_inline void Clear() override;
        __fast_inline void Write(bool val) override;
        [[nodiscard]] __fast_inline bool Read() const override;
        __fast_inline void Toggle() override;
    protected:
        __fast_inline void SetMode(GPIOMode mode) override;
        __fast_inline void SetPull(GPIOPull pull) override;
    private:
        uint32_t port;
        uint16_t pin;
    };

    inline void GPIO::Set()
    {
        GPIO_BOP(port) = (uint32_t)pin;
    }

    inline void GPIO::Clear()
    {
        GPIO_BC(port) = (uint32_t)pin;
    }

    inline void GPIO::Write(bool val)
    {
        if(val) Set();
        else Clear();
    }

    inline bool GPIO::Read() const
    {
        return (GPIO_ISTAT(port) & (uint32_t)pin);
    }

    inline void GPIO::Toggle()
    {
        GPIO_TG(port) = (uint32_t)pin;
    }

    inline void GPIO::SetMode(GPIOMode mode)
    {
        switch(mode)
        {
            case GPIOMode::OUTPUT_PUSHPULL:
            {
                gpio_mode_set(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
                GPIO_OMODE(port) &= (uint32_t)(~pin); // set pushpull
                break;
            }
            case GPIOMode::OUTPUT_OPENDRAIN:
            {
                gpio_mode_set(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
                GPIO_OMODE(port) |= (uint32_t)(pin); // set opendrain
                break;
            }
            case GPIOMode::ALTERNATE_MODE:
            {
                gpio_mode_set(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
                break;
            }
            case GPIOMode::INPUT:
            {
                gpio_mode_set(port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, pin);
                break;
            }
            case GPIOMode::ANALOG:
            {
                gpio_mode_set(port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, pin);
                break;
            }
            default: break;
        }
    }

    inline void GPIO::SetPull(GPIOPull pull)
    {
        uint32_t pull_up_down = GPIO_PUPD_NONE;
        if(pull == GPIOPull::PULL_UP) pull_up_down = GPIO_PUPD_PULLUP;
        else if(pull == GPIOPull::PULL_DOWN) pull_up_down = GPIO_PUPD_PULLDOWN;
        uint32_t pupd = GPIO_PUD(port);
        for(uint16_t i = 0; i < 16; i++)
        {
            if((1U << i) & pin)
            {
                pupd &= ~GPIO_PUPD_MASK(i);
                pupd |= GPIO_PUPD_SET(i, pull_up_down);
                break; // GPIO instance is for single pin
            }
        }
        GPIO_PUD(port) = pupd;
    }
}
#endif

#endif