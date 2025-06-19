#pragma once

#include "../../Common/Interface/gpio_base.hpp"
#include "main.h"

#if defined(HAL_GPIO_MODULE_ENABLED)

namespace iFOC::HAL
{
    class STM32GPIO final : public GPIOBase
    {
    public:
        STM32GPIO(GPIO_TypeDef *_port, uint32_t _pin) : port(_port), pin(_pin) {};
        __fast_inline void Set() final;
        __fast_inline void Clear() final;
        __fast_inline void Write(bool val) final;
        [[nodiscard]] __fast_inline bool Read() const final;
        __fast_inline void Toggle() final;
    protected:
        __fast_inline void SetMode(GPIOMode mode) final;
        __fast_inline void SetPull(GPIOPull pull) final;
    private:
        GPIO_TypeDef *port;
        uint32_t pin;
    };

    __fast_inline void STM32GPIO::Set()
    {
        LL_GPIO_SetOutputPin(port, pin);
    }

    __fast_inline void STM32GPIO::Clear()
    {
        LL_GPIO_ResetOutputPin(port, pin);
    }

    __fast_inline void STM32GPIO::Write(const bool val)
    {
        if(val) Set();
        else Clear();
    }

    __fast_inline bool STM32GPIO::Read() const
    {
        return LL_GPIO_IsInputPinSet(port, pin);
    }

    __fast_inline void STM32GPIO::Toggle()
    {
        LL_GPIO_TogglePin(port, pin);
    }

    __fast_inline void STM32GPIO::SetMode(const GPIOBase::GPIOMode mode)
    {
        switch(mode)
        {
            case GPIOBase::GPIOMode::OUTPUT_PUSHPULL:
            {
                LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_OUTPUT);
                LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_PUSHPULL);
                break;
            }
            case GPIOBase::GPIOMode::OUTPUT_OPENDRAIN:
            {
                LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_OUTPUT);
                LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_OPENDRAIN);
                break;
            }
            case GPIOBase::GPIOMode::ALTERNATE_MODE:
            {
                LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ALTERNATE);
                break;
            }
            case GPIOBase::GPIOMode::INPUT:
            {
                LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_INPUT);
                break;
            }
            case GPIOBase::GPIOMode::ANALOG:
            {
                LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ANALOG);
                break;
            }
                __builtin_unreachable();
        }
    }
    __fast_inline void STM32GPIO::SetPull(const GPIOPull pull)
    {
#if defined(STM32F1)
        if(pull == GPIOBase::GPIOPull::PULL_NO) ;
#else
        if(pull == GPIOBase::GPIOPull::PULL_NO) LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_NO);
#endif
        else if(pull == GPIOBase::GPIOPull::PULL_UP) LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_UP);
        else LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_DOWN);
    }
}

#endif