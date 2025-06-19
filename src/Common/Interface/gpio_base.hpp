#pragma once

#include <cstdint>
#include <type_traits>
#include "foc_types.hpp"

namespace iFOC::HAL
{
class GPIOBase
{
    OVERRIDE_NEW();
protected:
    enum class GPIOMode : uint8_t
    {
        OUTPUT_PUSHPULL = 0,
        OUTPUT_OPENDRAIN = 1,
        ALTERNATE_MODE = 2,
        INPUT = 3,
        ANALOG = 4,
    };
    enum class GPIOPull : uint8_t
    {
        PULL_NO = 0,
        PULL_UP = 1,
        PULL_DOWN = 2
    };
public:
    GPIOBase() = default;
    virtual ~GPIOBase() = default;
    GPIOBase(const GPIOBase&) = delete;
    GPIOBase(GPIOBase&&) = delete;

    virtual void Set() = 0;
    virtual void Clear() = 0;
    virtual void Write(bool val) = 0;
    [[nodiscard]] virtual bool Read() const = 0;
    virtual void Toggle();

    GPIOBase & ModeOutPP();
    GPIOBase & ModeOutOD();
    GPIOBase & ModeAlter();
    GPIOBase & ModeInput();

    GPIOBase & PullUp();
    GPIOBase & PullDown();
    GPIOBase & PullNo();

    GPIOBase & operator= (bool val);
    GPIOBase & operator= (int val);
    GPIOBase & operator= (const GPIOBase & other);
    explicit operator bool() const;
protected:
    virtual void SetMode(GPIOMode mode) = 0;
    virtual void SetPull(GPIOPull pull) = 0;
};

template<typename T>
concept GPIOImpl = std::is_base_of<GPIOBase, T>::value;
}