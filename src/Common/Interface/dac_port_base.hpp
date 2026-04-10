#pragma once

#include <cstdint>
#include "../foc_types.hpp"

namespace iFOC::HAL
{
class DACPortBase
{
protected:
    ~DACPortBase() = default;
    OVERRIDE_NEW();
public:
    DACPortBase() = default;
    DACPortBase(const DACPortBase&) = delete;
    DACPortBase(DACPortBase&&) = delete;
    virtual void Init() {}
    virtual void SetOutput_mV(real_t voltage) = 0;
};

template<typename T>
concept DACPortImpl = std::is_base_of<DACPortBase, T>::value;
}