#pragma once

#include <cstdint>
#include "foc_types.hpp"

namespace iFOC::HAL
{
class ADCPortBase
{
protected:
    ~ADCPortBase() = default;
    OVERRIDE_NEW();
public:
    ADCPortBase() = default;
    ADCPortBase(const ADCPortBase&) = delete;
    ADCPortBase(ADCPortBase&&) = delete;
    virtual real_t GetVoltage() { return 0.0f; }
    virtual real_t GetFullRangeVoltage() { return 0.0f; }
    virtual uint32_t GetRawValue() { return 0; }
    virtual uint32_t GetFullRange() { return 0; }
};
}