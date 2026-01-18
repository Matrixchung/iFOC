#pragma once

#include <cstdint>
#include "../foc_types.hpp"

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
    [[nodiscard]] real_t GetVoltage() { return GetVoltage_mV() * 0.001f; }
    [[nodiscard]] virtual real_t GetVoltage_mV() { return 0.0f; }
    [[nodiscard]] virtual real_t GetFullRangeVoltage() const { return 0.0f; }
    [[nodiscard]] virtual uint32_t GetRawValue() { return 0; }
    [[nodiscard]] virtual uint32_t GetVrefRawValue() { return 0; }
    [[nodiscard]] virtual uint32_t GetFullRange() const { return 0; }
};
}