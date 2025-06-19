#pragma once

#include "../Common/foc_types.hpp"
#include "../Common/foc_math.hpp"

namespace iFOC::Sense
{
template<uint8_t shunt_count>
class CurrSenseBase
{
    OVERRIDE_NEW();
public:
    CurrSenseBase(float gain, float shunt_ohm)
    {
        if(gain > 0.0f && shunt_ohm > 0.0f)
            current_factor = (float)(1.0f / (gain * (shunt_ohm * 1000.0f)));
        else misconfigured_area |= to_underlying(DataType::Base::MisconfiguredArea::CURR_SENSE_INIT_FACTOR_OUT_OF_RANGE);
    }
    ~CurrSenseBase() = default;
    CurrSenseBase(const CurrSenseBase<shunt_count>&) = delete;
    CurrSenseBase(CurrSenseBase<shunt_count>&&) = delete;
    // float shunt_values[shunt_count] = {0.0f};
    std::array<real_t, shunt_count> shunt_values;
    virtual void Update(float Ts) = 0;
    virtual void UpdateRemainingCurrent(float Ts) {};
    virtual void Calibrate() {};
    virtual bool IsCalibrated() { return false; }
protected:
    float current_factor = 0.0f;
};

template<typename T>
concept FOCCurrSenseImpl = std::is_base_of<CurrSenseBase<3>, T>::value;

template<typename T>
concept DCCurrSenseImpl = std::is_base_of<CurrSenseBase<1>, T>::value;
}