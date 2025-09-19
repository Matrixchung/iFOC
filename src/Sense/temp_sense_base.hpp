#pragma once

#include "../Common/foc_types.hpp"

namespace iFOC::Sense
{
class TempSenseBase
{
    OVERRIDE_NEW();
public:
    real_t temp_celsius = 0.0f;
    virtual real_t Update() = 0;
};

template<typename T>
concept TempSenseImpl = std::is_base_of<TempSenseBase, T>::value;
}