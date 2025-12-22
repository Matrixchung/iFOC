#pragma once

#include "../driver_base.hpp"

namespace iFOC::Driver
{
class DCDriverBase : public DriverBase
{
protected:
    uint32_t max_compare = 0;
public:
    virtual void SetOutputRaw(uint32_t ch, uint8_t dir) = 0;
    // [-1.0f, 1.0f]
    __fast_inline void SetOutputPu(real_t pu)
    {
        pu = _constrain(pu, -1.0f, 1.0f);
        SetOutputRaw((uint32_t)(ABS(pu) * (real_t)max_compare), pu >= 0.0f);
    }
};

template<typename T>
concept DCDriverImpl = std::is_base_of<DCDriverBase, T>::value;
}