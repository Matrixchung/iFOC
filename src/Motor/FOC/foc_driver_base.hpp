#pragma once

#include "../driver_base.hpp"

namespace iFOC::Driver
{
class FOCDriverBase : public DriverBase
{
protected:
    uint32_t max_compare = 0;
public:
    enum class Bridge : uint8_t
    {
        HB_U = 1,
        LB_U = 2,
        HB_V = 3,
        LB_V = 4,
        HB_W = 5,
        LB_W = 6
    };
    virtual void SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3) = 0;
    virtual void EnableBridge(Bridge bridge) = 0;
    virtual void DisableBridge(Bridge bridge) = 0;
    virtual float GetDeadTime() { return 0.0f; }

    __fast_inline void SetOutput3CHPu(real_t pu1, real_t pu2, real_t pu3) /// input range: [0, 1]
    {
        SetOutput3CHRaw((uint32_t)(pu1 * (real_t)max_compare),
                        (uint32_t)(pu2 * (real_t)max_compare),
                        (uint32_t)(pu3 * (real_t)max_compare));
    }

    template<typename... Args>
    __fast_inline void EnableBridges(Args... args)
    {
        (..., EnableBridge(args));
    }

    template<typename... Args>
    __fast_inline void DisableBridges(Args... args)
    {
        (..., DisableBridge(args));
    }
};

template<typename T>
concept FOCDriverImpl = std::is_base_of<FOCDriverBase, T>::value;
}