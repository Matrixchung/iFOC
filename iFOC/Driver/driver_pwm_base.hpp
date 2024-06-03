#ifndef _FOC_DRIVER_PWM_BASE_H
#define _FOC_DRIVER_PWM_BASE_H

#include "driver_base.hpp"

template <typename T>
class DriverPWMBase : public DriverBase<DriverPWMBase<T>>
{
public:
    inline bool Init() { return static_cast<T*>(this)->PortTIMInit(); };
    inline void SetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3) { static_cast<T*>(this)->PortSetOutputRaw(ch_1, ch_2, ch_3); };
    inline void DriverSetLSIdleState(uint8_t state) { static_cast<T*>(this)->SetLSIdleState(state); };
};

#endif