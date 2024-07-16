#ifndef _FOC_DRIVER_PWM_BASE_H
#define _FOC_DRIVER_PWM_BASE_H

#include "driver_base.hpp"

template <class T>
class DriverPWMBase : public DriverBase<DriverPWMBase<T>>
{
public:
    inline bool Init(bool initCNT) { return static_cast<T*>(this)->PortTIMInit(initCNT); };
    inline void SetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3) { static_cast<T*>(this)->PortSetOutputRaw(ch_1, ch_2, ch_3); };
    inline void SetLSIdleState(uint8_t state) { static_cast<T*>(this)->SetLSIdleState(state); };
    inline void EnableOutput() { static_cast<T*>(this)->EnableOutput(); };
    inline void DisableOutput() { static_cast<T*>(this)->DisableOutput(); };
};

#endif