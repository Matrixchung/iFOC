#pragma once

#include "hal_const.h"

#if defined(GD32_ENV)

#include "../../Motor/FOC/foc_driver_base.hpp"

namespace iFOC::Driver
{
class FOCDriver6PWM : public FOCDriverBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(FOCDriver6PWM);
public:
    explicit FOCDriver6PWM(uint32_t _htim);
    FOCDriver6PWM() = delete;
    FuncRetCode Init(bool initCNT) override;
    void SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3) override;
#if defined(GD32G5X3)
    __fast_inline void EnableAllOutputs() override { TIMER_CCHP0(htim) |= (uint32_t)TIMER_CCHP0_POEN; };
    __fast_inline void DisableAllOutputs() override { TIMER_CCHP0(htim) &= (~(uint32_t)TIMER_CCHP0_POEN); };
#endif
    void EnableBridge(Bridge bridge) override;
    void DisableBridge(Bridge bridge) override;
    real_t GetDeadTime() override;
private:
    uint32_t htim;
};

}

#endif