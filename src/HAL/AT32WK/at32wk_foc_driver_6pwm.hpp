#pragma once

#include "foc_driver_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED)

namespace iFOC::Driver
{
class FOCDriver6PWM : public FOCDriverBase
{
public:
    explicit FOCDriver6PWM(tmr_type *_htim);
    FOCDriver6PWM() = delete;
    FuncRetCode Init(bool initCNT) override;
    __fast_inline void SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3) override;
    __fast_inline void EnableAllOutputs() override { tmr_output_enable(htim, TRUE); };
    __fast_inline void DisableAllOutputs() override { tmr_output_enable(htim, FALSE); };
    void EnableBridge(Bridge bridge) override;
    void DisableBridge(Bridge bridge) override;
    real_t GetDeadTime() override;
private:
    tmr_type *htim;
};

__fast_inline void FOCDriver6PWM::SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3)
{
    // PWM Mode2: max_compare - ch1, PWM Mode1: ch1
    // tmr_channel_value_set(htim, TMR_SELECT_CHANNEL_1, max_compare - ch1);
    // tmr_channel_value_set(htim, TMR_SELECT_CHANNEL_2, max_compare - ch2);
    // tmr_channel_value_set(htim, TMR_SELECT_CHANNEL_3, max_compare - ch3);
    htim->c1dt = max_compare - ch1;
    htim->c2dt = max_compare - ch2;
    htim->c3dt = max_compare - ch3;
}
}

#endif