#pragma once

#include "dc_driver_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED) && defined(GPIO_MODULE_ENABLED)

#include "gpio_base.hpp"

namespace iFOC::Driver
{
class DCDriver1PWM final : public DCDriverBase
{
public:
    DCDriver1PWM(tmr_type *_htim, tmr_channel_select_type _ch);
    DCDriver1PWM(tmr_type *_htim, tmr_channel_select_type _ch, HAL::GPIOBase* _dir);
    DCDriver1PWM() = delete;
    FuncRetCode Init(bool initTIM) override;
    __fast_inline void EnableAllOutputs() override { tmr_channel_enable(htim, channel, TRUE); }
    __fast_inline void DisableAllOutputs() override { tmr_channel_enable(htim, channel, FALSE); }
    void SetOutputRaw(uint32_t ch, uint8_t dir) override;
private:
    tmr_type *htim;
    tmr_channel_select_type channel;
    HAL::GPIOBase* dir_gpio;
};
}

#endif