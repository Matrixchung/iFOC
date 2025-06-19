#include "stm32_foc_driver_6pwm.hpp"

#if defined(HAL_TIM_MODULE_ENABLED)

#include "../../DataType/board_config.hpp"

namespace iFOC::Driver
{
#define config iFOC::BoardConfig.GetConfig()

#if defined(STM32G4) || defined(STM32F3)
    // More about Dithering Mode: see STM32G4 RM0440
#define TIM_DITHERING_MODE_EN
#endif

FOCDriver6PWM::FOCDriver6PWM(TIM_TypeDef *_htim) : htim(_htim) {};

FuncRetCode FOCDriver6PWM::Init(bool initCNT)
{
    // Step #1: Reconfigure TIM based on config PWM frequency
    DisableAllOutputs();
    LL_TIM_DisableCounter(htim);
    auto pwm_freq = config.pwm_wave_freq();
    if(pwm_freq == 0) pwm_freq = 20000; // fallback to 20KHz
#if defined(STM32G4) // STM32G4: f(PCLK1) = f(TIMER)
    const auto timer_base_clock = HAL::GetCoreClockHz();
#endif
    uint32_t arr = (timer_base_clock / pwm_freq) / 2;
#ifdef TIM_DITHERING_MODE_EN
    arr *= 16;
#endif
    max_compare = arr;
    /*
     * In center-aligned mode, the counter counts from 0 to the autoreload value (content of the
     * TIMx_ARR register) – 1, generates a counter overflow event, then counts from the autoreload value
     * down to 1 and generates a counter underflow event. Then it restarts counting from 0. --- RM0440
     * [0 ~ ARR - 1, ARR ~ 1], [0 ~ ARR - 1, ARR ~ 1]... **ARR value should NOT subtract 1**
     */
    // arr -= 1;
    // const uint32_t rcr = 1;
    const uint32_t rcr = 0;
//     uint32_t ch4_compare = arr;
// #ifdef TIM_DITHERING_MODE_EN
//     ch4_compare -= 16;
// #else
//     ch4_compare -= 1;
// #endif
    LL_TIM_SetPrescaler(htim, 0);
    LL_TIM_SetCounterMode(htim, LL_TIM_COUNTERMODE_CENTER_DOWN); // Center Aligned mode1
    LL_TIM_SetRepetitionCounter(htim, rcr);
#ifdef TIM_DITHERING_MODE_EN
    /*
     * The ARR and CCR values will be updated automatically if the DITHEN bit is set / reset (for
     * instance, if ARR = 0x05 with DITHEN = 0, it will be updated to ARR = 0x50 with DITHEN = 1). --- RM0440
     */
    LL_TIM_EnableDithering(htim);
#endif
    LL_TIM_SetAutoReload(htim, arr);
    LL_TIM_SetClockDivision(htim, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_EnableARRPreload(htim);

    // Step #1: Set TRGO, PWM Modes...
    LL_TIM_SetTriggerOutput(htim, LL_TIM_TRGO_UPDATE);
    LL_TIM_OC_SetMode(htim, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetMode(htim, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetMode(htim, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);

    // Step #2: Initialize TIM
    LL_TIM_OC_SetPolarity(htim, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(htim, LL_TIM_CHANNEL_CH1N, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(htim, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(htim, LL_TIM_CHANNEL_CH2N, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(htim, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(htim, LL_TIM_CHANNEL_CH3N, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH3, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH1N, LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH2N, LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH3N, LL_TIM_OCIDLESTATE_HIGH);
    EnableBridges(Bridge::HB_U, Bridge::HB_V, Bridge::HB_W,
                  Bridge::LB_U, Bridge::LB_V, Bridge::LB_W);
    SetOutput3CHRaw(0, 0, 0);
    // SetOutput3CHPu(0.5f, 0.5f, 0.5f);
    __attribute__((unused)) uint32_t tmpsmcr = htim->SMCR & TIM_SMCR_SMS;
    if(initCNT)
    {
        EnableAllOutputs();
        LL_TIM_EnableCounter(htim);
    }
    return FuncRetCode::OK;
}

void FOCDriver6PWM::EnableBridge(FOCDriverBase::Bridge bridge)
{
    switch(bridge)
    {
        case Bridge::HB_U: return LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH1);
        case Bridge::HB_V: return LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH2);
        case Bridge::HB_W: return LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH3);
        case Bridge::LB_U: return LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH1N);
        case Bridge::LB_V: return LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH2N);
        case Bridge::LB_W: return LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH3N);
    }
}

void FOCDriver6PWM::DisableBridge(FOCDriverBase::Bridge bridge)
{
    switch(bridge)
    {
        case Bridge::HB_U: return LL_TIM_CC_DisableChannel(htim, LL_TIM_CHANNEL_CH1);
        case Bridge::HB_V: return LL_TIM_CC_DisableChannel(htim, LL_TIM_CHANNEL_CH2);
        case Bridge::HB_W: return LL_TIM_CC_DisableChannel(htim, LL_TIM_CHANNEL_CH3);
        case Bridge::LB_U: return LL_TIM_CC_DisableChannel(htim, LL_TIM_CHANNEL_CH1N);
        case Bridge::LB_V: return LL_TIM_CC_DisableChannel(htim, LL_TIM_CHANNEL_CH2N);
        case Bridge::LB_W: return LL_TIM_CC_DisableChannel(htim, LL_TIM_CHANNEL_CH3N);
    }
}

real_t FOCDriver6PWM::GetDeadTime()
{
    real_t dead_time = 0.0f;
    auto pclk1 = HAL::GetCoreClockHz();
    float Tdts = 1.0f / (float)(pclk1);
    auto DTGF = READ_BIT(htim->BDTR, TIM_BDTR_DTG);
    if(DTGF & (1 << 7))
    {
        if(DTGF & (1 << 6))
        {
            if(DTGF & (1 << 5))
            {
                dead_time = (32.0f + (float)(DTGF & 0x1F)) * 16.0f * Tdts;
            }
            else dead_time = (32.0f + (float)(DTGF & 0x1F)) * 8.0f * Tdts;
        }
        else dead_time = (64.0f + (float)(DTGF & 0x3F)) * 2.0f * Tdts;
    }
    else dead_time = (float)DTGF * Tdts;
    return dead_time;
}
}

#endif