#pragma once

#include "foc_driver_base.hpp"
#include "main.h"

#if defined(HAL_TIM_MODULE_ENABLED)

/*
 * Test A: Disabling single both High & Low side bridge will create a floating voltage on phase (when phases are floating
 * and not interconnected by motor), can be terminated by shorting this phase to other grounded phase
 * Test B: Only enable HB_U + LB_W, with phase U 50% duty: X (None of the low side bridges is open)
 * Test C: Only enable HB_U + HB/LB_W, with phase U 50% duty: LB_W opened 100%, while HB_U still floating (when empty).
 *         AFTER shorting phase U with phase W, it seems that phase U is active. Shorting phase V with phase W has no effect.
 *         Set phase U duty: 1%, shorting U and W with a cable, use oscilloscope to measure real HB_U duty: 1% √
 * Conclusion: Only through enables corresponding High side bridge can we control the low side status.
 */

/*
 * https://blog.csdn.net/wallace89/article/details/144520720
 * Counter Mode: Center Aligned mode1
 * For Low-Side sampling applications:
 *  Sample twice in a single PWM period (PWM Mode2, RCR = 0):
 *    1) ARR from 0 to ARR - 1: one Update Event, HS = 1, LS = 0 -> Sample remaining current, to dynamically calibrate zero-point
 *       After ADC injection finished, the TIM is now downcounting, so corresponding counter direction is DOWN
 *    2) ARR from ARR to 1: one Update Event, HS = 0, LS = 1 -> Sample Phase current, used in FOC regulating
 *       After ADC injection finished, the TIM is now upcounting, so corresponding counter direction is UP
 *    However, this limited inner loop algorithm's total execution time to (1 / 2*fPWM) seconds, because after half period the Update Event
 *    is called again with another ADC INJ interrupt.
 *
 *  Workaround: ADC INJ interrupt with a higher PRIORITY (still triggered by TIM Update Event)
 *              only does twice sampling (downcounting = remaining current, upcounting = phase current)
 *
 *              TIM CH4 triggers CC4 interrupt, only at CCR4 = 1 (HS = 0, LS = 1), process control loop
 *              at a lower priority than ADC INJ interrupt
 *
 * TRGO Event can be set up with Update Event, instead of OC4REF.
 * PS: RCR value doesn't change OC4REF frequency.
 *
 * PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive.
 * In downcounting, channel 1 is inactive (tim_oc1ref = 0) as long as TIMx_CNT>TIMx_CCR1 else active (tim_oc1ref = 1).
 *
 * PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active.
 * In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else inactive.
 *
 * Center aligned mode1: Initiates Compare Interrupt when downcounting
 * Center aligned mode2: Initiates Compare Interrupt when upcounting
 * Center aligned mode3: both up/downcounting
 */

namespace iFOC::Driver
{

class FOCDriver6PWM : public FOCDriverBase
{
public:
    explicit FOCDriver6PWM(TIM_TypeDef *_htim);
    FOCDriver6PWM() = delete;
    FuncRetCode Init(bool initCNT) override;
    __fast_inline void SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3) override;
    __fast_inline void EnableAllOutputs() override { LL_TIM_EnableAllOutputs(htim); };
    __fast_inline void DisableAllOutputs() override { LL_TIM_DisableAllOutputs(htim); };
    void EnableBridge(Bridge bridge) override;
    void DisableBridge(Bridge bridge) override;
    real_t GetDeadTime() override;
private:
    TIM_TypeDef *htim;
};

__fast_inline void FOCDriver6PWM::SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3)
{
    // PWM Mode2: max_compare - ch1, PWM Mode1: ch1
    LL_TIM_OC_SetCompareCH1(htim, max_compare - ch1);
    LL_TIM_OC_SetCompareCH2(htim, max_compare - ch2);
    LL_TIM_OC_SetCompareCH3(htim, max_compare - ch3);
}
}

#endif
