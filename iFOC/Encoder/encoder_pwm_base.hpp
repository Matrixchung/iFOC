#ifndef _FOC_ENCODER_PWM_BASE_H
#define _FOC_ENCODER_PWM_BASE_H

#include "encoder_base.hpp"
#include "speed_pll.hpp"
#include "sliding_filter.h"
#include "lowpass_filter.h"

#define ENCODER_PWM_MINIMUM_DUTY 0.00388443797038116047584365137169f
#define ENCODER_PWM_DUTY_SECTION 0.99417334304442825928623452294246f

// using TIM Input Capture direct and indirect mode, ARR max, and set a appropriate Prescaler
// freq = 1KHz+-10%, total 4119 bits, start 16 clocks consists of init and error, 
// so duty < 16/4119 = 0.00388443797038116047584365137169f, or freq not met requirement = error state.
// exit 8 clocks is low, so full degree duty is 4111/4119 = 0.99805778101480941976207817431415f
// duty section is 4095/4119 = 0.99417334304442825928623452294246f
// to calculate angle: normalize_rad(PI2 * (duty - min_duty) / (duty_section))
// the irq_single_round_angle has to be calculated in interrupt

// **if pwm wire is disconnected, the interrupt won't be triggered, so we implement IRQ watchdog

class EncoderPWMBase : public EncoderBase
{
public:
    bool Init() override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override;
    bool IsError() override {return (bool)error_flag;};
    void CaptureInterrupt();
protected:
    virtual bool PortTIMInit() = 0;
    virtual void PortICInterrupt() = 0;
    uint32_t capture_1 = 0;
    uint32_t capture_2 = 0;
    float irq_single_round_angle = 0.0f;
    float frequency = 0.0f;
    uint16_t irq_in_times = 0;
    uint16_t velocity_call_times = 0;
    uint8_t error_flag = 0;
    SlidingFilter sliding_filter = SlidingFilter(10);
    LowpassFilter speed_filter = LowpassFilter(0.001f);
    LowpassFilter angle_filter = LowpassFilter(0.001f);
    // SpeedPLL speed_pll = SpeedPLL(0.0f, 0.0f, 0.0f);
    float last_raw_angle = 0.0f;
    float last_velocity = 0.0f;
    float single_round_angle_prev = -1.0f;
    
};

bool EncoderPWMBase::Init()
{
    // use_speed_pll = true;
    // speed_pll = SpeedPLL(700.0f, 90000.0f, max_velocity);
    // direction = _dir;
    return PortTIMInit();
}

void EncoderPWMBase::Update(float Ts)
{
    if(error_flag == 0)
    {
        single_round_angle = normalize_rad(irq_single_round_angle);
        single_round_angle = angle_filter.GetOutput(single_round_angle, Ts);
        // single_round_angle = irq_single_round_angle;
        if(single_round_angle_prev >= 0.0f)
        {
            float delta = single_round_angle - single_round_angle_prev;
            if(delta > 0.8f * PI2) full_rotations -= 1;
            else if(delta < -0.8f * PI2) full_rotations += 1;
            raw_angle = full_rotations * PI2 + single_round_angle;
        }
        single_round_angle_prev = single_round_angle;
    }
}

void EncoderPWMBase::UpdateMidInterval(float Ts)
{
    // Interrupt watchdog
    if(velocity_call_times >= 10000)
    {
        if(irq_in_times == 0) error_flag = 1;
        irq_in_times = 0;
        velocity_call_times = 0;
    }
    else velocity_call_times++;

    // float vel = ((float)(full_rotations - last_rotations) * PI2 + (single_round_angle - last_angle)) / Ts;
    // last_angle = single_round_angle;
    // last_rotations = full_rotations;
    float vel = (float)(raw_angle - last_raw_angle) / Ts;
    // vel = sliding_filter.GetOutput(vel);
    vel = speed_filter.GetOutput(vel, Ts);
    // if(ABS(vel) >= 1.0f && ABS(vel - last_velocity) >= 100.0f) vel = last_velocity;
    last_velocity = vel;
    last_raw_angle = raw_angle;
    // return speed_filter.GetOutput(vel, Ts);
    // return sliding_filter.GetOutput(vel);
    velocity = sliding_filter.GetOutput(vel);

    // return speed_pll.Calculate(single_round_angle, Ts);

    // float vel = speed_pll.Calculate(single_round_angle, Ts);
    // return speed_filter.GetOutput(vel, Ts);
}

bool EncoderPWMBase::IsCalibrated()
{
    return true;
}

void EncoderPWMBase::CaptureInterrupt()
{
    irq_in_times++;
    PortICInterrupt();
}

#endif