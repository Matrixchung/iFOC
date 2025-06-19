#pragma once

#include "../foc_motor.hpp"
#include "wave_injector.hpp"

namespace iFOC
{
class TonePlayerTask final : public Task
{
private:
    Vector<real_t> note_period_list;
    decltype(note_period_list)::const_iterator iter_begin;
    float beat_time = 0.0f;
    float beat_timer = 0.0f;
    float inject_voltage = 0.0f;
    bool play_complete = true;
    bool is_encoder_bypassed = false; // bypass encoder to prevent unintended movement
    WaveInjector wave{WaveInjector::WaveType::SINUSOIDAL};
public:
    TonePlayerTask();
    FuncRetCode PlaySound(const Vector<real_t>& freq_list, float Tbeat, float voltage, bool is_bypass = false);
    FuncRetCode PlaySound(const Vector<real_t>& freq_list, float Tbeat, bool is_bypass = false);
    void UpdateRT(float Ts) final;
    void UpdateNormal() final;
};
}