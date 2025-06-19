#pragma once

#include <cstdint>

// Used to generate sinusoidal or square waveform with given period T or frequency F.

// To minimize repeating calculation overhead, we can predefine a stable Ts for GetWaveform(Ts)
// So we can have a pre-calculated sin table at given step when using sinusodial wave
// instead of call sin() every loop

#define ENABLE_WAVE_INJ_PRE_CALC_TABLE

#ifdef ENABLE_WAVE_INJ_PRE_CALC_TABLE
#include <cstddef>
#endif

namespace iFOC
{
class WaveInjector
{
public:
    enum class WaveType : uint8_t
    {
        SQUARE = 0,
        SINUSOIDAL
    };
    explicit WaveInjector(WaveType type) : wave_type(type) {};
    WaveInjector() : WaveInjector(WaveType::SQUARE) {};
    void SetFrequency(float f);
    void SetPeriod(float t);
    float GetWaveform(float Ts);
private:
    WaveType wave_type = WaveType::SQUARE;
    float period_time = 0.0f;
    float state_timer = 0.0f;
public:
    void PrepareTable(float Ts);
#ifdef ENABLE_WAVE_INJ_PRE_CALC_TABLE
public:
    ~WaveInjector();
private:
    float *sin_table = nullptr;
    size_t sin_table_len = 0;
    size_t sin_table_ptr = 0;
#endif
};
}