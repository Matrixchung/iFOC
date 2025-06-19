#include "wave_injector.hpp"
#include "hal_impl.hpp"

#ifdef ENABLE_WAVE_INJ_PRE_CALC_TABLE
static constexpr size_t MAX_TABLE_LENGTH = 256; // Max memory size: 256 * sizeof(float) = 1024 Bytes
#endif

namespace iFOC
{
void WaveInjector::SetFrequency(float f)
{
    period_time = 1.0f / f;
    state_timer = 0.0f;
}

void WaveInjector::SetPeriod(float t)
{
    period_time = t;
    state_timer = 0.0f;
}

float WaveInjector::GetWaveform(float Ts)
{
    state_timer += Ts;
    if(state_timer >= period_time) state_timer = 0.0f;
    if(wave_type == WaveType::SQUARE)
    {
        if(state_timer <= period_time * 0.5f) return 1.0f;
        return -1.0f;
    }
#ifdef ENABLE_WAVE_INJ_PRE_CALC_TABLE
    if(sin_table_len && sin_table)
    {
        float x = sin_table[sin_table_ptr];
        sin_table_ptr++;
        if(sin_table_ptr >= sin_table_len) sin_table_ptr = 0;
        return x;
    }
#endif
    float angle = PI2 * state_timer / period_time;
    float ret = 0.0f, dummy;
    HAL::sinf_cosf_impl(angle, ret, dummy);
    return ret;
}
#ifdef ENABLE_WAVE_INJ_PRE_CALC_TABLE
void WaveInjector::PrepareTable(float Ts)
{
    if(period_time == 0.0f || Ts >= period_time || wave_type != WaveType::SINUSOIDAL)
    {
        sin_table_len = 0; return;
    }
    auto len = (size_t)(period_time / Ts);
    if(len > MAX_TABLE_LENGTH)
    {
        sin_table_len = 0; return;
    }
    sin_table = (float *) pvPortMalloc(len * sizeof(float));
    if(!sin_table)
    {
        sin_table_len = 0; return;
    }
    float timer = 0.0f;
    for(size_t i = 0; i < len; i++)
    {
        timer += Ts;
        if(timer >= period_time) timer = 0.0f;
        float angle = PI2 * timer / period_time;
        float ret = 0.0f, dummy;
        HAL::sinf_cosf_impl(angle, ret, dummy);
        sin_table[i] = ret;
    }
    sin_table_len = len;
    sin_table_ptr = 0;
}
WaveInjector::~WaveInjector()
{
    vPortFree(sin_table);
}
#else
void WaveInjector::PrepareTable(float Ts) {}
#endif
}



