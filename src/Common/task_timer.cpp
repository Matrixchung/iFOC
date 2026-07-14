#include "task_timer.hpp"
#include "../HAL/hal_impl.hpp"

#ifdef __GNUC__
#pragma GCC push_options
#pragma GCC optimize (3)
#endif

#if (configGENERATE_RUN_TIME_STATS == 1)
extern "C"
{
void configureTimerForRunTimeStats(void) {}
unsigned long getRunTimeCounterValue(void)
{
    return iFOC::HAL::PerfCounter::GetCounter();
}
}
#endif

namespace iFOC
{
#ifdef MEASURE_TASK_TIME_IN_MICROS
uint32_t TaskTimer::start()
{
    return HAL::PerfCounter::GetCounter();
}

void TaskTimer::stop(const uint32_t start_cycle)
{
    const uint32_t delta_cnt = HAL::PerfCounter::GetCounter() - start_cycle;
    elapsed_time_us = delta_cnt / HAL::PerfCounter::counter_to_us;
    if(elapsed_time_us > max_elapsed_time_us) max_elapsed_time_us = elapsed_time_us;
}
#else
uint32_t TaskTimer::start()
{
    return HAL::PerfCounter::GetCounter();
}

void TaskTimer::stop(const uint32_t start_cycle)
{
    elapsed_time_cycle = HAL::PerfCounter::GetCounter() - start_cycle;
    if(elapsed_time_cycle > max_elapsed_time_cycle) max_elapsed_time_cycle = elapsed_time_cycle;
}
#endif

TaskTimerContext::TaskTimerContext(TaskTimer &t)  : timer(t), start_time(TaskTimer::start()) {}
TaskTimerContext::~TaskTimerContext() { timer.stop(start_time); };
}
#ifdef __GNUC__
#pragma GCC pop_options
#endif