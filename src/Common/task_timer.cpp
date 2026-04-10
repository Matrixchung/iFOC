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
    return HAL::PerfCounter::GetCounter() / HAL::PerfCounter::counter_to_us;
}

void TaskTimer::stop(uint32_t start_us)
{
    const uint32_t end_time_us = HAL::PerfCounter::GetCounter() / HAL::PerfCounter::counter_to_us;
    elapsed_time_us = end_time_us - start_us;
    if(end_time_us < start_us) elapsed_time_us += HAL::PerfCounter::max_counter_us;
    // max_elapsed_time_us = IFOC_MAX(max_elapsed_time_us, elapsed_time_us);
    if(elapsed_time_us > max_elapsed_time_us) max_elapsed_time_us = elapsed_time_us;
}
#else
uint32_t TaskTimer::start()
{
    return HAL::PerfCounter::GetCounter();
}

void TaskTimer::stop(uint32_t start_cycle)
{
    uint32_t end_time_cycle = HAL::PerfCounter::GetCounter();
    elapsed_time_cycle = end_time_cycle - start_cycle;
    if(end_time_cycle < start_cycle) elapsed_time_cycle += HAL::PerfCounter::max_counter;
    if(elapsed_time_cycle > max_elapsed_time_cycle) max_elapsed_time_cycle = elapsed_time_cycle;
}
#endif

TaskTimerContext::TaskTimerContext(TaskTimer &t)  : timer(t), start_time(timer.start()) {}
TaskTimerContext::~TaskTimerContext() { timer.stop(start_time); };
}
#ifdef __GNUC__
#pragma GCC pop_options
#endif