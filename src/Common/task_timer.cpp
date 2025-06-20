#pragma once

#include "task_timer.hpp"
#include "hal_impl.hpp"

#pragma GCC push_options
#pragma GCC optimize (2)

namespace iFOC
{
#ifdef MEASURE_TASK_TIME_IN_MICROS
uint32_t TaskTimer::start()
{
    return HAL::PerfCounter::GetCounter() / HAL::PerfCounter::counter_to_us;
}

void TaskTimer::stop(uint32_t start_us)
{
    uint32_t end_time_us = HAL::PerfCounter::GetCounter() / HAL::PerfCounter::counter_to_us;
    elapsed_time_us = end_time_us - start_us;
    if(end_time_us < start_us) elapsed_time_us += HAL::PerfCounter::max_counter_us;
    // max_elapsed_time_us = MAX(max_elapsed_time_us, elapsed_time_us);
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
#pragma GCC pop_options