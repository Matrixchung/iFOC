#pragma once

#include "foc_types.hpp"

// Credits: ODrive, https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/task_timer.hpp
// Tested under STM32G4 @ 170MHz, single MEASURE_TIME takes about 1us

namespace iFOC
{

// #define MEASURE_TASK_TIME_IN_CYCLES
#define MEASURE_TASK_TIME_IN_MICROS

#ifdef MEASURE_TASK_TIME_IN_MICROS
struct TaskTimer
{
public:
    // uint32_t start_time_us = 0;
    // uint32_t end_time_us = 0;
    uint32_t elapsed_time_us = 0;
    uint32_t max_elapsed_time_us = 0;
    uint32_t start();
    void stop(uint32_t start_us);
};
#else
struct TaskTimer
{
public:
    // uint32_t start_time_cycle = 0;
    // uint32_t end_time_cycle = 0;
    uint32_t elapsed_time_cycle = 0;
    uint32_t max_elapsed_time_cycle = 0;
    uint32_t start();
    void stop(uint32_t start_cycle);
};
#endif

struct TaskTimerContext
{
public:
    TaskTimerContext(const TaskTimerContext&) = delete;
    TaskTimerContext(const TaskTimerContext&&) = delete;
    void operator=(const TaskTimerContext&) = delete;
    void operator=(const TaskTimerContext&&) = delete;
    explicit TaskTimerContext(TaskTimer& t);
    ~TaskTimerContext();
    TaskTimer& timer;
    uint32_t start_time = 0;
    bool exit = false;
};
}

#define MEASURE_TIME(timer) for(iFOC::TaskTimerContext __ctx{(timer)}; !__ctx.exit; __ctx.exit = true)