#pragma once

#include "foc_types.hpp"

namespace iFOC
{
class SoftwareTimer
{
public:
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(SoftwareTimer);
    using TimerCallback = std::function<void(uint8_t id)>;
    struct Timer
    {
        uint8_t id = 0; // UNIQUE id, thus maximum timer count = 256
        uint32_t curr_val = 0;
        uint32_t set_interval = 0;
        TimerCallback callback = nullptr;
    };
    SoftwareTimer() = default;
    ~SoftwareTimer() = default;
    FuncRetCode AddTimer(uint8_t id, uint32_t interval, TimerCallback cb);
    FuncRetCode DelTimer(uint8_t id);
    FuncRetCode SetTimerInterval(uint8_t id, uint32_t interval);
    void UpdateTimer(uint32_t tickrate);
private:
    List<Timer> timers{};
};
}