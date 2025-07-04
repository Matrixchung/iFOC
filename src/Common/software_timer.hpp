#pragma once

#include "foc_types.hpp"

namespace iFOC
{
class SoftwareTimer
{
public:
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(SoftwareTimer);
    using TimerCallback = std::function<void(uint16_t id)>;
    struct Timer
    {
        uint16_t id = 0; // UNIQUE id, thus maximum timer count = 65536
        uint32_t curr_val = 0;
        uint32_t set_interval = 0;
        TimerCallback callback = nullptr;
    };
    SoftwareTimer() = default;
    ~SoftwareTimer() = default;
    uint16_t GetUnusedTimerID();
    FuncRetCode AddTimer(uint16_t id, uint32_t interval, TimerCallback cb);
    FuncRetCode DelTimer(uint16_t id);
    FuncRetCode SetTimerInterval(uint16_t id, uint32_t interval);
    FuncRetCode ResetTimer(uint16_t id);
    void UpdateTimer(uint32_t tickrate);
private:
    List<Timer> timers{};
};
}