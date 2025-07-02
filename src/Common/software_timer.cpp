#include "software_timer.hpp"

namespace iFOC
{
FuncRetCode SoftwareTimer::AddTimer(uint8_t id, uint32_t interval, TimerCallback cb)
{
    if(!cb || interval == 0) return FuncRetCode::INVALID_INPUT;
    for(const auto& timer : timers)
    {
        if(timer.id == id) return FuncRetCode::PARAM_DUPLICATED;
    }
    timers.emplace_back(id, 0, interval, cb);
    return FuncRetCode::OK;
}

FuncRetCode SoftwareTimer::DelTimer(uint8_t id)
{
    const auto removed = timers.remove_if([id](const Timer& timer)
    {
        if(timer.id == id) return true;
        return false;
    });
    if(removed > 0) return FuncRetCode::OK;
    return FuncRetCode::PARAM_NOT_EXIST;
}

FuncRetCode SoftwareTimer::SetTimerInterval(uint8_t id, uint32_t interval)
{
    if(interval == 0) return FuncRetCode::INVALID_INPUT;
    for(auto& timer : timers)
    {
        if(timer.id == id)
        {
            timer.curr_val = 0; // reset first
            timer.set_interval = interval;
            return FuncRetCode::OK;
        }
    }
    return FuncRetCode::PARAM_NOT_EXIST;
}

void SoftwareTimer::UpdateTimer(uint32_t tickrate)
{
    for(auto& timer : timers)
    {
        timer.curr_val += tickrate;
        if(timer.curr_val >= timer.set_interval)
        {
            if(timer.callback) timer.callback(timer.id);
            timer.curr_val = 0;
        }
    }
}
}
