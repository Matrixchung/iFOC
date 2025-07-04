#include "software_timer.hpp"

#include <algorithm>

namespace iFOC
{

uint16_t SoftwareTimer::GetUnusedTimerID()
{
    for(size_t i = 0; i < timers.size(); i++)
    {
        if(timers[i].set_interval == 0) return i;
    }
    return 0xFFFF;
}

FuncRetCode SoftwareTimer::AddTimer(uint16_t id, uint32_t interval, TimerCallback cb)
{
    if(!cb || interval == 0 || id >= timers.size()) return FuncRetCode::INVALID_INPUT;
    if(timers[id].set_interval > 0) return FuncRetCode::PARAM_DUPLICATED;
    timers[id].id = id;
    timers[id].curr_val = 0;
    timers[id].set_interval = interval;
    timers[id].callback = cb;
    return FuncRetCode::OK;
}

FuncRetCode SoftwareTimer::DelTimer(uint16_t id)
{
    if(id >= timers.size()) return FuncRetCode::PARAM_NOT_EXIST;
    timers[id].set_interval = 0;
    timers[id].callback = nullptr;
    return FuncRetCode::OK;
}

FuncRetCode SoftwareTimer::SetTimerInterval(uint16_t id, uint32_t interval)
{
    if(id >= timers.size()) return FuncRetCode::PARAM_NOT_EXIST;
    if(interval == 0) return FuncRetCode::INVALID_INPUT;
    timers[id].set_interval = interval;
    return FuncRetCode::OK;
}

FuncRetCode SoftwareTimer::ResetTimer(uint16_t id)
{
    if(id >= timers.size()) return FuncRetCode::PARAM_NOT_EXIST;
    timers[id].curr_val = 0;
    return FuncRetCode::OK;
}

void SoftwareTimer::UpdateTimer(uint32_t tickrate)
{
    for(size_t i = 0; i < timers.size(); i++)
    {
        if(timers[i].set_interval > 0 && timers[i].callback)
        {
            timers[i].curr_val += tickrate;
            if(timers[i].curr_val >= timers[i].set_interval)
            {
                timers[i].callback(i);
                timers[i].curr_val = 0;
            }
        }
    }
}


// uint16_t SoftwareTimer::GetUnusedTimerID()
// {
//     for(uint16_t id = 0; id < 0xFFFF; id++)
//     {
//         const auto it = std::find_if(timers.cbegin(), timers.cend(), [id](const Timer& t)
//         {
//            return t.id == id && t.set_interval > 0;
//         });
//         if(it == timers.cend()) return id;
//     }
//     return 0xFFFF;
// }
//
// FuncRetCode SoftwareTimer::AddTimer(uint16_t id, uint32_t interval, TimerCallback cb)
// {
//     if(!cb || interval == 0) return FuncRetCode::INVALID_INPUT;
//     for(const auto& timer : timers)
//     {
//         if(timer.id == id) return FuncRetCode::PARAM_DUPLICATED;
//     }
//     timers.emplace_back(id, 0, interval, cb);
//     return FuncRetCode::OK;
// }
//
// FuncRetCode SoftwareTimer::DelTimer(uint16_t id)
// {
//     // const auto removed = timers.remove_if([id](const Timer& timer)
//     // {
//     //     if(timer.id == id || timer.set_interval == 0) return true;
//     //     return false;
//     // });
//     // if(removed > 0) return FuncRetCode::OK;
//     // return FuncRetCode::PARAM_NOT_EXIST;
//
//     // Delete timer by set the interval to 0, instead of directly delete it
//     // To prevent call DelTimer(id) inside timer.callback()
//     return SetTimerInterval(id, 0);
// }
//
// FuncRetCode SoftwareTimer::SetTimerInterval(uint16_t id, uint32_t interval)
// {
//     // if(interval == 0) return FuncRetCode::INVALID_INPUT;
//     for(auto& timer : timers)
//     {
//         if(timer.id == id)
//         {
//             timer.curr_val = 0; // reset first
//             timer.set_interval = interval;
//             return FuncRetCode::OK;
//         }
//     }
//     return FuncRetCode::PARAM_NOT_EXIST;
// }
//
// FuncRetCode SoftwareTimer::ResetTimer(uint16_t id)
// {
//     for(auto& timer : timers)
//     {
//         if(timer.id == id)
//         {
//             timer.curr_val = 0;
//             return FuncRetCode::OK;
//         }
//     }
//     return FuncRetCode::PARAM_NOT_EXIST;
// }
//
// void SoftwareTimer::UpdateTimer(uint32_t tickrate)
// {
//     for(auto& timer : timers)
//     {
//         if(timer.set_interval > 0)
//         {
//             timer.curr_val += tickrate;
//             if(timer.curr_val >= timer.set_interval)
//             {
//                 if(timer.callback) timer.callback(timer.id); // MUST NOT call remove_if inside callback()
//                 timer.curr_val = 0;
//             }
//         }
//     }
//     timers.remove_if([](const Timer& timer) // clear unused timer
//     {
//         return timer.set_interval == 0;
//     });
// }
}
