#pragma once

#include "foc_task.hpp"
#include "motor_state.h"
#include <functional>

namespace iFOC::FOC
{
class StateMachineTask final : public Task
{
public:
    // requested_state, current_state, last_state
    using TransitionCallback = std::function<void(MotorState, MotorState, MotorState)>;
    StateMachineTask();
    void InitNormal() final;
    void UpdateNormal() final;
    MotorState RequestState(MotorState new_state);
    MotorState BackToLastState();
    bool CheckStateRequirement(MotorState new_state);
    __fast_inline MotorState GetState() { return current_state; }
    __fast_inline void RegisterSuccessCallback(auto cb) { trans_success_cb = cb; }
    __fast_inline void RegisterFailureCallback(auto cb) { trans_failure_cb = cb; }
    // __fast_inline void RegisterSuccessCallback(auto && cb) { trans_success_cb = std::move(cb); }
    // __fast_inline void RegisterFailureCallback(auto && cb) { trans_failure_cb = std::move(cb); }
protected:
    MotorState current_state = MotorState::IDLE;
    MotorState last_state = MotorState::IDLE;
private:
    TransitionCallback trans_success_cb;
    TransitionCallback trans_failure_cb;
};
}