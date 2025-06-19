#pragma once

#include "task_processor.hpp"
#include "encoder_base.hpp"

namespace iFOC::Encoder
{
class UpdateEncoderTask final: public iFOC::Task
{
    // Output only, thus not keeping any pointer of the motor instance.
public:
    EncoderBase* const encoder;
public:
    explicit UpdateEncoderTask(EncoderBase* _enc) : iFOC::Task(_enc->GetName()), encoder(_enc)
    {
        RegisterTask(TaskType::RT_TASK, TaskType::MID_TASK);
        // The normal loop runs in RTOS scheduler without precise timedelta.
    };
    void InitRT() final;
    void UpdateMid(float Ts) final;
    void UpdateRT(float Ts) final;
};

}