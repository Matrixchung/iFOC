#pragma once

#include "encoder_base.hpp"

namespace iFOC::Encoder
{
class UpdateEncoderTask final: public Task
{
public:
    EncoderBase* const encoder;
    explicit UpdateEncoderTask(EncoderBase* _enc) : Task(_enc->GetName()), encoder(_enc)
    {
        RegisterTask(TaskType::RT_TASK, TaskType::MID_TASK);
        // The normal loop runs in RTOS scheduler without precise timedelta.
    };
    void InitRT() final;
    void UpdateMid(float Ts) final;
    void UpdateRT(float Ts) final;
};

}