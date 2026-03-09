#include "task_update_encoder.hpp"

#include "../Motor/FOC/foc_motor.hpp"

namespace iFOC::Encoder
{

void UpdateEncoderTask::InitRT()
{
    const auto foc = GetMotor<FOCMotor>();
    encoder->Init(foc->GetInternalID());
}

void UpdateEncoderTask::UpdateMid(const float Ts)
{
    encoder->UpdateMid(Ts);
}

void UpdateEncoderTask::UpdateRT(const float Ts)
{
    encoder->UpdateRT(Ts);
}

}
