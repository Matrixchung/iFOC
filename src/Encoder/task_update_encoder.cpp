#include "task_update_encoder.hpp"

namespace iFOC::Encoder
{

void UpdateEncoderTask::InitRT()
{
    encoder->Init();
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
