#include "task_update_encoder.hpp"

namespace iFOC::Encoder
{

void UpdateEncoderTask::InitRT()
{
    encoder->Init();
}

void UpdateEncoderTask::UpdateMid(float Ts)
{
    encoder->UpdateMid(Ts);
}

void UpdateEncoderTask::UpdateRT(float Ts)
{
    encoder->UpdateRT(Ts);
}

}