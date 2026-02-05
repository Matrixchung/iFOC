#include "foc_task_tone_player.hpp"

static constexpr float GLOBAL_MAX_INJECT_VOLTAGE = 5.0f;
static constexpr float GLOBAL_MAX_INJECT_CURRENT = 5.0f;

namespace iFOC
{
TonePlayerTask::TonePlayerTask() : Task("TonePlayer"), iter_begin(note_period_list.cbegin())
{
    // RegisterTask(TaskType::RT_TASK, TaskType::NORMAL_TASK);
    // config.rtos_priority = tskIDLE_PRIORITY + 1;
    RegisterTask(TaskType::RT_TASK);
    note_period_list.reserve(8);
}

TonePlayerTask::~TonePlayerTask()
{
    if(const auto foc = GetMotor<FOCMotor>())
    {
        foc->Uqd_target.d = 0.0f;
        if(foc->GetCurrentState() == MotorState::IDLE) foc->Disarm();
        if(is_encoder_bypassed)
        {
            foc->UnbypassTaskByName("EncArbiter");
            foc->UnbypassTaskByName("CurrLoop");
        }
    }
}

FuncRetCode TonePlayerTask::PlaySound(const Vector<real_t>& freq_list, const float Tbeat, const float voltage, const bool is_bypass)
{
    if(!play_complete) return FuncRetCode::BUSY;
    const auto foc = GetMotor<FOCMotor>();
    is_encoder_bypassed = is_bypass;
    if(is_encoder_bypassed)
    {
        foc->BypassTaskByName("EncArbiter");
        foc->BypassTaskByName("CurrLoop");
    }
    std::for_each(freq_list.begin(), freq_list.end(), [this](const real_t freq){
        this->note_period_list.emplace_back(1.0f / freq);
    });
    wave.SetWaveType(BoardConfig().GetConfig().use_square_wave_tone() ? WaveInjector::WaveType::SQUARE : WaveInjector::WaveType::SINUSOIDAL);
    beat_timer = 0.0f;
    // note_timer = 0.0f;
    beat_time = Tbeat;
    iter_begin = note_period_list.cbegin();
    wave.SetPeriod(*iter_begin);
    inject_voltage = voltage;
    foc->Arm();
    play_complete = false;
    return FuncRetCode::OK;
}

FuncRetCode TonePlayerTask::PlaySound(const Vector<real_t>& freq_list, const float Tbeat, const bool is_bypass)
{
    const auto foc = GetMotor<FOCMotor>();
    float Uinject = MAX(foc->GetBusSense()->voltage, foc->GetConfig().max_voltage()) * 0.5f;
    if(Uinject >= GLOBAL_MAX_INJECT_VOLTAGE) Uinject = GLOBAL_MAX_INJECT_VOLTAGE;
    if(foc->GetConfig().phase_resistance_valid())
    {
        float Umax = GLOBAL_MAX_INJECT_CURRENT * foc->GetConfig().phase_resistance();
        Uinject = MIN(Uinject, Umax);
    }
    return PlaySound(freq_list, Tbeat, Uinject, is_bypass);
}

FuncRetCode TonePlayerTask::PlaySoundContinuously(const Vector<real_t>& freq_list, const float Tbeat, const bool is_bypass)
{
    continuous = true;
    return PlaySound(freq_list, Tbeat, is_bypass);
}

void TonePlayerTask::UpdateRT(const float Ts)
{
    if(!play_complete)
    {
        const auto foc = GetMotor<FOCMotor>();
        if(beat_timer <= beat_time)
        {
            beat_timer += Ts;
            foc->Uqd_target.d = inject_voltage * wave.GetWaveform(Ts);
            // note_timer += Ts;
            // if(note_timer >= *iter_begin) note_timer = 0.0f;
            // else if(note_timer <= *iter_begin * 0.5f) foc->Uqd_target.d = inject_voltage;
            // else foc->Uqd_target.d = -inject_voltage;
        }
        else
        {
            ++iter_begin;
            wave.SetPeriod(*iter_begin);
            beat_timer = 0.0f;
            if(iter_begin == note_period_list.cend())
            {
                if(!continuous)
                {
                    play_complete = true;
                    foc->Uqd_target.d = 0.0f;
                }
                else // never ends actively, must be ended by external task
                {
                    foc->Uqd_target.d = 0.0f;
                    iter_begin = note_period_list.cbegin(); // iterate back, looping
                    wave.SetPeriod(*iter_begin);
                }
            }
        }
    }
}

bool TonePlayerTask::IsCompleted() const
{
    return play_complete;
}

bool TonePlayerTask::IsContinuous() const
{
    return continuous;
}

// void TonePlayerTask::UpdateNormal()
// {
//     if(play_complete && !note_period_list.empty())
//     {
//         const auto foc = GetMotor<FOCMotor>();
//         note_period_list.clear();
//         if(is_encoder_bypassed)
//         {
//             foc->UnbypassTaskByName("EncArbiter");
//             foc->UnbypassTaskByName("CurrLoop");
//         }
//         foc->RemoveTaskByName(GetName());
//     }
//     sleep(100);
// }

}