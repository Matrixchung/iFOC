#include "foc_task_tone_player.hpp"

#define foc GetMotor<FOCMotor>()
static constexpr float GLOBAL_MAX_INJECT_VOLTAGE = 5.0f;

namespace iFOC
{
TonePlayerTask::TonePlayerTask() : Task("TonePlayer"), iter_begin(note_period_list.cbegin())
{
    RegisterTask(TaskType::RT_TASK, TaskType::NORMAL_TASK);
    config.rtos_priority = 1;
    note_period_list.reserve(8);
}

FuncRetCode TonePlayerTask::PlaySound(const Vector<real_t>& freq_list, float Tbeat, float voltage, bool is_bypass)
{
    if(!play_complete) return FuncRetCode::BUSY;
    is_encoder_bypassed = is_bypass;
    if(is_encoder_bypassed)
    {
        foc->BypassTaskByName("EncArbiter");
        foc->BypassTaskByName("CurrLoop");
    }
    std::for_each(freq_list.begin(), freq_list.end(), [this](real_t freq){
        this->note_period_list.emplace_back(1.0f / freq);
    });
    beat_timer = 0.0f;
    // note_timer = 0.0f;
    beat_time = Tbeat;
    iter_begin = note_period_list.cbegin();
    wave.SetPeriod(*iter_begin);
    inject_voltage = voltage;
    play_complete = false;
    return FuncRetCode::OK;
}

FuncRetCode TonePlayerTask::PlaySound(const Vector<real_t>& freq_list, float Tbeat, bool is_bypass)
{
    float Uinject = MAX(foc->GetBusSense()->voltage, foc->GetConfig().max_voltage()) * 0.5f;
    if(Uinject >= GLOBAL_MAX_INJECT_VOLTAGE) Uinject = GLOBAL_MAX_INJECT_VOLTAGE;
    return PlaySound(freq_list, Tbeat, Uinject, is_bypass);
}

void TonePlayerTask::UpdateRT(float Ts)
{
    if(!play_complete)
    {
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
                play_complete = true;
                foc->Uqd_target.d = 0.0f;
                return;
            }
        }
    }
}

void TonePlayerTask::UpdateNormal()
{
    if(play_complete && !note_period_list.empty())
    {
        note_period_list.clear();
        if(is_encoder_bypassed)
        {
            foc->UnbypassTaskByName("EncArbiter");
            foc->UnbypassTaskByName("CurrLoop");
        }
        foc->RemoveTaskByName(GetName());
    }
    sleep(100);
}

}