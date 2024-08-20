#ifndef _FOC_MODULE_SOUND_INJECTOR_HPP
#define _FOC_MODULE_SOUND_INJECTOR_HPP

#include "foc_type.h"
#include <array>
#include "module_base.hpp"

// Input different frequency of square waves to Ud

// Note -> Freq:
//       1    1#   2    2#   3    4    4#   5    5#   6    6#   7
// Low  262  277  294  311  330  349  370  392  415  440  466  494
// Mid  523  554  587  622  659  698  740  784  831  880  932  988
// Hi.  1046 1109 1175 1245 1318 1397 1480 1568 1661 1760 1865 1976

typedef enum Note
{
    L01 = 0, L15, L2, L25, L3, L4, L45, L05, L55, L6, L65, L7,
    M1, M15, M2, M25, M3, M4, M45, M5, M55, M6, M65, M7,
    H1, H15, H2, H25, H3, H4, H45, H5, H55, H6, H65, H7,
}Note;

// const static uint16_t _sound_freq[36] = {262,277,294,311,330,349,370,392,415,440,466,494,523,554,587,622,659,698,740,784,831,880,932,988,1046,1109,1175,1245,1318,1397,1480,1568,1661,1760,1865,1976};
const static float _sound_Ts[36] = {1/262.0f,1/277.0f,1/294.0f,1/311.0f,1/330.0f,1/349.0f,1/370.0f,1/392.0f,1/415.0f,1/440.0f,1/466.0f,1/494.0f,1/523.0f,1/554.0f,1/587.0f,1/622.0f,1/659.0f,1/698.0f,1/740.0f,1/784.0f,1/831.0f,1/880.0f,1/932.0f,1/988.0f,1/1046.0f,1/1109.0f,1/1175.0f,1/1245.0f,1/1318.0f,1/1397.0f,1/1480.0f,1/1568.0f,1/1661.0f,1/1760.0f,1/1865.0f,1/1976.0f};

class SoundInjector : public ModuleBase
{
public:
    template<std::size_t _Nm>
    bool PlaySound(const std::array<Note, _Nm> &array, uint16_t BPM);
    bool Postprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts) final;
    float inject_voltage = 0.0f;
private:
    bool play_complete = true;
    float beat_time = 0.0f; // = 60s / BPM
    float state_timer = 0.0f;
    float note_timer = 0.0f;
    float flip_time = 0.0f;
    const Note* iter_begin = nullptr;
    const Note* iter_end = nullptr;
};

template<std::size_t _Nm>
bool SoundInjector::PlaySound(const std::array<Note, _Nm> &array, uint16_t BPM)
{
    if(!play_complete) return false;
    play_complete = false;
    state_timer = 0.0f;
    beat_time = 60.0f / (float)BPM;
    iter_begin = array.begin();
    iter_end = array.end();
    flip_time = _sound_Ts[*iter_begin] * 0.5f;
    return true;
}

bool SoundInjector::Postprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts)
{
    if(!play_complete && iter_begin != nullptr && iter_end != nullptr && iter_begin <= iter_end)
    {
        if(iter_begin == iter_end)
        {
            play_complete = true;
            state_timer = 0.0f;
            note_timer = 0.0f;
            return true;
        }
        state_timer += Ts;
        note_timer += Ts;
        if(note_timer < flip_time) out->Uqd.d += inject_voltage;
        // if(note_timer < flip_time) in->Iqd_target.d += inject_current;
        else
        {
            out->Uqd.d -= inject_voltage;
            // in->Iqd_target.d -= inject_current;
            if(note_timer >= _sound_Ts[*iter_begin]) note_timer = 0;
        }
        if(state_timer >= beat_time)
        {
            iter_begin++;
            state_timer = 0.0f;
            note_timer = 0.0f;
            flip_time = _sound_Ts[*iter_begin] * 0.5f;
        }
    }
    return true;
}

#endif