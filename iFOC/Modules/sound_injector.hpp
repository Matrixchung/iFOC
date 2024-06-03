#ifndef _FOC_MODULE_SOUND_INJECTOR_HPP
#define _FOC_MODULE_SOUND_INJECTOR_HPP

#include "foc_type.h"
#include <array>

// Input different frequency of square waves to Ud

// Note -> Freq:
//       1    1#   2    2#   3    4    4#   5    5#   6    6#   7
// Low  262  277  294  311  330  349  370  392  415  440  466  494
// Mid  523  554  587  622  659  698  740  784  831  880  932  988
// Hi.  1046 1109 1175 1245 1318 1397 1480 1568 1661 1760 1865 1976

typedef enum Note
{
    L1 = 0, L15, L2, L25, L3, L4, L45, L5, L55, L6, L65, L7,
    M1, M15, M2, M25, M3, M4, M45, M5, M55, M6, M65, M7,
    H1, H15, H2, H25, H3, H4, H45, H5, H55, H6, H65, H7,
}Note;

const static uint16_t _sound_freq[36] = {262,277,294,311,330,349,370,392,415,440,466,494,523,554,587,622,659,698,740,784,831,880,932,988,1046,1109,1175,1245,1318,1397,1480,1568,1661,1760,1865,1976};

class SoundInjector
{
public:
    template<std::size_t _Nm>
    bool PlaySound(const std::array<Note, _Nm> &array, uint8_t BPM);
    void Update(qd_t &_modified, float Ts); // should be updated in inner loop
private:
    bool play_complete = true;
    float beat_time = 0.0f; // = 60s / BPM
    float state_timer = 0.0f;
    const Note* iter_begin = nullptr;
    const Note* iter_end = nullptr;
};

template<std::size_t _Nm>
bool SoundInjector::PlaySound(const std::array<Note, _Nm> &array, uint8_t BPM)
{
    if(!play_complete) return false;
    play_complete = false;
    state_timer = 0.0f;
    beat_time = 60.0f / (float)BPM;
    iter_begin = array.begin();
    iter_end = array.end();
    return true;
}

void SoundInjector::Update(qd_t &_modified, float Ts)
{
    if(!play_complete && iter_begin != nullptr && iter_end != nullptr && iter_begin <= iter_end)
    {
        if(iter_begin == iter_end)
        {
            play_complete = true;
            state_timer = 0.0f;
            return;
        }
        state_timer += Ts;
        if(state_timer >= beat_time)
        {
            // uint16_t freq = _sound_freq[*iter_begin];
            iter_begin++;
            state_timer = 0.0f;
        }
    }
}

#endif