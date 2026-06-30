#ifndef _VOFA_HPP
#define _VOFA_HPP

#include <cstdint>

constexpr size_t vofa_channel_count = 8;

class VOFA
{
public:
    VOFA() = default;
    VOFA(const VOFA &) = delete;
    VOFA &operator=(const VOFA &) = delete;
    void add(const uint8_t ch, float target)
    {
        if(ch >= vofa_channel_count) return;
        channel[ch] = target;
    }
    uint8_t *buffer() { return (uint8_t*)channel; }
private:
    float channel[vofa_channel_count] = {0.0f};
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
};
static VOFA vofa;

#endif