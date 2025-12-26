#pragma once

#include "foc_wave_gen_base.hpp"
#include "foc_math.hpp"

namespace iFOC::FOC
{
class WaveGenSVPWM final : public WaveGenBase
{
public:
    void UpdateRT(float Ts) override;
};
}