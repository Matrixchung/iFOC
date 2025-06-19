#pragma once

#include "../foc_motor.hpp"

namespace iFOC::FOC
{
// WaveGen: Input <- [Uqd_target, elec_angle_rad]
//          Output -> Topen_Pu[3]
class WaveGenBase : public Task
{
public:
    WaveGenBase();
    void InitRT() final;
    void UpdateRT(float Ts) final;
    virtual void UpdateWave(float Ts) = 0;
};
}