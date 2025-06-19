#pragma once

#include "../foc_motor.hpp"

namespace iFOC::FOC
{
// CurrLoop: Input <- [Iqd_target, Iqd_measured, elec_angle_rad, elec_omega_rad_s...]
//           Output -> Uqd_target
class CurrLoopBase : public Task
{
public:
    CurrLoopBase();
    void InitRT() final;
    void UpdateRT(float Ts) final;
    virtual void InitCurrLoop();
    virtual void UpdateCurrLoop(float Ts) = 0;
    virtual void ResetCurrLoop() = 0;
};
}