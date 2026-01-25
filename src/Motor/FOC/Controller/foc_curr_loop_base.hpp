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
    virtual void ResetCurrLoop() = 0;
};
}