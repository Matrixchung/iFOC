#include "foc_wave_gen_svpwm.hpp"

#define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
void WaveGenSVPWM::UpdateWave(float Ts)
{
    // 1.5*Tpwm angle compensation, useful in high speed
    // https://www.zhihu.com/question/625597876/answer/3258736005
    float compensated_angle_rad = normalize_rad(foc->elec_angle_rad + foc->elec_omega_rad_s * 1.5f * Ts);
    // float compensated_angle_rad = foc->elec_angle_rad;
    FOC_SVPWM(foc->Uqd_target,
              compensated_angle_rad,
              foc->GetBusSense()->voltage,
              Tabc);
    foc->GetDriver()->SetOutput3CHPu(Tabc[0], Tabc[1], Tabc[2]);
}
}