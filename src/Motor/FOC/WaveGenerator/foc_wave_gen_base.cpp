#include "foc_wave_gen_base.hpp"

#define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
WaveGenBase::WaveGenBase() : Task("WaveGen")
{
    RegisterTask(TaskType::RT_TASK);
}

void WaveGenBase::InitRT()
{
    foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
    foc->GetDriver()->EnableBridges(Driver::FOCDriverBase::Bridge::HB_U,
                                    Driver::FOCDriverBase::Bridge::LB_U,
                                    Driver::FOCDriverBase::Bridge::HB_V,
                                    Driver::FOCDriverBase::Bridge::LB_V,
                                    Driver::FOCDriverBase::Bridge::HB_W,
                                    Driver::FOCDriverBase::Bridge::LB_W);
}

void WaveGenBase::UpdateRT(float Ts)
{
    UpdateWave(Ts);
}
}


