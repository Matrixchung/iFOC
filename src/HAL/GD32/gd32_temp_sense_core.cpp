#include "gd32_temp_sense_core.hpp"

#if defined(GD32_ENV)

#if defined(GD32G5X3)
#define ADC_TEMP_CALIBRATION_VALUE_25      (REG16(0x1FFFB3F8) & 0x0FFF)
#else
#error "Temp_calib address not defined for current platform"
#endif

namespace iFOC::Sense
{
TempSenseCore::TempSenseCore(HAL::ADCPortBase* _port) : port(_port) {}

real_t TempSenseCore::Update()
{
    temp_celsius = (real_t)((ADC_TEMP_CALIBRATION_VALUE_25) - (int32_t)port->GetRawValue()) * 3.3f / 4095 / 3.99f * 1000 + 25.0f;
    return temp_celsius;
}
}

#endif