#include "foc_types.hpp"
#include "foc_math.hpp"

namespace iFOC
{
std::underlying_type_t<DataType::Base::MisconfiguredArea> misconfigured_area = to_underlying(DataType::Base::MisconfiguredArea::NONE);

float RT_LOOP_TS = 0.00005f;

float MID_LOOP_TS = 0.001f;

uint8_t SYSTEM_MOTOR_NUM = 0;
}