#include "foc_types.hpp"
#include "foc_math.hpp"

namespace iFOC
{
volatile std::underlying_type_t<DataType::Base::MisconfiguredArea> misconfigured_area = to_underlying(DataType::Base::MisconfiguredArea::NONE);

volatile float RT_LOOP_TS = 0.00005f;

volatile float MID_LOOP_TS = 0.001f;

volatile uint8_t SYSTEM_MOTOR_NUM = 0;
}