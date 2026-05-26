#ifndef IFOC_GIM6010_CPP_CLASSES_HPP
#define IFOC_GIM6010_CPP_CLASSES_HPP

#include "global_include.h"
#include "adc_handler.h"
#include "temp_sense_ntc.hpp"
#include "bus_sense_ina237.hpp"
#include "bus_sense_series.hpp"
#include "curr_sense_two_shunts.hpp"

#include "encoder_mt6835.hpp"
#include "foc_driver_drv830x.hpp"
#include "usb_protocol.hpp"

#include "at32wk_include.hpp"

extern iFOC::FOCMotor* motor_1;

extern iFOC::HAL::UART* uart1;
extern iFOC::HAL::UART* uart3;
extern iFOC::HAL::CAN* can1;

// extern iFOC::Protocol::USBProtocolFOC* usb_protocol;

extern iFOC::Driver::FOCDriverDRV830x* drv830x;


#endif //IFOC_GIM6010_CPP_CLASSES_HPP
