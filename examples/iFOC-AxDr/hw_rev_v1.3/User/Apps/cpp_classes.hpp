//
// Created by vip99 on 1/20/2026.
//

#ifndef IFOC_AXDR_CPP_CLASSES_HPP
#define IFOC_AXDR_CPP_CLASSES_HPP

#include "global_include.h"

#include "rtos_task.hpp"

#include "foc_motor.hpp"
#include "bus_sense_adc.hpp"
#include "curr_sense_three_shunts.hpp"
#include "stm32_include.hpp"

#include "vofa.hpp"
#include "lcd.h"

extern iFOC::HAL::UART* uart3;

extern iFOC::FOCMotor* motor_1;

void display_foc();

#endif //IFOC_AXDR_CPP_CLASSES_HPP