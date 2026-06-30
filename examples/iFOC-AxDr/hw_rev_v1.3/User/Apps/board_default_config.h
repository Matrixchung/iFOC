#pragma once

#include "board_config.hpp"

#define APPLY_DEFAULT_BOARD_CONFIG() \
do{                                  \
auto& c = iFOC::BoardConfig().GetConfig(); \
c.set_uart_1_protocol(iFOC::DataType::Comm::UARTProtocol::UNIFIED); \
c.set_uart_1_baudrate(iFOC::DataType::Comm::UARTBaudrate::BAUD_921600); \
c.set_pwm_wave_freq(20000);          \
c.set_speed_loop_freq(5000);         \
c.set_bus_overvoltage_limit(48.0f);  \
c.set_bus_undervoltage_limit(8.0f);  \
c.set_bus_max_positive_current(30.0f);      \
c.set_bus_max_negative_current(-10.0f);      \
c.set_bus_sense_shunt_ohm(0.001f);   \
c.set_max_regen_current(10.0f);      \
c.set_current_sense_gain(20.0f);      \
c.set_current_sense_shunt_ohm(0.001f);       \
c.set_current_sense_f_lp(5000);      \
c.set_play_startup_tone(true);       \
c.set_use_square_wave_tone(true);        \
c.set_enable_can_terminal_resistor(false); \
}while(0)