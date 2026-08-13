#pragma once

#include "at32wk_gpio.hpp"
#include "at32wk_foc_driver_6pwm.hpp"
#include "at32wk_dc_driver_1pwm.hpp"
#include "at32wk_spi.hpp"
#include "at32wk_uart.hpp"
#include "at32wk_uart_hs.hpp"
#include "at32wk_usb_uart_hs.hpp"
#include "at32wk_can.hpp"
#include "at32wk_canfd.hpp"
#include "at32wk_ws2812.hpp"
#include "at32wk_adc_port.hpp"
#include "at32wk_temp_sense_core.hpp"
#include "at32wk_encoder_ab.hpp"
#include "at32wk_bkp.hpp"

#if defined(AT32F403AxG) || defined(AT32F407xx)
#else
extern uint32_t uptime_sec;
#endif