#pragma once

#if defined __has_include
#  if __has_include ("main.h")
#  include "main.h"
#  else
#error "No main.h specified, please check hal_const.h"
#  endif
#else
#include "main.h"
#endif

#ifdef USE_HAL_DRIVER // STM32 Environment
#include "stm32_nvm_address.h"

#endif

/* Checking const validity */

#if !defined(FLASH_WRITE_GRAN_BITS) || \
    !defined(FLASH_SECTOR_SIZE_BYTES) || \
    !defined(FLASH_USER_START_ADDR) || \
    !defined(FLASH_USER_AREA_SIZE)
#error "At least one of the required constant is not defined, please check hal_const.h"
#endif

#if (FLASH_WRITE_GRAN_BITS % 8 != 0)
#error "FLASH_WRITE_GRAN_BITS % 8 != 0"
#endif