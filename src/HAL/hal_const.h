#pragma once

#if defined __has_include
#  if __has_include ("main.h") // STM32 Environment
    #  include "main.h"
#  elif __has_include ("wk_gpio.h") // AT32 with Workbench Environment
    #  define AT32WK_ENV
    #  include "wk_gpio.h"
#elif __has_include ("hpm_common.h") && __has_include("board.h") // HPMicro Environment
    #  define HPM_ENV
    #  include "board.h"
#elif defined GD32_ENV
    #if __has_include ("gd32g5x3_init.h")
        #ifndef GD32G5X3
            #define GD32G5X3
        #endif
        #include "gd32g5x3_init.h"
    #else
        #error "Supported GD32 platform not found, please implement"
    #endif
#else
    #error "No main.h specified, please check hal_const.h"
#  endif
#else
    #include "main.h"
#endif

#ifdef IFOC_CANFD_AVAILABLE
#undef IFOC_CANFD_AVAILABLE
#endif

#ifdef USE_HAL_DRIVER // STM32 Environment
#include "STM32/stm32_nvm_address.h"

#elif defined(AT32F403Axx) || defined(AT32F407xx) // AT32F40x Environment
#include "AT32WK/at32wk_nvm_address.h"

#elif defined(AT32F435xx) || defined(AT32F435xG) // AT32F435 Environment
#include "AT32WK/at32wk_nvm_address.h"

#elif defined(AT32F456xx) // AT32F456 Environment
#include "AT32WK/at32wk_nvm_address.h"
#define IFOC_CANFD_AVAILABLE

#elif defined(HPM_ENV)
#include "HPMicro/hpmicro_nvm_address.h"

#elif defined(GD32_ENV)
#include "GD32/gd32_nvm_address.h"
#endif

/* Checking const validity */

#if !defined(FLASH_WRITE_GRAN_BITS) || \
!defined(FLASH_SECTOR_SIZE_BYTES) || \
!defined(FLASH_USER_START_ADDR) || \
!defined(FLASH_USER_AREA_SIZE)
#error "At least one of the required constant is not defined, please check hal_const.h"
#endif

#if defined(USE_EASYFLASH) && defined(USE_FLASHDB)
#error "No more than one NVM backbone handler can be selected"
#endif

#if (FLASH_WRITE_GRAN_BITS % 8 != 0)
#error "FLASH_WRITE_GRAN_BITS % 8 != 0"
#endif