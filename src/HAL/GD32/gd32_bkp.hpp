#pragma once

#include "../hal_const.h"
#include "../hal_impl.hpp"

#if defined(GD32_ENV)

// #define GD32_TEST_BKP

#ifdef GD32_TEST_BKP
#include "at32wk_bkp_struct.h" // use AT32 bkp struct
bool read_bkp(bkp_struct_t* dst);
void write_bkp(bkp_struct_t* src);
#endif

namespace iFOC::HAL::Bootloader
{
    void SetAppInitSuccessFlag();
    void OnHardFault();
#ifdef GD32_TEST_BKP
    void TestBKP(bkp_struct_t* bkp);
#endif
}

#endif