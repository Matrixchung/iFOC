#pragma once

#include "../hal_const.h"
#include "../hal_impl.hpp"

#if defined(AT32WK_ENV) && defined(PWC_MODULE_ENABLED) && defined(CRM_MODULE_ENABLED)

// #define AT32WK_TEST_BKP

#ifdef AT32WK_TEST_BKP
#include "at32wk_bkp_struct.h"
bool read_bpr(bkp_struct_t* dst);
void write_bpr(bkp_struct_t* src);
#endif

namespace iFOC::HAL::Bootloader
{
    void SetAppInitSuccessFlag();
    void OnHardFault();
#ifdef AT32WK_TEST_BKP
    void TestBKP(bkp_struct_t* bkp);
#endif
}

#endif