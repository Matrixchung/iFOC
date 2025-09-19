#pragma once

#include "../../Encoder/encoder_ab_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED)

namespace iFOC::Encoder
{
class EncoderAB final : public EncoderABBase
{
public:
    EncoderAB(tmr_type *_htim, uint32_t _cpr);
    FuncRetCode Init() override;
    void UpdatePulse() override;
private:
    tmr_type *htim;
};
}

#endif