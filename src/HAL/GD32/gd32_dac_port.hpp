#pragma once

#include "../../Common/Interface/dac_port_base.hpp"
#include "../hal_const.h"

#if defined(GD32_ENV)

namespace iFOC::HAL
{
class DACPort final : public DACPortBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(DACPort);
public:
    DACPort(uint32_t _dac, uint8_t _ch);
    DACPort(uint32_t _dac, uint8_t _ch, uint32_t* _pVrefint);
    void Init() override;
    void SetOutput_mV(real_t voltage) override;
private:
    uint32_t hdac;
    uint8_t ch;
    uint32_t* m_pVrefint = nullptr;
};
}

#endif