#ifndef _FOC_GPIO_BASE_H
#define _FOC_GPIO_BASE_H

#include "stdint.h"

class GPIOBase
{
public:
    virtual void SetState(bool state) = 0;
    virtual void ToggleState() = 0;
    virtual bool ReadState() = 0;
};

#endif