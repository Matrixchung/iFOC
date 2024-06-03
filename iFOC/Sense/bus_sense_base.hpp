#ifndef _FOC_BUS_SENSE_BASE_H
#define _FOC_BUS_SENSE_BASE_H

#include "foc_header.h"
#include "sliding_filter.h"

template <typename T>
class BusSenseBase
{
public:
    float Vbus = 0.0f;
    float Ibus = 0.0f;
    void BusSenseUpdate() { static_cast<T*>(this)->Update(); };
};

#endif