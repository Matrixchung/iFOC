#ifndef _FOC_BUS_SENSE_STATIC_H
#define _FOC_BUS_SENSE_STATIC_H

#include "bus_sense_base.hpp"

class BusSenseStatic : public BusSenseBase<BusSenseStatic>
{
public:
    BusSenseStatic(float _vbus)
    {
        Vbus = _vbus;
    };
    void Update() {};
};

#endif