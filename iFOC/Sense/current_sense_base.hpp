#ifndef _FOC_CURRENT_SENSE_BASE_H
#define _FOC_CURRENT_SENSE_BASE_H

#include "foc_header.h"

template <typename T>
class CurrentSenseBase
{
public:
    CurrentSenseBase(float sense_gain, float shunt_mohm)
    {
        if(sense_gain > 0.0f && shunt_mohm > 0.0f) current_factor = (float)(1.0f / (sense_gain * shunt_mohm));
        else current_factor = 1.0f;
    };
    abc_t Iabc;
    void CurrentSenseUpdate() { static_cast<T*>(this)->Update(); };
protected:
    float current_factor = 1.0f;
};

#endif