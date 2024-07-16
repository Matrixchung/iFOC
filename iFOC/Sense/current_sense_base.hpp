#ifndef _FOC_CURRENT_SENSE_BASE_H
#define _FOC_CURRENT_SENSE_BASE_H

#include "foc_header.h"

template <class T>
class CurrentSenseBase
{
public:
    CurrentSenseBase(float sense_gain, float shunt_mohm)
    {
        if(sense_gain > 0.0f && shunt_mohm > 0.0f) current_factor = (float)(1.0f / (sense_gain * shunt_mohm));
        else current_factor = 1.0f;
    };
    abc_t Iabc = {.a = 0.0f, .b = 0.0f, .c = 0.0f};
    void CurrentSenseUpdate() { static_cast<T*>(this)->Update(); };
protected:
    float current_factor = 1.0f;
};

class CurrentSenseDefault : public CurrentSenseBase<CurrentSenseDefault>
{
public:
    CurrentSenseDefault(float sense_gain, float shunt_mohm) : CurrentSenseBase(sense_gain, shunt_mohm) {};
    void Update() {};
};

#endif