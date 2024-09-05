#ifndef _FOC_DRIVER_BDC_BASE_H
#define _FOC_DRIVER_BDC_BASE_H

#include "foc_math.h"

template<class T>
class DriverBDCBase
{
public:
    DriverBDCBase(uint16_t _max_compare) : max_compare(_max_compare) {};
    bool DriverInit(bool initCNT) { return static_cast<T*>(this)->Init(initCNT); }
    void DriverEnableOutput() { static_cast<T*>(this)->EnableOutput(); };
    void DriverDisableOutput() { static_cast<T*>(this)->DisableOutput(); };
    void SetOutput(uint16_t ch_1, uint8_t dir) 
    { 
        static_cast<T*>(this)->SetOutputRaw(ch_1, dir); 
        _dir = dir == 1 ? 1 : -1;
    }
    uint16_t GetMaxCompare() {return max_compare;};
    void SetOutputPct(float pct_1) // [-1.0f, 1.0f]
    {
        pct_1 = _constrain(pct_1, -1.0f, 1.0f);
        if(pct_1 < 0.0f) 
        {
            SetOutput((uint16_t)(-pct_1 * max_compare), 0);
            _dir = -1;
        }
        else
        {
            SetOutput((uint16_t)(pct_1 * max_compare), 1);
            _dir = 1;
        }
    }
    int8_t _dir = 1;
protected:
    uint16_t max_compare = 0;
};

#endif