#ifndef _FOC_BUS_SENSE_MIRROR_H
#define _FOC_BUS_SENSE_MIRROR_H

#include "bus_sense_base.hpp"

// use to mirror an existing bus sense result, for example from other channel

class BusSenseMirror : public BusSenseBase<BusSenseMirror>
{
public:
    BusSenseMirror(float *_pVbus, float *_pIbus): pVbus(_pVbus), pIbus(_pIbus) {};
    template<class T>
    BusSenseMirror(T *base)
    {
        pVbus = &base->Vbus;
        pIbus = &base->Ibus;
    };
    void Update()
    {
        Vbus = *pVbus;
        Ibus = *pIbus;
    }
private:
    float *pVbus;
    float *pIbus;
};

#endif