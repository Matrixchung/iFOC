#ifndef _FOC_BUS_SENSE_MIRROR_H
#define _FOC_BUS_SENSE_MIRROR_H

#include "bus_sense_base.hpp"

// use to mirror an existing bus sense result, for example from other channel

class BusSenseMirror : public BusSenseBase<BusSenseMirror>
{
public:
    BusSenseMirror(const float *_pVbus, const float *_pIbus): pVbus(_pVbus), pIbus(_pIbus) {};
    template<class T>
    explicit BusSenseMirror(const T *base)
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
    const float *pVbus;
    const float *pIbus;
};

#endif