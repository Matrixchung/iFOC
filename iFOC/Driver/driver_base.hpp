#ifndef _FOC_DRIVER_BASE_H
#define _FOC_DRIVER_BASE_H

#include "foc_header.h"

template<class T>
class DriverBase
{
public:
    inline bool DriverInit(bool initCNT) { return static_cast<T*>(this)->Init(initCNT); };
    inline void DriverEnableOutput() { static_cast<T*>(this)->EnableOutput(); };
    inline void DriverDisableOutput() { static_cast<T*>(this)->DisableOutput(); };
    inline void DriverSetLSIdleState(uint8_t state) { static_cast<T*>(this)->SetLSIdleState(state); };
    inline void SetOutput(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3) { static_cast<T*>(this)->SetOutputRaw(ch_1, ch_2, ch_3); };
    inline uint16_t GetMaxCompare() {return max_compare;};
    inline void SetOutputPct(float pct_1, float pct_2, float pct_3) // [0 - 1]
    {
        return SetOutput((uint16_t)(pct_1 * (uint16_t)max_compare), (uint16_t)(pct_2 * (uint16_t)max_compare), (uint16_t)(pct_3 * (uint16_t)max_compare));
    }
protected:
    uint16_t max_compare = 0;
private:

};

// Default do-nothing placeholder for DriverBase, mainly for pointer cast
class DriverDefault : public DriverBase<DriverDefault>
{
public:
    bool Init(bool initCNT) { return true; };
    void EnableOutput() {};
    void DisableOutput() {};
    void SetLSIdleState(uint8_t state) {};
    void SetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3) {};
};

template<typename T>
concept DriverImpl = std::is_base_of<DriverBase<T>, T>::value;

#endif