#ifndef _FOC_DRIVER_DRV8323_BASE_H
#define _FOC_DRIVER_DRV8323_BASE_H

#include "driver_base.hpp"

#define DRV8323_REG_DEFAULT_0x03 0x3FF
#define DRV8323_REG_DEFAULT_0x04 0x7FF
#define DRV8323_REG_DEFAULT_0x05 0x159
#define DRV8323_REG_DEFAULT_0x06 0x283
#define DRV8323_REG_CAL_0x06     0x29F // 0010 1001 1111(when calibrating three shunts amplifier)

template <class T>
class DriverDRV8323Base : public DriverBase<DriverDRV8323Base<T>>
{
public:
    DriverDRV8323Base(const uint16_t _reg_hs, const uint16_t _reg_ls)
    : reg_hs_value(_reg_hs), reg_ls_value(_reg_ls) {};
    bool Init(bool initCNT);
    inline void SetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3) { static_cast<T*>(this)->PortSetOutputRaw(ch_1, ch_2, ch_3); };
    inline void SetLSIdleState(uint8_t state) { static_cast<T*>(this)->SetLSIdleState(state); };
    void nFAULT_IRQHandler();
    uint8_t bus_fault_flag = 0;
    uint8_t nFAULT_flag = 0;
private:
    uint16_t reg_hs_value = DRV8323_REG_DEFAULT_0x03; // based on IdriveP and IdriveN settings
    uint16_t reg_ls_value = DRV8323_REG_DEFAULT_0x04;
    uint16_t ReadReg16(uint16_t reg, uint16_t *data);
    uint16_t WriteReg16(uint16_t reg, uint16_t data);
    void DelayMs(uint32_t ms)
    {
        while(ms--)DelayUs(1000);
    };
    inline bool SPIInit() { return static_cast<T*>(this)->PortSPIInit(); };
    inline bool TIMInit(bool initCNT) { return static_cast<T*>(this)->PortTIMInit(initCNT); };
    inline uint16_t SPIRead16(uint16_t reg, uint16_t *data) { return static_cast<T*>(this)->PortSPIRead16(reg, data); };
    inline void SetEN(uint8_t state) { return static_cast<T*>(this)->PortSetEN(state); };
    inline void SetCS(uint8_t state) { return static_cast<T*>(this)->PortSetCS(state); }; // CS is low valid
    inline bool ReadFAULT() { return static_cast<T*>(this)->PortReadFAULT(); };
    inline void DelayUs(uint32_t us) { return static_cast<T*>(this)->PortDelayUs(us); };
};

template <class T>
bool DriverDRV8323Base<T>::Init(bool initCNT)
{
    SetCS(1);
    DelayMs(50);
    // reset sequence
    SetEN(1);
    DelayUs(8);
    SetEN(0);
    DelayUs(20);
    SetEN(1);
    // init control logic
    DelayMs(10);
    if(!SPIInit()) return false;
    uint16_t ret = 0;
    ReadReg16(0x03, &ret);
    if(ret != DRV8323_REG_DEFAULT_0x03) bus_fault_flag = 1;
    ReadReg16(0x04, &ret);
    if(ret != DRV8323_REG_DEFAULT_0x04) bus_fault_flag = 1;
    ReadReg16(0x05, &ret);
    if(ret != DRV8323_REG_DEFAULT_0x05) bus_fault_flag = 1;
    ReadReg16(0x06, &ret);
    if(ret != DRV8323_REG_DEFAULT_0x06) bus_fault_flag = 1;
    if(bus_fault_flag) return false;
    // Idrive settings
    WriteReg16(0x03, reg_hs_value);
    WriteReg16(0x04, reg_ls_value);
    // Calibrate Process
    DelayMs(10);
    WriteReg16(0x06, DRV8323_REG_CAL_0x06);
    DelayMs(20);
    WriteReg16(0x06, DRV8323_REG_DEFAULT_0x06);
    // Init PWM
    return TIMInit(initCNT);
}

template <class T>
void DriverDRV8323Base<T>::nFAULT_IRQHandler()
{
    bool state = ReadFAULT();
    for(uint8_t i = 0; i < 3; i++)
    {
        if(state != ReadFAULT()) return;
    }
    if(state == false) nFAULT_flag = 1;
    else nFAULT_flag = 0;
}

template <class T>
uint16_t DriverDRV8323Base<T>::ReadReg16(uint16_t reg, uint16_t *data)
{
    SetCS(0);
    uint16_t addr = 0x8000 | (reg << 11);
    DelayUs(500);
    uint16_t state = SPIRead16(addr, data);
    DelayUs(500);
    SetCS(1);
    DelayUs(500);
    return state;
}

template <class T>
uint16_t DriverDRV8323Base<T>::WriteReg16(uint16_t reg, uint16_t data)
{
    SetCS(0);
    uint16_t addr = (reg << 11) | (data & 0x7FF);
    DelayUs(500);
    uint16_t state = SPIRead16(addr, &data);
    DelayUs(500);
    SetCS(1);
    DelayUs(500);
    return state;
}

#endif