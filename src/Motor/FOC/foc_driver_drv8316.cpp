#include "foc_driver_drv8316.hpp"

#include "../../DataType/board_config.hpp"
#include "../../Common/foc_math.hpp"

// https://www.cnblogs.com/128d256/p/18816342

//! \brief Defines the address mask
//!
#define DRV8316_ADDR_MASK                   (0x7E00)
//! \brief Defines the data mask
//!
#define DRV8316_DATA_MASK                   (0x00FF)
//! \brief Defines the R/W mask
//!
#define DRV8316_RW_MASK                     (0x8000)

#define DRV8316_REG_UNLOCK_KEY              (0x03)
#define DRV8316_REG_LOCK_KEY                (0x06)

#define DRV8316_WAKEUP_TIME_US               (2000)
#define DRV8316_FAULT_POLL_INTERVAL_US       (100)
#define DRV8316_FAULT_POLL_COUNT             (50)

namespace iFOC::Driver
{
FOCDriverDRV8316::FOCDriverDRV8316(FOCDriverBase *_pwm, SPIBase *_spi, GPIOBase *_nsleep, GPIOBase *_drvoff) : pwm_base(_pwm), spi_base(_spi), nsleep(_nsleep), drvoff(_drvoff) {}

void FOCDriverDRV8316::OnFaultIRQ()
{
    if(drvoff) drvoff->Set();
}

FuncRetCode FOCDriverDRV8316::Init(bool initCNT)
{
    if(!pwm_base || !spi_base) return FuncRetCode::INVALID_INPUT;

    // Because Driver init() is called after CurrSense init(), we cannot change the config value,
    // only check the validity.
    // For DRV8316-board, we are forced to set the curr_sense_gain to 1.0f, with shunt_ohm as follows.
    // Here we should determine CSA_GAIN first:
    // the options for DRV8316 is 0.15V/A (0h), 0.3V/A (1h), 0.6V/A (2h), 1.2V/A (3h)
    // DRV8316 SOx = (Vref/2) ± GAIN * Ioutx, for bi-direction sensing with 3.3V ADC,
    // current sense range is: ±11A (0.15V/A), ±5.5A (0.3V/A), ±2.75A (0.6V/A) and ±1.375A (1.2V/A).
    // with given gain=1.0f, the corresponding shunt_ohm settings are (R=U/I):
    // 0.15Ohm (±11A, 0.15V/A), 0.3Ohm (±5.5A, 0.3V/A), 0.6Ohm (±2.75A, 0.6V/A), 1.2Ohm (±1.375A, 1.2V/A)
    const auto set_sense_gain = BoardConfig().GetConfig().current_sense_gain();
    if(set_sense_gain != 1.0f) return FuncRetCode::PARAM_OUT_BOUND;
    const auto set_shunt_ohm = BoardConfig().GetConfig().current_sense_shunt_ohm();
    uint8_t CSA_GAIN = 0x00;
    if(set_shunt_ohm == 0.15f) CSA_GAIN = 0x00;
    else if(set_shunt_ohm == 0.3f) CSA_GAIN = 0x01;
    else if(set_shunt_ohm == 0.6f) CSA_GAIN = 0x02;
    else if(set_shunt_ohm == 1.2f) CSA_GAIN = 0x03;
    else return FuncRetCode::PARAM_OUT_BOUND;

    spi_base->SetClock(1000000); // 1MHz SPI clock
    spi_base->SetDataWidth(SPIBase::DataWidth::BYTE);
    spi_base->SetCPOLCPHA(0, 1);
    spi_base->SetCS(true);
    if(const auto r = spi_base->Init(); r != FuncRetCode::OK)
    {
        // SPI init failed, try to disable gate & device
        if(nsleep)
        {
            nsleep->ModeOutPP();
            nsleep->Clear();
        }
        if(drvoff)
        {
            drvoff->ModeOutPP();
            drvoff->Set();
        }
        return r;
    }
    if(drvoff)
    {
        drvoff->ModeOutPP();
        drvoff->PullUp();
        drvoff->Set(); // disable drv for now
    }
    if(nsleep)
    {
        // Enter sleep fully, then wait for the device and SPI bus to wake up.
        nsleep->ModeOutPP();
        nsleep->Clear();
        HAL::DelayUs(DRV8316_WAKEUP_TIME_US);
        nsleep->Set();
    }
    HAL::DelayUs(DRV8316_WAKEUP_TIME_US);

    // Do not require FAULT=0 before CLR_FLT. A status fault may already be
    // latched here, so waiting for it to self-clear would block initialization.

    // #1: Unlock all registers by writing DRV8316_REG_UNLOCK_KEY to CONTROL_1
    {
        control_1 ctrl_1{};
        ctrl_1.bit.reg_lock = DRV8316_REG_UNLOCK_KEY;
        if(const auto r = WriteReg(CONTROL_1, ctrl_1.reg); r != FuncRetCode::OK) return r;
        // Read back
        if(const auto r = ReadReg(CONTROL_1, &ctrl_1.reg); r != FuncRetCode::OK) return r;
        if(ctrl_1.bit.reg_lock != DRV8316_REG_UNLOCK_KEY) return FuncRetCode::ACCESS_VIOLATION;
    }

    // #2: Control Register 2
    {
        control_2 ctrl_2{};
        if(const auto r = ReadReg(CONTROL_2, &ctrl_2.reg); r != FuncRetCode::OK) return r;
        // modify register
        ctrl_2.bit.sdo_mode = 1; // SDO IO in Push-Pull Mode
        ctrl_2.bit.slew_rate = 0x03; // Slew rate is 200 V/us
        ctrl_2.bit.pwm_mode = 0; // PWM 6x Mode
        ctrl_2.bit.clear_fault = 1; // Clear Fault, W1C
        if(const auto r = WriteReg(CONTROL_2, ctrl_2.reg); r != FuncRetCode::OK) return r;
        // Read back
        control_2 new_ctrl_2{};
        ctrl_2.bit.clear_fault = 0;
        if(const auto r = ReadReg(CONTROL_2, &new_ctrl_2.reg); r != FuncRetCode::OK) return r;
        if(new_ctrl_2.reg != ctrl_2.reg) return FuncRetCode::ACCESS_VIOLATION;
    }

    // #3: Control Register 3
    {
        control_3 ctrl_3{};
        if(const auto r = ReadReg(CONTROL_3, &ctrl_3.reg); r != FuncRetCode::OK) return r;
        // modify register
        ctrl_3.bit.pwm_100_duty_freq_sel = 0; // 100% PWM Duty Cycle: 20KHz
        ctrl_3.bit.overvoltage_level_sel = 0; // Vbus overvoltage level: 34V
        ctrl_3.bit.overvoltage_prot_en = 1; // Vbus overvoltage protection enabled
        ctrl_3.bit.overtemp_report = 1; // Overtemp reporting on nFAULT is enabled
        if(const auto r = WriteReg(CONTROL_3, ctrl_3.reg); r != FuncRetCode::OK) return r;
        // Read back
        control_3 new_ctrl_3{};
        if(const auto r = ReadReg(CONTROL_3, &new_ctrl_3.reg); r != FuncRetCode::OK) return r;
        if(ctrl_3.reg != new_ctrl_3.reg) return FuncRetCode::ACCESS_VIOLATION;
    }

    // #4: Control Register 4
    {
        control_4 ctrl_4{};
        if(const auto r = ReadReg(CONTROL_4, &ctrl_4.reg); r != FuncRetCode::OK) return r;
        // modify register
        ctrl_4.bit.drv_off = 0; // SPI DRV_OFF bit disabled
        ctrl_4.bit.ocp_cbc = 1; // OCP clearing in PWM cycle change: enabled
        ctrl_4.bit.ocp_deg_time = 0; // OCP deglitch time: 0.2us
        ctrl_4.bit.ocp_retry = 0; // OCP retry time: 5ms
        ctrl_4.bit.ocp_level = 0; // OCP level: 16A (far from normal operation range)
        ctrl_4.bit.ocp_mode = 0; // OCP causes a latched fault
        if(const auto r = WriteReg(CONTROL_4, ctrl_4.reg); r != FuncRetCode::OK) return r;
        // Read back
        control_4 new_ctrl_4{};
        if(const auto r = ReadReg(CONTROL_4, &new_ctrl_4.reg); r != FuncRetCode::OK) return r;
        if(ctrl_4.reg != new_ctrl_4.reg) return FuncRetCode::ACCESS_VIOLATION;
    }

    // #5: Control Register 5
    {
        control_5 ctrl_5{};
        if(const auto r = ReadReg(CONTROL_5, &ctrl_5.reg); r != FuncRetCode::OK) return r;
        // modify register
        ctrl_5.bit.Ilim_recir = 0; // Brake Mode
        ctrl_5.bit.en_aar = 0;
        ctrl_5.bit.en_asr = 0;
        ctrl_5.bit.csa_gain = CSA_GAIN;
        if(const auto r = WriteReg(CONTROL_5, ctrl_5.reg); r != FuncRetCode::OK) return r;
        // Read back
        control_5 new_ctrl_5{};
        if(const auto r = ReadReg(CONTROL_5, &new_ctrl_5.reg); r != FuncRetCode::OK) return r;
        if(ctrl_5.reg != new_ctrl_5.reg) return FuncRetCode::ACCESS_VIOLATION;
    }

    // #6: Control Register 6 (BUCK settings, leave it default)

    // #7: Control Register 10, leave default

    // init pwm
    if(const auto r = pwm_base->Init(initCNT); r != FuncRetCode::OK) return r;
    if(drvoff) drvoff->Clear(); // enable drv

    // DRVOFF transitions can assert nFAULT. Clear status once more after
    // releasing DRVOFF, then check for persistent faults.
    {
        control_2 ctrl_2{};
        if(const auto r = ReadReg(CONTROL_2, &ctrl_2.reg); r != FuncRetCode::OK)
        {
            DisableAllOutputs();
            return r;
        }
        ctrl_2.bit.clear_fault = 1;
        if(const auto r = WriteReg(CONTROL_2, ctrl_2.reg); r != FuncRetCode::OK)
        {
            DisableAllOutputs();
            return r;
        }
    }

    ic_status ic_stat{};
    bool fault_cleared = false;
    for(uint8_t retry = 0; retry < DRV8316_FAULT_POLL_COUNT; retry++)
    {
        if(const auto r = ReadReg(IC_STATUS, &ic_stat.reg); r != FuncRetCode::OK)
        {
            DisableAllOutputs();
            return r;
        }
        if(!ic_stat.bit.fault)
        {
            fault_cleared = true;
            break;
        }
        HAL::DelayUs(DRV8316_FAULT_POLL_INTERVAL_US);
    }
    if(!fault_cleared)
    {
        DisableAllOutputs();
        return FuncRetCode::HARDWARE_ERROR;
    }

    max_compare = pwm_base->max_compare;
    return FuncRetCode::OK;
}

FOCDriverDRV8316::FOCDriverDRV8316(FOCDriverBase *_pwm, SPIBase *_spi) : FOCDriverDRV8316(_pwm, _spi, nullptr, nullptr) {}

FuncRetCode FOCDriverDRV8316::WriteReg(const reg_address addr, const uint8_t data) const
{
    const uint16_t ctrl_word = GetControlWord(false, addr, data);
    const uint16_t tx_data = (ctrl_word << 8) | (ctrl_word >> 8); // shift bits (little-endian)

    uint16_t dummy = 0;
    // HAL::DelayUs(1);
    spi_base->SetCS(false);
    HAL::DelayUs(1);
    if(const auto r = spi_base->WriteReadBytes((const uint8_t*)&tx_data, (uint8_t*)&dummy, 2); r != FuncRetCode::OK)
    {
        spi_base->SetCS(true);
        return r;
    }
    HAL::DelayUs(1);
    return FuncRetCode::OK;
}

FuncRetCode FOCDriverDRV8316::ReadReg(const reg_address addr, uint8_t* data) const
{
    const uint16_t ctrl_word = GetControlWord(true, addr, 0);
    const uint16_t tx_data = (ctrl_word << 8) | (ctrl_word >> 8); // shift bits (little-endian)
    uint16_t rx_data = 0;
    *data = 0;
    // HAL::DelayUs(1);
    spi_base->SetCS(false);
    HAL::DelayUs(1);
    if(const auto r = spi_base->WriteReadBytes((const uint8_t*)&tx_data, (uint8_t*)&rx_data, 2); r != FuncRetCode::OK)
    {
        spi_base->SetCS(true);
        return r;
    }
    *data = rx_data >> 8;
    HAL::DelayUs(1);
    return FuncRetCode::OK;
}

uint16_t FOCDriverDRV8316::GetControlWord(const bool read, const reg_address reg_addr, const uint8_t data)
{
    const uint16_t p_addr = reg_addr;
    const uint16_t p_data = data;
    const uint16_t p_mode = read ? DRV8316_RW_MASK : 0;

    uint16_t calc = (p_mode & 0x8000) | (p_addr & DRV8316_ADDR_MASK) | (p_data & DRV8316_DATA_MASK);
    uint16_t parity = 0;
    while(calc)
    {
        parity ^= (calc & 1);
        calc >>= 1;
    }
    parity <<= 8;

    const uint16_t ctrl_word = p_mode | p_addr | parity | (p_data & DRV8316_DATA_MASK);

    return ctrl_word;
}

}
