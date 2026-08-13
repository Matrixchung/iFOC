#pragma once

#include "foc_driver_base.hpp"
#include "../../Common/Interface/spi_base.hpp"
#include "../../Common/Interface/gpio_base.hpp"

namespace iFOC::Driver
{
class FOCDriverDRV8316 final : public FOCDriverBase
{
    using SPIBase = HAL::SPIBase;
    using GPIOBase = HAL::GPIOBase;
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(FOCDriverDRV8316);
public:
    FOCDriverDRV8316(FOCDriverBase *_pwm, SPIBase *_spi);
    FOCDriverDRV8316(FOCDriverBase *_pwm, SPIBase *_spi, GPIOBase *_nsleep, GPIOBase *_drvoff);
    FOCDriverDRV8316() = delete;
    void OnFaultIRQ();
    FuncRetCode Init(bool initCNT) override;
    void SetOutput3CHRaw(const uint32_t ch1, const uint32_t ch2, const uint32_t ch3) override { pwm_base->SetOutput3CHRaw(ch1, ch2, ch3); };
    void EnableAllOutputs() override { pwm_base->EnableAllOutputs(); if(drvoff) drvoff->Clear(); };
    void DisableAllOutputs() override { pwm_base->DisableAllOutputs(); if(drvoff) drvoff->Set(); };
    void EnableBridge(const Bridge bridge) override { pwm_base->EnableBridge(bridge); };
    void DisableBridge(const Bridge bridge) override { pwm_base->DisableBridge(bridge); };
    real_t GetDeadTime() override { return pwm_base->GetDeadTime(); };
private:
    typedef enum reg_address
    {
        IC_STATUS = (0x0 << 9),
        STATUS_1 = (0x1 << 9),
        STATUS_2 = (0x2 << 9),
        CONTROL_1 = (0x3 << 9),
        CONTROL_2 = (0x4 << 9),
        CONTROL_3 = (0x5 << 9),
        CONTROL_4 = (0x6 << 9),
        CONTROL_5 = (0x7 << 9),
        CONTROL_6 = (0x8 << 9),
        CONTROL_10 = (0xC << 9),
    } reg_address;
    typedef union ic_status
    {
        uint8_t reg{};
        struct
        {
            uint8_t fault             : 1; /* [0] */
            uint8_t overtemp          : 1; /* [1] */
            uint8_t bus_overvoltage   : 1; /* [2] */
            uint8_t no_power_on_reset : 1; /* [3] */
            uint8_t overcurrent       : 1; /* [4] */
            uint8_t spi_fault         : 1; /* [5] */
            uint8_t buck_fault        : 1; /* [6] */
            uint8_t                   : 1; /* [7] */
        } bit;
    } ic_status;
    typedef union status_1
    {
        uint8_t reg{};
        struct
        {
            uint8_t phase_a_l_oc      : 1; /* [0] */
            uint8_t phase_a_h_oc      : 1; /* [1] */
            uint8_t phase_b_l_oc      : 1; /* [2] */
            uint8_t phase_b_h_oc      : 1; /* [3] */
            uint8_t phase_c_l_oc      : 1; /* [4] */
            uint8_t phase_c_h_oc      : 1; /* [5] */
            uint8_t overtemp_shutdown : 1; /* [6] */
            uint8_t overtemp_warn     : 1; /* [7] */
        } bit;
    } status_1;
    typedef union status_2
    {
        uint8_t reg{};
        struct
        {
            uint8_t spi_addr_err      : 1; /* [0] */
            uint8_t spi_clock_err     : 1; /* [1] */
            uint8_t spi_parity_err    : 1; /* [2] */
            uint8_t vcp_undervoltage  : 1; /* [3] */
            uint8_t buck_undervoltage : 1; /* [4] */
            uint8_t buck_overcurrent  : 1; /* [5] */
            uint8_t otp_err           : 1; /* [6] */
            uint8_t                   : 1; /* [7] */
        } bit;
    } status_2;
    typedef union control_1
    {
        uint8_t reg{};
        struct
        {
            uint8_t reg_lock : 3; /* [0:2] */
            uint8_t          : 5; /* [3:7] */
        } bit;
    } control_1;
    typedef union control_2
    {
        uint8_t reg{};
        struct
        {
            uint8_t clear_fault : 1; /* [0] */
            uint8_t pwm_mode    : 2; /* [1:2] */
            uint8_t slew_rate   : 2; /* [3:4] */
            uint8_t sdo_mode    : 1; /* [5] */
            uint8_t             : 2; /* [6:7] */
        } bit;
    } control_2;
    typedef union control_3
    {
        uint8_t reg{};
        struct
        {
            uint8_t overtemp_report       : 1; /* [0] */
            uint8_t                       : 1; /* [1] */ // reserved for DRV8316, spi_fault_report for DRV8316C
            uint8_t overvoltage_prot_en   : 1; /* [2] */
            uint8_t overvoltage_level_sel : 1; /* [3] */
            uint8_t pwm_100_duty_freq_sel : 1; /* [4] */
            uint8_t                       : 3; /* [5:7] */
        } bit;
    } control_3;
    typedef union control_4
    {
        uint8_t reg{};
        struct
        {
            uint8_t ocp_mode     : 2; /* [0:1] */
            uint8_t ocp_level    : 1; /* [2] */
            uint8_t ocp_retry    : 1; /* [3] */
            uint8_t ocp_deg_time : 2; /* [4:5] */
            uint8_t ocp_cbc      : 1; /* [6] */
            uint8_t drv_off      : 1; /* [7] */
        } bit;
    } control_4;
    typedef union control_5
    {
        uint8_t reg{};
        struct
        {
            uint8_t csa_gain   : 2; /* [0:1] */
            uint8_t en_asr     : 1; /* [2] */
            uint8_t en_aar     : 1; /* [3] */
            uint8_t            : 2; /* [4:5] */
            uint8_t Ilim_recir : 1; /* [6] */
            uint8_t            : 1; /* [7] */
        } bit;
    } control_5;
    typedef union control_6
    {
        uint8_t reg{};
        struct
        {
            uint8_t buck_dis    : 1; /* [0] */
            uint8_t buck_sel    : 2; /* [1:2] */
            uint8_t buck_cl     : 1; /* [3] */
            uint8_t buck_ps_dis : 1; /* [4] */
            uint8_t             : 3; /* [5:7] */
        } bit;
    } control_6;
    typedef union control_10
    {
        uint8_t reg{};
        struct
        {
            uint8_t delay_target : 4; /* [0:3] */
            uint8_t delay_cmp_en : 1; /* [4] */
            uint8_t              : 3; /* [5:7] */
        } bit;
    } control_10;
    FuncRetCode WriteReg(const reg_address addr, const uint8_t data) const;
    FuncRetCode ReadReg(const reg_address addr, uint8_t *data) const;
    static uint16_t GetControlWord(const bool read, const reg_address reg_addr, const uint8_t data);
    FOCDriverBase *pwm_base = nullptr;
    SPIBase *spi_base = nullptr;
    GPIOBase *nsleep = nullptr;
    GPIOBase *drvoff = nullptr;
};
}
