#pragma once

#include "foc_driver_base.hpp"
#include "../../Common/Interface/spi_base.hpp"
#include "../../Common/Interface/gpio_base.hpp"

namespace iFOC::Driver
{
class FOCDriverDRV830x final : public FOCDriverBase
{
    using SPIBase = HAL::SPIBase;
    using GPIOBase = HAL::GPIOBase;
public:
    FOCDriverDRV830x(FOCDriverBase *_pwm, SPIBase *_spi);
    FOCDriverDRV830x(FOCDriverBase *_pwm, SPIBase *_spi, GPIOBase *_en);
    FOCDriverDRV830x() = delete;
    void OnFaultIRQ();
    FuncRetCode Init(bool initCNT) override;
    void SetOutput3CHRaw(const uint32_t ch1, const uint32_t ch2, const uint32_t ch3) override { pwm_base->SetOutput3CHRaw(ch1, ch2, ch3); };
    void EnableAllOutputs() override { pwm_base->EnableAllOutputs(); };
    void DisableAllOutputs() override { pwm_base->DisableAllOutputs(); };
    void EnableBridge(const Bridge bridge) override { pwm_base->EnableBridge(bridge); };
    void DisableBridge(const Bridge bridge) override { pwm_base->DisableBridge(bridge); };
    real_t GetDeadTime() override { return pwm_base->GetDeadTime(); };
private:
    typedef union status_1
    {
        uint16_t reg{};
        struct
        {
            uint16_t phase_c_l_oc   : 1; /* [0] */
            uint16_t phase_c_h_oc   : 1; /* [1] */
            uint16_t phase_b_l_oc   : 1; /* [2] */
            uint16_t phase_b_h_oc   : 1; /* [3] */
            uint16_t phase_a_l_oc   : 1; /* [4] */
            uint16_t phase_a_h_oc   : 1; /* [5] */
            uint16_t overtemp_warn  : 1; /* [6] */
            uint16_t overtemp_shtdn : 1; /* [7] */
            uint16_t pvdd_uv        : 1; /* [8] */
            uint16_t gvdd_uv        : 1; /* [9] */
            uint16_t fault          : 1; /* [10] */
            uint16_t                : 5; /* [15:11] */
        } bit;
    } status_1;
    typedef union status_2
    {
        uint16_t reg{};
        struct
        {
            uint16_t device_id      : 4; /* [3:0] */
            uint16_t                : 3; /* [6:4] */
            uint16_t gvdd_ov        : 1; /* [7] */
            uint16_t               : 8; /* [15:8] */
        } bit;
    } status_2;
    typedef union cr_1
    {
        uint16_t reg{};
        struct
        {
            uint16_t gate_current   : 2; /* [1:0] */
            uint16_t gate_reset     : 1; /* [2] */
            uint16_t pwm_mode       : 1; /* [3] */
            uint16_t ocp_mode       : 2; /* [5:4] */
            uint16_t oc_adj_set     : 5; /* [10:6] */
            uint16_t                : 5; /* [15:11] */
        } bit;
    } cr_1;
    typedef union cr_2
    {
        uint16_t reg{};
        struct
        {
            uint16_t octw_mode      : 2; /* [1:0] */
            uint16_t sense_gain     : 2; /* [3:2] */
            uint16_t dc_cal_ch1     : 1; /* [4] */
            uint16_t dc_cal_ch2     : 1; /* [5] */
            uint16_t oc_toff        : 1; /* [6] */
            uint16_t                : 9; /* [15:7] */
        } bit;
    } cr_2;
    FuncRetCode WriteReg(uint8_t reg, uint16_t data);
    FuncRetCode ReadReg(uint8_t reg, uint16_t *data);
    uint16_t SPITransfer(uint16_t tx);
    FuncRetCode SPIInit();
    FOCDriverBase *pwm_base = nullptr;
    SPIBase *spi_base = nullptr;
    GPIOBase *en_gate = nullptr;
};
}