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
    FuncRetCode Init(bool initCNT) override;
    void SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3) override { pwm_base->SetOutput3CHRaw(ch1, ch2, ch3); };
    void EnableAllOutputs() override { pwm_base->EnableAllOutputs(); };
    void DisableAllOutputs() override { pwm_base->DisableAllOutputs(); };
    void EnableBridge(Bridge bridge) override { pwm_base->EnableBridge(bridge); };
    void DisableBridge(Bridge bridge) override { pwm_base->DisableBridge(bridge); };
    real_t GetDeadTime() override { return pwm_base->GetDeadTime(); };
    union
    {
        uint16_t status_1{};
        struct
        {
            uint8_t phase_c_l_oc   : 1; /* [0] */
            uint8_t phase_c_h_oc   : 1; /* [1] */
            uint8_t phase_b_l_oc   : 1; /* [2] */
            uint8_t phase_b_h_oc   : 1; /* [3] */
            uint8_t phase_a_l_oc   : 1; /* [4] */
            uint8_t phase_a_h_oc   : 1; /* [5] */
            uint8_t overtemp_warn  : 1; /* [6] */
            uint8_t overtemp_shtdn : 1; /* [7] */
            uint8_t pvdd_uv        : 1; /* [8] */
            uint8_t gvdd_uv        : 1; /* [9] */
            uint8_t fault          : 1; /* [10] */
            uint8_t                : 5; /* [15:11] */
        } status_1_bit;
    };
    union
    {
        uint16_t status_2{};
        struct
        {
            uint8_t device_id      : 4; /* [3:0] */
            uint8_t                : 3; /* [6:4] */
            uint8_t gvdd_ov        : 1; /* [7] */
            uint16_t               : 8; /* [15:8] */
        } status_2_bit;
    };
    union
    {
        uint16_t cr_1{};
        struct
        {
            uint8_t gate_current   : 2; /* [1:0] */
            uint8_t gate_reset     : 1; /* [2] */
            uint8_t pwm_mode       : 1; /* [3] */
            uint8_t ocp_mode       : 2; /* [5:4] */
            uint8_t oc_adj_set     : 5; /* [10:6] */
            uint8_t                : 5; /* [15:11] */
        } cr_1_bit;
    };
    union
    {
        uint16_t cr_2{};
        struct
        {
            uint8_t octw_mode      : 2; /* [1:0] */
            uint8_t sense_gain     : 2; /* [3:2] */
            uint8_t dc_cal_ch1     : 1; /* [4] */
            uint8_t dc_cal_ch2     : 1; /* [5] */
            uint8_t oc_toff        : 1; /* [6] */
            uint8_t                : 9; /* [15:7] */
        } cr_2_bit;
    };
private:
    FuncRetCode WriteReg(uint8_t reg, uint16_t data);
    FuncRetCode ReadReg(uint8_t reg, uint16_t *data);
    uint16_t SPITransfer(uint16_t tx);
    FOCDriverBase *pwm_base = nullptr;
    SPIBase *spi_base = nullptr;
    GPIOBase *en_gate = nullptr;
};
}