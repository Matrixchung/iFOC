#include "foc_driver_drv830x.hpp"

#include "board_config.hpp"
#include "foc_math.hpp"

/*
 * DRV830x SPI: CPOL = 0, CPHA = 1(2 Edge), MAX CLK = 10 MHz
 *
 * The SDI input data word is 16bits long and consists of:
 *  * 1 R/W bit W [15]
 *  * 4 addr bits A [14:11]
 *  * 11 data bits D [10:0]
 *
 * The SDO output data word is 16bits long and consists of:
 *  * 1 fault frame bit F [15]
 *  * 4 addr bits A [14:11]
 *  * 11 data bits D [10:0]
 */

namespace iFOC::Driver
{
FOCDriverDRV830x::FOCDriverDRV830x(FOCDriverBase* _pwm, SPIBase* _spi) : pwm_base(_pwm), spi_base(_spi) {}

FOCDriverDRV830x::FOCDriverDRV830x(FOCDriverBase* _pwm, SPIBase* _spi, GPIOBase *_en) : pwm_base(_pwm), spi_base(_spi), en_gate(_en) {}

void FOCDriverDRV830x::OnFaultIRQ()
{
    SPIInit(); // there might be other SPI devices with different CPOL/CPHA
    status_1 st1;
    status_2 st2;
    ReadReg(0x00, &st1.reg);
    ReadReg(0x01, &st2.reg);
}

FuncRetCode FOCDriverDRV830x::Init(bool initCNT)
{
    if(en_gate)
    {
        // reset sequence
        en_gate->ModeOutPP();
        en_gate->Clear();
        HAL::DelayMs(1);
        // en_gate->Set();
        // HAL::DelayMs(1);
        // en_gate->Clear();
    }
    HAL::DelayMs(5);
    // init spi first
    if(const auto r = SPIInit(); r != FuncRetCode::OK) return r;
    HAL::DelayMs(1);

    if(en_gate)
    {
        en_gate->Set();
        HAL::DelayMs(5);
    }
    else
    {
        // SPI_RESET command to CR1, if no EN_GATE pin presented
        WriteReg(0x02, 0x04);
        HAL::DelayMs(1);
    }
    status_1 st1{};
    status_2 st2{};
    cr_1 cr1{};
    cr_2 cr2{};
    cr_1 cr1_test{};
    cr1_test.bit.gate_current = 0x02;

    // read status (self-test)
    if(const auto r = ReadReg(0x00, &st1.reg); r != FuncRetCode::OK) return r;
    if(const auto r = ReadReg(0x01, &st2.reg); r != FuncRetCode::OK) return r;

    WriteReg(0x02, cr1_test.reg);
    if(const auto r = ReadReg(0x02, &cr1.reg); r != FuncRetCode::OK) return r;
    if(const auto r = ReadReg(0x03, &cr2.reg); r != FuncRetCode::OK) return r;

    // validate SPI R/W accessibility
    if(cr1.reg != cr1_test.reg)
    {
        return FuncRetCode::HARDWARE_ERROR;
    }
    // validate device id
    if(st2.bit.device_id != 0x01)
    {
        return FuncRetCode::CRC_MISMATCH;
    }
    // validate device state
    if(st1.reg != 0) // fault
    {
        return FuncRetCode::BUSY;
    }

    // cr1.reg = 0x00;
    // WriteReg(0x02, cr1.reg);
    // if(const auto r = ReadReg(0x02, &cr1.reg); r != FuncRetCode::OK) return r;

    // shunt amplifiers
    // #1: performing DC calibration (optional)
    cr2.reg = 0;
    cr2.bit.dc_cal_ch1 = 1;
    cr2.bit.dc_cal_ch2 = 1;
    WriteReg(0x03, cr2.reg);
    HAL::DelayMs(1);

    // #2: Deciding the GAIN value
    cr2.reg = 0;
    auto target_gain = (int)BoardConfig().GetConfig().current_sense_gain();
    switch(target_gain)
    {
        case 10:
        {
            cr2.bit.sense_gain = 0;
            break;
        }
        case 20:
        {
            cr2.bit.sense_gain = 1;
            break;
        }
        case 40:
        {
            cr2.bit.sense_gain = 2;
            break;
        }
        case 80:
        {
            cr2.bit.sense_gain = 3;
            break;
        }
        default: return FuncRetCode::PARAM_OUT_BOUND; // if GAIN out of bound, cr_2 will keep dc_1/2 shorted, as we want.
    }
    WriteReg(0x03, cr2.reg);

    // Here we double-check the written value
    cr_2 temp;
    if(const auto r = ReadReg(0x03, &temp.reg); r != FuncRetCode::OK) return r;
    if(temp.reg != cr2.reg) return FuncRetCode::CRC_MISMATCH;

    cr1.reg = 0;
    // cr1.bit.ocp_mode = 1; // OCP_MODE: OC latch shut down
    // cr1.bit.oc_adj_set = 21; // Vds approximately 0.730V?
    cr1.bit.oc_adj_set = 26; // Vds = 1.324V
    WriteReg(0x02, cr1.reg);

    // last stage: init counter
    if(const auto r = pwm_base->Init(initCNT); r != FuncRetCode::OK) return r;
    max_compare = pwm_base->max_compare;
    return FuncRetCode::OK;
}

FuncRetCode FOCDriverDRV830x::WriteReg(const uint8_t reg, const uint16_t data) const
{
    if(!(reg & 0x2)) return FuncRetCode::PARAM_OUT_BOUND;
    const uint16_t cmd = ((reg & 0x7) << 11) | (data & 0x7FF);
    SPITransfer(cmd);
    return FuncRetCode::OK;
}

FuncRetCode FOCDriverDRV830x::ReadReg(const uint8_t reg, uint16_t* data) const
{
    *data = 0;
    const uint16_t cmd = (1 << 15) | ((reg & 0x7) << 11);
    SPITransfer(cmd);
    const uint16_t temp = SPITransfer(0xFFFF);
    // #1: verify frame fault bit F [15]
    if(temp & (1 << 15)) return FuncRetCode::INVALID_RESULT;
    // #2: verify result address bits A [14:11]
    const uint8_t result_reg = (temp >> 11) & 0xF;
    if(result_reg != reg) return FuncRetCode::CRC_MISMATCH;
    // #3: store data result D [10:0]
    *data = temp & 0x7FF;
    return FuncRetCode::OK;
}

uint16_t FOCDriverDRV830x::SPITransfer(const uint16_t tx) const
{
    const uint16_t tx_data = (tx << 8) | (tx >> 8); // shift bits (little-endian)
    uint16_t rx_data = 0;
    spi_base->SetCS(false);
    HAL::DelayUs(5);
    if(const auto r = spi_base->WriteReadBytes((const uint8_t*)&tx_data, (uint8_t*)&rx_data, 2); r != FuncRetCode::OK)
    {
        spi_base->SetCS(true); // fixed nCS not pulled high if return state error
        return 0;
    }
    spi_base->SetCS(true);
    HAL::DelayUs(5);
    return (rx_data << 8) | (rx_data >> 8);
}

FuncRetCode FOCDriverDRV830x::SPIInit() const
{
    spi_base->SetClock(800000);
    spi_base->SetDataWidth(SPIBase::DataWidth::BYTE);
    spi_base->SetCPOLCPHA(0, 1);
    spi_base->SetCS(true);
    return spi_base->Init();
}
}
