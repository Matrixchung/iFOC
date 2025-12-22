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
    // HAL::DelayMs(10);
    // init spi first
    spi_base->SetClock(800000);
    spi_base->SetDataWidth(SPIBase::DataWidth::BYTE);
    spi_base->SetCPOLCPHA(0, 1);
    spi_base->SetCS(true);
    if(const auto r = spi_base->Init(); r != FuncRetCode::OK) return r;
    HAL::DelayMs(1);

    if(en_gate)
    {
        en_gate->Set();
        HAL::DelayMs(1);
    }
    else
    {
        // SPI_RESET command to CR1, if no EN_GATE pin presented
        WriteReg(0x02, 0x04);
        HAL::DelayMs(1);
    }

    // read status (self-test)
    if(const auto r = ReadReg(0x00, &status_1); r != FuncRetCode::OK) return r;
    if(const auto r = ReadReg(0x01, &status_2); r != FuncRetCode::OK) return r;
    WriteReg(0x02, 0x02);
    if(const auto r = ReadReg(0x02, &cr_1); r != FuncRetCode::OK) return r;
    if(const auto r = ReadReg(0x03, &cr_2); r != FuncRetCode::OK) return r;

    // validate SPI R/W accessibility
    if(cr_1 != 0x02)
    {
        return FuncRetCode::HARDWARE_ERROR;
    }
    // validate device id
    if(status_2_bit.device_id != 0x01)
    {
        return FuncRetCode::CRC_MISMATCH;
    }
    // validate device state
    if(status_1 != 0) // fault
    {
        return FuncRetCode::BUSY;
    }

    WriteReg(0x02, 0x00);
    if(const auto r = ReadReg(0x02, &cr_1); r != FuncRetCode::OK) return r;

    // shunt amplifiers
    // #1: performing DC calibration (optional)
    cr_2 = 0;
    cr_2_bit.dc_cal_ch1 = 1;
    cr_2_bit.dc_cal_ch2 = 1;
    WriteReg(0x03, cr_2);
    HAL::DelayMs(1);

    // #2: Deciding the GAIN value
    cr_2 = 0;
    auto target_gain = (int)BoardConfig().GetConfig().current_sense_gain();
    switch(target_gain)
    {
        case 10:
        {
            cr_2_bit.sense_gain = 0;
            break;
        }
        case 20:
        {
            cr_2_bit.sense_gain = 1;
            break;
        }
        case 40:
        {
            cr_2_bit.sense_gain = 2;
            break;
        }
        case 80:
        {
            cr_2_bit.sense_gain = 3;
            break;
        }
        default: return FuncRetCode::PARAM_OUT_BOUND;
    }
    WriteReg(0x03, cr_2);

    // last stage: init counter
    if(const auto r = pwm_base->Init(initCNT); r != FuncRetCode::OK) return r;
    max_compare = pwm_base->max_compare;
    return FuncRetCode::OK;
}

FuncRetCode FOCDriverDRV830x::WriteReg(uint8_t reg, uint16_t data)
{
    if(!(reg & 0x2)) return FuncRetCode::PARAM_OUT_BOUND;
    uint16_t cmd = ((reg & 0x7) << 11) | (data & 0x7FF);
    SPITransfer(cmd);
    return FuncRetCode::OK;
}

FuncRetCode FOCDriverDRV830x::ReadReg(uint8_t reg, uint16_t* data)
{
    *data = 0;
    uint16_t cmd = (1 << 15) | ((reg & 0x7) << 11);
    SPITransfer(cmd);
    uint16_t temp = SPITransfer(0xFFFF);
    // #1: verify frame fault bit F [15]
    if(temp & (1 << 15)) return FuncRetCode::INVALID_RESULT;
    // #2: verify result address bits A [14:11]
    uint8_t result_reg = (temp >> 11) & 0xF;
    if(result_reg != reg) return FuncRetCode::CRC_MISMATCH;
    // #3: store data result D [10:0]
    *data = temp & 0x7FF;
    return FuncRetCode::OK;
}

uint16_t FOCDriverDRV830x::SPITransfer(uint16_t tx)
{
    uint16_t tx_data = (tx << 8) | (tx >> 8); // shift bits (little-endian)
    uint16_t rx_data = 0;
    spi_base->SetCS(false);
    HAL::DelayUs(5);
    if(const auto r = spi_base->WriteReadBytes((const uint8_t*)&tx_data, (uint8_t*)&rx_data, 2); r != FuncRetCode::OK) return 0;
    spi_base->SetCS(true);
    HAL::DelayUs(5);
    return (rx_data << 8) | (rx_data >> 8);
}
}
