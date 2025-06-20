#include "encoder_mt6835.hpp"

namespace iFOC::Encoder
{
EncoderMT6835::EncoderMT6835(SPIBase *_spi, GPIOBase *_gpio) : EncoderBase("MT6835", Type::ABSOLUTE_ENCODER, 1), spi(_spi), cal_gpio(_gpio) {};
EncoderMT6835::EncoderMT6835(SPIBase *_spi) : EncoderMT6835(_spi, nullptr) {};

EncoderMT6835::~EncoderMT6835()
{

}

// MT6835 SPI: CPOL = 1, CPHA = 1(2 Edge), MAX CLK = 16 MHz
// Write: 4 bit COMMAND + 12 bit REGISTER + 8 bit DATA
// Read:  --------------------------------+ 8 bit RETR

FuncRetCode EncoderMT6835::Init()
{
    result_valid = false;
    spi->SetCPOLCPHA(1, 1);
    spi->SetDataWidth(SPIBase::DataWidth::BYTE);
    spi->SetClock(24000000); // using 24 MHz MAX, decrease if communication error persists
    if(const auto r = spi->Init(); r != FuncRetCode::OK) return r;
    uint8_t temp = 0;
    // Step #1: Read System Bandwidth Register (0x011), as chip detection
    if(const auto r = ReadReg(0x011, &temp); r != FuncRetCode::OK) return r;
    if(temp != 0x5) return FuncRetCode::HARDWARE_ERROR; // Communication Error
    // Step #2: Turn off ABZ output at Register 0x008
    temp = (1 << 1);
    if(const auto r = WriteReg(0x008, temp); r != FuncRetCode::OK) return r;
    // Step #3: Turn off UVM output at Register 0x00B
    temp = (1 << 4);
    if(const auto r = WriteReg(0x00B, temp); r != FuncRetCode::OK) return r;
    // Step #4: Switch PWM_FQ to lower freq (497Hz), and set PWM_SEL to "speed data" at Register 0x00C
    //          to minimize pin crosstalk noise.
    temp = 0x00;
    if(const auto r = ReadReg(0x00C, &temp); r != FuncRetCode::OK) return r;
    temp |= (1 << 4); // Set PWM_FQ to 0x1 (497Hz)
    temp &= 0xF8;     // Clear [2:0] to 0x0
    temp |= (1 << 1); // Set PWM_SEL[2:0] to 0x2
    if(const auto r = WriteReg(0x00C, temp); r != FuncRetCode::OK) return r;
    // Step #5: Try to read angle
    if(const auto r = ReadAbsAngleRad(); r != FuncRetCode::OK) return r;
    last_angle_cnt = now_angle_cnt;
    multi_round_angle_rad = single_round_angle_rad;
    result_valid = true;
    return FuncRetCode::OK;
}

void EncoderMT6835::UpdateRT(float Ts)
{
    // Step #1: We get single_round_angle_rad & now_angle_cnt
    ReadAbsAngleRad();
}

void EncoderMT6835::UpdateMid(float Ts)
{
    // Step #2: Calculate delta
    int delta = (int)now_angle_cnt - (int)last_angle_cnt;
    last_angle_cnt = now_angle_cnt;
    // Step #3: Calculate full_rotations and multi_round_angle_rad
    if(delta > CPRdiv2)
    {
        full_rotations--;
        delta -= CPR;
    }
    else if(delta < -CPRdiv2)
    {
        full_rotations++;
        delta += CPR;
    }
    multi_round_angle_rad = full_rotations * PI2 + single_round_angle_rad;
    if(startup_timer <= 10)
    {
        startup_timer++;
    }
    else
    {
        // Step #4: Calculate velocity
        // real_t vel = (multi_round_angle_rad - last_multi_round_angle_rad) / Ts;
        real_t vel = ((real_t)delta * PI2divCPR_f) / Ts;
        angular_speed_rad_s = speed_lpf.GetOutput(vel, Ts);
    }
}

FuncRetCode EncoderMT6835::ReadAbsAngleRad()
{
    // uint8_t ret[6] = {0x00}; // length: 6 for burst read (see datasheet and ReadAngleRegBurst())
    // for(uint8_t i = 0; i < 4; i++)
    // {
    //     if(auto r = ReadReg(0x003 + i, &ret[i + 2]); r != FuncRetCode::OK) return r;
    // }
    ReadAngleRegBurst(rx_buf);
    uint8_t _get_crc = get_crc8(rx_buf + 2, 3);
    if(_get_crc == rx_buf[5])
    {
        device_error &= ~(to_underlying(DeviceError::CRC_ERROR)); // CRC passed
        now_angle_cnt = (uint32_t)(rx_buf[2] << 13) | (uint32_t)(rx_buf[3] << 5) | (uint32_t)(rx_buf[4] >> 3); // 21 bit angle
        if(sign_and_deduction_ratio < 0.0f) now_angle_cnt = CPR - now_angle_cnt;
        // single_round_angle_rad = (float)angle / CPR_f;
        // single_round_angle_rad *= PI2;
        single_round_angle_rad = (float)now_angle_cnt * PI2divCPR_f;
        device_error = (device_error & 0xF8) | (rx_buf[4] & 0x07);
        return FuncRetCode::OK;
    }
    device_error |= to_underlying(DeviceError::CRC_ERROR);
    return FuncRetCode::CRC_MISMATCH;
}

FuncRetCode EncoderMT6835::WriteReg(uint16_t reg, uint8_t data)
{
    // uint8_t tx_buf[3];
    tx_buf[0] = (0x06 << 4) | (uint8_t)(reg >> 12);
    tx_buf[1] = (uint8_t)reg;
    tx_buf[2] = data;
    return spi->WriteBytes(tx_buf, 3);
}

FuncRetCode EncoderMT6835::ReadReg(uint16_t reg, uint8_t* data)
{
    // uint8_t tx_buf[3];
    // uint8_t rx_buf[3] = {0x00};
    tx_buf[0] = (0x03 << 4) | (uint8_t)(reg >> 12);
    tx_buf[1] = (uint8_t)reg;
    tx_buf[2] = 0x00;
    *data = 0x00;
    if(const auto result = spi->WriteReadBytes(tx_buf, rx_buf, 3); result != FuncRetCode::OK) return result;
    *data = rx_buf[2];
    return FuncRetCode::OK;
}

__fast_inline FuncRetCode EncoderMT6835::ReadAngleRegBurst(uint8_t *ret)
{
    // uint8_t tx_buf[6] = {0x00};
    tx_buf[0] = (0x0A << 4); // Burst read angle register
    tx_buf[1] = 0x03;
    return spi->WriteReadBytes(tx_buf, ret, 6);
}

FuncRetCode EncoderMT6835::BurnEEPROM()
{
    // uint8_t tx_buf[3];
    // uint8_t rx_buf[3] = {0x00};
    tx_buf[0] = (0x0C << 4);
    tx_buf[1] = 0;
    tx_buf[2] = 0;
    if(const auto result = spi->WriteReadBytes(tx_buf, rx_buf, 3); result != FuncRetCode::OK) return result;
    if(rx_buf[2] == 0x55) return FuncRetCode::OK;
    return FuncRetCode::HARDWARE_ERROR;
}

FuncRetCode EncoderMT6835::GetSelfCalibrationState(SelfCalibState& ret)
{
    uint8_t data = 0;
    ret = SelfCalibState::NO_CALIB;
    // 0x113[7:6]
    if(const auto result = ReadReg(0x113, &data); result != FuncRetCode::OK) return result;
    data >>= 6;
    switch(data)
    {
        case to_underlying(SelfCalibState::CALIB_ONGOING): ret = SelfCalibState::CALIB_ONGOING; break;
        case to_underlying(SelfCalibState::CALIB_FAILED): ret = SelfCalibState::CALIB_FAILED; break;
        case to_underlying(SelfCalibState::CALIB_SUCCESS): ret = SelfCalibState::CALIB_SUCCESS; break;
        default: break;
    }
    return FuncRetCode::OK;
}

FuncRetCode EncoderMT6835::SetSelfCalibrationRPM(real_t rpm)
{
    if(rpm >= 6400.0f || rpm < 25.0f) return FuncRetCode::NOT_SUPPORTED;
    uint8_t original_reg = 0x00;
    if(const auto result = ReadReg(0x00E, &original_reg); result != FuncRetCode::OK) return result;
    uint8_t original_cal_freq = (original_reg & 0x70) >> 4;
    if(original_cal_freq != 0x03) return FuncRetCode::NOT_SUPPORTED;
    original_reg &= 0x8F; // ignore [6:4]
    uint8_t reg = 0x03;
    if(rpm >= 3200.0f) reg = 0x00;
    else if(rpm >= 1600.0f) reg = 0x01;
    else if(rpm >= 800.0f) reg = 0x02;
    else if(rpm >= 400.0f) reg = 0x03;
    else if(rpm >= 200.0f) reg = 0x04;
    else if(rpm >= 100.0f) reg = 0x05;
    else if(rpm >= 50.0f) reg = 0x06;
    else if(rpm >= 25.0f) reg = 0x07;
    original_reg |= (reg << 4);
    if(const auto result = WriteReg(0x00E, original_reg); result != FuncRetCode::OK) return result;
    return FuncRetCode::OK;
}

}
