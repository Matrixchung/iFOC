#pragma once

#include "encoder_base.hpp"
#include "../Common/Interface/spi_base.hpp"
#include "../DataType/lookup_table.hpp"

namespace iFOC::Encoder
{
class EncoderMT6835 final : public EncoderBase
{
private:
    using SPIBase = HAL::SPIBase;
public:
    enum class DeviceError : uint8_t
    {
        NONE = 0,
        OVERSPEED = (1 << 0),
        MAG_FIELD_WEAK = (1 << 1),
        CORE_UVLO = (1 << 2),
        CRC_ERROR = (1 << 3),
    };
    explicit EncoderMT6835(SPIBase *_spi);
    ~EncoderMT6835() override;
    FuncRetCode Init(uint8_t motor_id) override;
    void UpdateRT(float Ts) override;
    void UpdateMid(float Ts) override;
    void UpdateLUT(const DataType::LookupTable& new_lut) override;
    std::underlying_type_t<DeviceError> device_error = to_underlying(DeviceError::NONE);
// private:
    FuncRetCode ReadAbsAngleRad();
    FuncRetCode WriteReg(uint16_t reg, uint8_t data) const;
    FuncRetCode ReadReg(uint16_t reg, uint8_t* data) const;
    FuncRetCode ReadAngleRegBurst(uint8_t *ret) const; // fixed length: 6
    FuncRetCode BurnEEPROM() const;
// private:
    static constexpr uint32_t CPR = 2097151; // 2^21 = 2097152, 2^21 - 1 = 2097151
    // static constexpr int CPRdiv2 = (CPR >> 1);
    static constexpr real_t CPR_f = (real_t)CPR;
    static constexpr real_t PI2divCPR_f = PI2 / CPR_f;
    DataType::LookupTable nonlinear_lut;
    SPIBase *spi = nullptr;
    real_t last_compensated_angle_rad = 0.0f;
    // uint32_t now_angle_cnt = 0;
    // uint32_t last_angle_cnt = 0;
    // uint8_t tx_buf[6] = {0x00};
    // uint8_t rx_buf[6] = {0x00};
    uint8_t startup_timer = 0;
};

}