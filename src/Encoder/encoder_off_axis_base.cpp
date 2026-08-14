#include "encoder_off_axis_base.hpp"

#include "../DataType/blob_nvm_storage.hpp"

namespace iFOC::Encoder
{
EncoderOffAxisBase::EncoderOffAxisBase() : EncoderBase("EncOffAxis", Type::ABSOLUTE_ENCODER, 1) {}

FuncRetCode EncoderOffAxisBase::Init(uint8_t motor_id)
{
    // try to read peak calibration result, "ofp0", "ofp1"...
    char key[sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX) + 1];
    memcpy(key, OFF_AXIS_PEAK_DB_KEY_PREFIX, sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX) - 1);
    key[sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX) - 1] = motor_id + '0';
    key[sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX)] = '\0';

    uint8_t buffer[sizeof(param_t) * 2 + sizeof(int8_t) + sizeof(uint8_t)]{};
    uint16_t size = sizeof(buffer);

    if(BlobNVMStorage().ReadNVM(key, buffer, &size) == FuncRetCode::OK && size == sizeof(buffer))
    {
        uint8_t calc_crc8 = get_crc8(buffer, sizeof(buffer) - sizeof(uint8_t));
        if(calc_crc8 == buffer[sizeof(buffer) - 1])
        {
            memcpy(&param_ChA, buffer, sizeof(param_t));
            memcpy(&param_ChB, buffer + sizeof(param_t), sizeof(param_t));
            int8_t sign = 1;
            memcpy(&sign, buffer + sizeof(param_t) * 2, sizeof(int8_t));
            SetSign(sign);
        }
    }

    // try to read LUT result, "ofl0", "ofl1" ...
    memcpy(key, OFF_AXIS_LUT_DB_KEY_PREFIX, sizeof(OFF_AXIS_LUT_DB_KEY_PREFIX) - 1);
    key[sizeof(OFF_AXIS_LUT_DB_KEY_PREFIX) - 1] = motor_id + '0';
    key[sizeof(OFF_AXIS_LUT_DB_KEY_PREFIX)] = '\0';

    auto buffer_size = BlobNVMStorage().GetKVSize(key);
    if(DataType::LookupTable::getTableSizeBySerializedSize(buffer_size) == OFF_AXIS_LUT_POINTS)
    {
        uint8_t* deserialize_buffer = (uint8_t*)pvPortMalloc(buffer_size * sizeof(uint8_t));
        if(deserialize_buffer)
        {
            if(BlobNVMStorage().ReadNVM(key, deserialize_buffer, &buffer_size) == FuncRetCode::OK)
            {
                offset_lut.deserialize(deserialize_buffer, buffer_size);
            }
            vPortFree(deserialize_buffer);
            deserialize_buffer = nullptr;
        }
    }
    else // incorrect size, delete KV
    {
        BlobNVMStorage().ClearNVM(key);
    }

    return FuncRetCode::OK;
}

void EncoderOffAxisBase::UpdateMid(const float Ts)
{
    result_valid = IsConnected() && IsCalibrated();

    raw_single_round_angle_rad = normalize_rad(GetAtan2());
    if(IsLUTCalibrated())
    {
        const float error = offset_lut.lookupPeriodic(raw_single_round_angle_rad);
        compensated_single_round_angle_rad = normalize_rad(raw_single_round_angle_rad + error);
    }
    else compensated_single_round_angle_rad = raw_single_round_angle_rad;

    real_t delta = compensated_single_round_angle_rad - last_compensated_angle_rad;
    last_compensated_angle_rad = compensated_single_round_angle_rad;

    if(delta > PI)
    {
        full_rotations--;
        delta -= PI2;
    }
    else if(delta < -PI)
    {
        full_rotations++;
        delta += PI2;
    }

    multi_round_angle_rad = full_rotations * PI2 + compensated_single_round_angle_rad;

    const real_t vel = delta / Ts;
    angular_speed_rad_s = speed_lpf.GetOutput(vel, Ts);

    if(IsConnected())
    {
        switch(calib_state)
        {
            case CalibrationState::PEAK:
            {
                // CH_A
                const auto ch_a = GetChannelA_mV();
                if(ch_a >= param_ChA.Vmax) param_ChA.Vmax = ch_a;
                if(ch_a <= param_ChA.Vmin) param_ChA.Vmin = ch_a;
                if(param_ChA.Vmax > param_ChA.Vmin) param_ChA.Vamp = 0.5f * (param_ChA.Vmax - param_ChA.Vmin);

                // CH_B
                const auto ch_b = GetChannelB_mV();
                if(ch_b >= param_ChB.Vmax) param_ChB.Vmax = ch_b;
                if(ch_b <= param_ChB.Vmin) param_ChB.Vmin = ch_b;
                if(param_ChB.Vmax > param_ChB.Vmin) param_ChB.Vamp = 0.5f * (param_ChB.Vmax - param_ChB.Vmin);
                break;
            }
            case CalibrationState::LUT:
            {
                break;
            }
            default: break;
        }
    }
}

void EncoderOffAxisBase::SaveConfig(uint8_t motor_id)
{
    static_assert(sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX) == sizeof(OFF_AXIS_LUT_DB_KEY_PREFIX));

    // try to save peak calibration result, "ofp0", "ofp1"...
    char key[sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX) + 1];
    memcpy(key, OFF_AXIS_PEAK_DB_KEY_PREFIX, sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX) - 1);
    key[sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX) - 1] = motor_id + '0';
    key[sizeof(OFF_AXIS_PEAK_DB_KEY_PREFIX)] = '\0';

    uint8_t buffer[sizeof(param_t) * 2 + sizeof(int8_t) + sizeof(uint8_t)]{};
    const int8_t sign = GetSign();
    memcpy(buffer, &param_ChA, sizeof(param_t));
    memcpy(buffer + sizeof(param_t), &param_ChB, sizeof(param_t));
    memcpy(buffer + sizeof(param_t) * 2, &sign, sizeof(int8_t));
    buffer[sizeof(buffer) - 1] = get_crc8(buffer, sizeof(buffer) - sizeof(uint8_t));
    BlobNVMStorage().SaveNVM(key, buffer, sizeof(buffer));

    // try to save LUT calibration result, "ofl0", "ofl1"...
    if(offset_lut.getTableSize() == OFF_AXIS_LUT_POINTS)
    {
        memcpy(key, OFF_AXIS_LUT_DB_KEY_PREFIX, sizeof(OFF_AXIS_LUT_DB_KEY_PREFIX) - 1);
        key[sizeof(OFF_AXIS_LUT_DB_KEY_PREFIX) - 1] = motor_id + '0';
        key[sizeof(OFF_AXIS_LUT_DB_KEY_PREFIX)] = '\0';
        auto lut_serialized_size = offset_lut.getSerializedSize();
        uint8_t* serialize_buffer = (uint8_t*)pvPortMalloc(lut_serialized_size * sizeof(uint8_t));
        if(serialize_buffer)
        {
            if(offset_lut.serialize(serialize_buffer, lut_serialized_size))
            {
                BlobNVMStorage().SaveNVM(key, serialize_buffer, lut_serialized_size);
            }
            vPortFree(serialize_buffer);
            serialize_buffer = nullptr;
        }
    }
}

void EncoderOffAxisBase::SetCalibrationMode(const CalibrationState state)
{
    calib_state = state;
}

std::pair<float, float> EncoderOffAxisBase::GetChannelA_MinMax_mV() const
{
    return {param_ChA.Vmin, param_ChA.Vmax};
}

std::pair<float, float> EncoderOffAxisBase::GetChannelB_MinMax_mV() const
{
    return {param_ChB.Vmin, param_ChB.Vmax};
}

float EncoderOffAxisBase::GetChannelA_Amplitude_mV() const
{
    return param_ChA.Vamp;
}

float EncoderOffAxisBase::GetChannelB_Amplitude_mV() const
{
    return param_ChB.Vamp;
}

float EncoderOffAxisBase::GetChannelA_Normed()
{
    if(!IsPeakCalibrated()) return 0.0f;
    return _constrain((GetChannelA_mV() - param_ChA.Vmin - param_ChA.Vamp) / (param_ChA.Vamp), -1.0f, 1.0f);
}

float EncoderOffAxisBase::GetChannelB_Normed()
{
    if(!IsPeakCalibrated()) return 0.0f;
    return _constrain((GetChannelB_mV() - param_ChB.Vmin - param_ChB.Vamp) / (param_ChB.Vamp), -1.0f, 1.0f);
}

float EncoderOffAxisBase::GetAtan2()
{
    if(!IsPeakCalibrated()) return 0.0f;
    const float angle = atan2f(GetChannelA_Normed(), GetChannelB_Normed());
    if(sign_and_deduction_ratio < 0.0f) return PI2 - angle;
    return angle;
}

bool EncoderOffAxisBase::IsCalibrated() const
{
    return calib_state == CalibrationState::NONE && IsPeakCalibrated() && IsLUTCalibrated();
}

bool EncoderOffAxisBase::IsPeakCalibrated() const
{
    return calib_state != CalibrationState::PEAK && param_ChA.Vamp >= 100.0f && param_ChB.Vamp >= 100.0f;
}

bool EncoderOffAxisBase::IsLUTCalibrated() const
{
    return offset_lut.getTableSize() == OFF_AXIS_LUT_POINTS;
}

}
