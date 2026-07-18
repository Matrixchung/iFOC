#include "encoder_vernier.hpp"

namespace iFOC::Encoder
{
    EncoderVernier::EncoderVernier(EncoderBase* p, EncoderBase* v, float p_gear, float v_gear) :
    EncoderBase("EncVernier", Type::ABSOLUTE_ENCODER, 1), primary_enc(p), vernier_enc(v)
    {
        if(p_gear > 0.0f && v_gear > 0.0f) gear_ratio = ABS(p_gear / v_gear);
    }

    EncoderVernier::~EncoderVernier()
    {
        if(vernier_enc) delete vernier_enc; // call destructor
    }

    FuncRetCode EncoderVernier::Init(const uint8_t motor_id)
    {
        // change encoder sign first
        vernier_enc->SetSign(primary_enc->GetSign());
        auto ret = vernier_enc->Init(motor_id);
        if(ret == FuncRetCode::OK) result_valid = vernier_enc->IsResultValid();
        else result_valid = false;
        return ret;
    }

    void EncoderVernier::UpdateRT(const float Ts)
    {
        vernier_enc->UpdateRT(Ts);
        raw_single_round_angle_rad = vernier_enc->raw_single_round_angle_rad;
        compensated_single_round_angle_rad = vernier_enc->compensated_single_round_angle_rad;
        result_valid = vernier_enc->IsResultValid();
    }

    void EncoderVernier::UpdateMid(float Ts)
    {
        vernier_enc->UpdateMid(Ts);
        multi_round_angle_rad = vernier_enc->multi_round_angle_rad;
        angular_speed_rad_s = vernier_enc->angular_speed_rad_s;
        full_rotations = vernier_enc->full_rotations;
    }

    float EncoderVernier::GetGearRatio() const
    {
        return gear_ratio;
    }
}

