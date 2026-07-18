#pragma once

#include "encoder_base.hpp"

namespace iFOC::Encoder
{
class EncoderVernier final : public EncoderBase
{
    DELETE_COPY_CONSTRUCTOR(EncoderVernier);
public:
    // Copy the primary_enc, own the vernier_enc
    EncoderVernier(EncoderBase* p, EncoderBase* v, float p_gear, float v_gear);
    ~EncoderVernier() override;
    FuncRetCode Init(uint8_t motor_id) override;
    void UpdateRT(float Ts) override;
    void UpdateMid(float Ts) override;
    [[nodiscard]] float GetGearRatio() const;
private:
    EncoderBase* primary_enc;
    EncoderBase* vernier_enc;
    float gear_ratio = 1.0f;
};
}
