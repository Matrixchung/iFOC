#pragma once

#include "../foc_motor.hpp"
#include "../../../Encoder/encoder_base.hpp"

namespace iFOC::FOC
{
class OpenLoopController final : public Task
{
    OVERRIDE_NEW();
public:
    OpenLoopController();
    ~OpenLoopController() override;
    void InitMid() override;
    void UpdateMid(float Ts) override;
private:
    class EncoderOpenLoop final : public Encoder::EncoderBase
    {
    public:
        friend class OpenLoopController;
        EncoderOpenLoop();
    };
    EncoderOpenLoop* encoder = nullptr;
};
}