#pragma once

#include "../foc_motor.hpp"
#include "../../../DataType/lookup_table.hpp"

namespace iFOC::FOC
{
class EncoderArbiterTask final : public Task
{
public:
    EncoderArbiterTask();
    void InitRT() override;
    void UpdateRT(float Ts) override;
private:
    DataType::LookupTable nonlinear_lut;
};
}