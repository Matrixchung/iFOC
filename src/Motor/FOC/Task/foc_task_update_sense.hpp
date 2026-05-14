#pragma once

#include "../../../Common/Filter/sliding_filter.hpp"
#include "../foc_motor.hpp"

namespace iFOC
{
class UpdateSenseTask final : public Task
{
public:
    UpdateSenseTask();
    void UpdateRT(float Ts) override;
    void UpdateNormal() override;
private:
    static constexpr size_t SHUNT_IMBAL_SUM_WINDOW_SIZE = 100;
    Filter::LowpassFilter Iabc_abs_sum_filter{10000.0f};
    Filter::SlidingFilter Ia_abs_filter{SHUNT_IMBAL_SUM_WINDOW_SIZE};
    Filter::SlidingFilter Ib_abs_filter{SHUNT_IMBAL_SUM_WINDOW_SIZE};
    Filter::SlidingFilter Ic_abs_filter{SHUNT_IMBAL_SUM_WINDOW_SIZE};
    std::array<float, SHUNT_IMBAL_SUM_WINDOW_SIZE> Iabc_abs_sum_window{};

    float Iabc_abs_sum_total = 0.0f;
    size_t shunt_imbal_det_counter = 0;
    size_t shunt_imbal_det_window = 0;
    size_t shunt_imbal_det_interp_idx = 0;

    uint8_t overcurrent_tick = 0;
    uint8_t calibration_timeout_ms = 0;
    uint8_t temperature_sense_tick = 0;
};
}