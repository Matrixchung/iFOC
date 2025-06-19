#pragma once

#include "foc_types.hpp"

namespace iFOC
{
class TrajController
{
public:
    enum class TrajStage : uint8_t
    {
        ARRIVED = 0,
        ACCELERATING,
        CRUISING,
        DECELERATING
    };
    TrajController() = default;

    /// Plan a trapezoidal track (also known as T-shape), unit agreements example listed below.
    /// \param target_pos Target position, e.g. degree (all arguments' units are based on this)
    /// \param current_pos Current position, degree
    /// \param current_speed Current speed, degree/s
    /// \param cruise_speed Expected cruise speed (we may not reach cruise speed for short track), degree/s
    /// \param max_accel Expected max acceleration value, degree/s^2
    /// \param max_decel Expected max deceleration value, degree/s^2
    void PlanTrajectory(float target_pos, float current_pos, float current_speed,
                        float cruise_speed, float max_accel, float max_decel);

    /// Plan a simple acceleration from current_speed to cruise_speed.
    /// In this case the position starts from zero.
    /// \param current_speed Current speed, e.g. degree/s
    /// \param cruise_speed Expected cruise speed(max speed), degree/s
    /// \param time_to_accel Time to accelerate, s
    void PlanAcceleration(float current_speed, float cruise_speed, float time_to_accel);

    /// Reset the trajectory internal parameters.
    void Reset();

    /// Update the parameters, should be called frequently
    /// \param Ts Time delta, s
    void Update(float Ts);
    [[nodiscard]] __fast_inline float GetFinalPos() const { return final_pos; }
    [[nodiscard]] __fast_inline float GetCurrPos() const { return curr_pos; }
    [[nodiscard]] __fast_inline float GetCurrSpeed() const { return curr_speed; }
    [[nodiscard]] __fast_inline float GetCurrAccel() const { return curr_accel; }
    [[nodiscard]] __fast_inline bool GetState() const { return task_done; }
    [[nodiscard]] __fast_inline TrajStage GetTrajStage() const { return stage; }
private:
    float ref_accel = 0.0f;
    float ref_decel = 0.0f;
    float ref_speed = 0.0f;
    float accel_time = 0.0f;
    float decel_time = 0.0f;
    float cruise_time = 0.0f;
    float total_time = 0.0f;
    // total_time = Taccel + Tcruise + Tdecel
    float init_pos = 0.0f;
    float init_speed = 0.0f;
    float start_cruise_pos = 0.0f;

    float curr_pos = 0.0f;
    float curr_speed = 0.0f;
    float curr_accel = 0.0f;

    float final_pos = 0.0f;

    float state_timer = 0.0f;
    TrajStage stage = TrajStage::ARRIVED;
    bool task_done = true;
};
}