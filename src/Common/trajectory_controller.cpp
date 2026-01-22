#include "trajectory_controller.hpp"

namespace iFOC
{
void TrajController::PlanTrajectory(const float target_pos, const float current_pos, const float current_speed,
                                    const float cruise_speed, float max_accel, float max_decel, const bool is_s_curve)
{
    // To ensure responsiveness, current ongoing tasks are overrided.
    if(max_accel < 0.0f) max_accel = -max_accel;
    if(max_decel < 0.0f) max_decel = -max_decel;
    if(max_accel <= 0.001f || max_decel <= 0.001f) return;

    const float dX = target_pos - current_pos;
    const float min_stop_dist = (current_speed * current_speed) / (2.0f * max_decel); // minimum stopping distance from current_speed to 0
    const float dX_stop = current_speed >= 0 ? min_stop_dist : -min_stop_dist; // if current_speed >= 0, then we need to move forward and decel, vise versa
    const float s = (dX - dX_stop) >= 0 ? 1.0f : -1.0f; // sign
    ref_accel = s * max_accel;
    ref_decel = -s * max_decel;
    ref_speed = s * cruise_speed;

    // if current_speed > ref_speed, we need to decel first, and cruise, and decel again.
    if((s * current_speed) > (s * ref_speed)) ref_accel = -ref_accel;

    accel_time = (ref_speed - current_speed) / ref_accel;
    decel_time = -ref_speed / ref_decel; // make sure positive time

    // minimum distance required to reach ref cruising speed
    const float dX_min = 0.5f * accel_time * (ref_speed + current_speed) + 0.5f * decel_time * ref_speed;

    if(s * dX < s * dX_min)
    {
        // No enough distance to accel - cruise - decel, we just use triangle profile (accel - decel)
        ref_speed = s * std::sqrt(std::max((ref_decel * current_speed * current_speed + 2.0f * ref_accel * ref_decel * dX) / (ref_decel - ref_accel), 0.0f));
        accel_time = std::max((ref_speed - current_speed) / ref_accel, 0.0f);
        decel_time = std::max(-ref_speed / ref_decel, 0.0f);
        cruise_time = 0.0f; // No time for cruising.
    }
    else
    {
        cruise_time = (dX - dX_min) / ref_speed;
    }

    total_time = accel_time + cruise_time + decel_time;
    init_pos = current_pos;
    init_speed = current_speed;
    final_pos = target_pos;
    start_cruise_pos = current_pos + current_speed * accel_time + 0.5f * ref_accel * (accel_time * accel_time); // pos at start of cruising phase

    s_curve = is_s_curve;
    if(s_curve)
    {
        // S-Shaped acceleration
        A1 = 1.0f * (ref_speed - init_speed);
        B1 = 3.0f * (init_speed - ref_speed);
        C1 = 2.5f * (ref_speed - init_speed);
        F1 = init_speed;

        // S-Shaped deceleration
        A2 = 1.0f * (0.0f - ref_speed);
        B2 = 3.0f * (ref_speed - 0.0f);
        C2 = 2.5f * (0.0f - ref_speed);
        F2 = ref_speed;
    }

    task_done = false;
    stage = TrajStage::ACCELERATING;
    state_timer = 0.0f;
}

void TrajController::PlanAcceleration(float current_speed, float cruise_speed, float time_to_accel)
{
    if(time_to_accel <= 0.0f) return;
    const float speed_delta = cruise_speed - current_speed; // can be positive or negative
    ref_accel = speed_delta / time_to_accel;
    init_pos = 0.0f;
    init_speed = current_speed;
    accel_time = time_to_accel;
    cruise_time = time_to_accel;
    ref_speed = cruise_speed;
    start_cruise_pos = current_speed * accel_time + 0.5f * ref_accel * (accel_time * accel_time);
    final_pos = start_cruise_pos + ref_speed * cruise_time;
    decel_time = 0.0f;
    total_time = accel_time + cruise_time;

    task_done = false;
    s_curve = false;
    stage = TrajStage::ACCELERATING;
    state_timer = 0.0f;
}

void TrajController::Update(const float Ts)
{
    if(!task_done)
    {
        state_timer += Ts;
        if(!s_curve) // T-Shape
        {
            if(state_timer < accel_time) // accelerating phase
            {
                stage = TrajStage::ACCELERATING;
                curr_pos = init_pos + init_speed * state_timer + 0.5f * ref_accel * (state_timer * state_timer);
                curr_speed = init_speed + ref_accel * state_timer;
                curr_accel = ref_accel;
            }
            else if(state_timer < accel_time + cruise_time) // cruising phase
            {
                stage = TrajStage::CRUISING;
                curr_pos = start_cruise_pos + ref_speed * (state_timer - accel_time);
                curr_speed = ref_speed;
                curr_accel = 0.0f;
            }
            else if(state_timer < total_time) // decelerating phase
            {
                stage = TrajStage::DECELERATING;
                const float Td = state_timer - total_time;
                curr_pos = final_pos + 0.5f * ref_decel * (Td * Td);
                curr_speed = ref_decel * Td;
                curr_accel = ref_decel;
            }
            else
            {
                curr_pos = final_pos;
                curr_speed = 0.0f;
                curr_accel = 0.0f;
                state_timer = 0.0f;
                task_done = true;
                s_curve = false;
                A1 = B1 = C1 = F1 = A2 = B2 = C2 = F2 = 0.0f;
                stage = TrajStage::ARRIVED;
            }
        }
        else
        {
            if(state_timer < accel_time) // accelerating phase
            {
                stage = TrajStage::ACCELERATING;
                const float t_ratio = state_timer / accel_time;
                const float t_ratio_pow2 = t_ratio * t_ratio;
                const float t_ratio_pow3 = t_ratio_pow2 * t_ratio;
                const float t_ratio_pow4 = t_ratio_pow3 * t_ratio;
                const float t_ratio_pow5 = t_ratio_pow4 * t_ratio;
                const float t_ratio_pow6 = t_ratio_pow5 * t_ratio;
                curr_pos = init_pos + accel_time * (A1 * t_ratio_pow6 + B1 * t_ratio_pow5 + C1 * t_ratio_pow4 + F1 * t_ratio);
                curr_speed = (6.0f * A1 * t_ratio_pow5 + 5.0f * B1 * t_ratio_pow4 + 4.0f * C1 * t_ratio_pow3 + F1);
                curr_accel = (5.0f * 6.0f * A1 * t_ratio_pow4 + 4.0f * 5.0f * B1 * t_ratio_pow3 + 3.0f * 4.0f * C1 * t_ratio_pow2) / accel_time;
            }
            else if(state_timer < accel_time + cruise_time) // cruising phase
            {
                stage = TrajStage::CRUISING;
                curr_pos = start_cruise_pos + ref_speed * (state_timer - accel_time);
                curr_speed = ref_speed;
                curr_accel = 0.0f;
            }
            else if(state_timer < total_time) // decelerating phase
            {
                stage = TrajStage::DECELERATING;
                const float Tb = accel_time + cruise_time;
                const float Tc = total_time;
                const float y = (state_timer - Tb) / (Tc - Tb);
                const float x_pow2 = y * y;
                const float x_pow3 = x_pow2 * y;
                const float x_pow4 = x_pow3 * y;
                const float x_pow5 = x_pow4 * y;
                const float x_pow6 = x_pow5 * y;
                curr_pos = start_cruise_pos + ref_speed * cruise_time + (Tc - Tb) * (A2 * x_pow6 + B2 * x_pow5 + C2 * x_pow4 + F2 * y);
                curr_speed = (6.0f * A2 * x_pow5 + 5.0f * B2 * x_pow4 + 4.0f * C2 * x_pow3 + F2);
                curr_accel = (5.0f * 6.0f * A2 * x_pow4 + 4.0f * 5.0f * B2 * x_pow3 + 3.0f * 4.0f * C2 * x_pow2) / (Tc - Tb);
            }
            else
            {
                curr_pos = final_pos;
                curr_speed = 0.0f;
                curr_accel = 0.0f;
                state_timer = 0.0f;
                task_done = true;
                s_curve = false;
                A1 = B1 = C1 = F1 = A2 = B2 = C2 = F2 = 0.0f;
                stage = TrajStage::ARRIVED;
            }
        }
    }
    else stage = TrajStage::ARRIVED;
}

void TrajController::DecelerateInAdvance()
{
    if(stage == TrajStage::CRUISING)
    {
        const float start_decelerate_pos = start_cruise_pos + ref_speed * cruise_time;
        const float subtract_pos = start_decelerate_pos - curr_pos;
        final_pos -= subtract_pos;
        state_timer = accel_time + cruise_time;
        stage = TrajStage::DECELERATING;
    }
}

void TrajController::Reset()
{
    task_done = true;
    s_curve = false;
    stage = TrajStage::ARRIVED;
    state_timer = 0.0f;
    ref_accel = 0.0f;
    ref_decel = 0.0f;
    ref_speed = 0.0f;
    accel_time = 0.0f;
    decel_time = 0.0f;
    cruise_time = 0.0f;
    total_time = 0.0f;
    init_pos = 0.0f;
    init_speed = 0.0f;
    start_cruise_pos = 0.0f;
    curr_pos = 0.0f;
    curr_speed = 0.0f;
    curr_accel = 0.0f;
    final_pos = 0.0f;
    A1 = B1 = C1 = F1 = A2 = B2 = C2 = F2 = 0.0f;
}

}