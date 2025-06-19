#include "trajectory_controller.hpp"

namespace iFOC
{
void TrajController::PlanTrajectory(float target_pos, float current_pos, float current_speed,
                                    float cruise_speed, float max_accel, float max_decel)
{
    // To ensure responsiveness, current ongoing tasks are overrided.
    if(max_accel < 0.0f) max_accel = -max_accel;
    if(max_decel < 0.0f) max_decel = -max_decel;
    if(max_accel <= 0.1f || max_decel <= 0.1f) return;

    float dX = target_pos - current_pos;
    float min_stop_dist = (current_speed * current_speed) / (2.0f * max_decel); // minimum stopping distance from current_speed to 0
    float dX_stop = current_speed >= 0 ? min_stop_dist : -min_stop_dist; // if current_speed >= 0, then we need to move forward and decel, vise versa
    float s = (dX - dX_stop) >= 0 ? 1.0f : -1.0f; // sign
    ref_accel = s * max_accel;
    ref_decel = -s * max_decel;
    ref_speed = s * cruise_speed;

    // if current_speed > ref_speed, we need to decel first, and cruise, and decel again.
    if((s * current_speed) > (s * ref_speed)) ref_accel = -ref_accel;

    accel_time = (ref_speed - current_speed) / ref_accel;
    decel_time = -ref_speed / ref_decel; // make sure positive time

    // minimum distance required to reach ref cruising speed
    float dX_min = 0.5f * accel_time * (ref_speed + current_speed) + 0.5f * decel_time * ref_speed;

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

    task_done = false;
    stage = TrajStage::ACCELERATING;
    state_timer = 0.0f;
}

void TrajController::PlanAcceleration(float current_speed, float cruise_speed, float time_to_accel)
{
    if(time_to_accel <= 0.0f) return;
    float speed_delta = cruise_speed - current_speed; // can be positive or negative
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
    stage = TrajStage::ACCELERATING;
    state_timer = 0.0f;
}

void TrajController::Update(float Ts)
{
    if(!task_done)
    {
        state_timer += Ts;
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
            float Td = state_timer - total_time;
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
            stage = TrajStage::ARRIVED;
        }
    }
    else stage = TrajStage::ARRIVED;
}

void TrajController::Reset()
{
    task_done = true;
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
}

}