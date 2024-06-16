#ifndef _FOC_MODULE_TRAJECTORY_CONTROLLER_HPP
#define _FOC_MODULE_TRAJECTORY_CONTROLLER_HPP

#include "cmath"
#include "module_base.hpp"

class TrajController : public ModuleBase
{
public:
    TrajController() {};
    void PlanTrajectory(float target_pos, float current_pos, float current_speed,
                        float cruise_speed, float max_accel, float max_decel); // pos all absolute
    void Preprocess(foc_state_input_t &in, foc_state_output_t &out, float Ts) final;
    void Reset();
    float final_pos = 0.0f;
    inline float GetFinalPos() { return final_pos; };
    inline float GetPos() { return set_pos; };
    inline float GetSpeed() { return set_speed; };
    inline float GetAccel() { return set_accel; };
    inline bool GetState() { return task_done; };
private:
    float ref_accel = 0.0f;   // Maximum Acceleration (signed)
    float ref_decel = 0.0f;   // Maximum Deceleration (signed)
    float ref_speed = 0.0f;   // Cruising Speed (signed)
    float accel_time = 0.0f;  // Time to accelerate from Current Speed to Cruising Speed
    float decel_time = 0.0f;  // Time to decelerate from Cruising Speed to 0
    float cruise_time = 0.0f; // Time spend in cruising stage
    float total_time = 0.0f;  // Taccel + Tcruise + Tdecel
    float init_pos = 0.0f;
    float init_speed = 0.0f;
    
    float start_cruise_pos = 0.0f;

    // The following X/dX/d(dX) (pos,speed,accel) are updated simultaneously, we can use any of them in different PID levels
    float set_pos = 0.0f; 
    float set_speed = 0.0f;
    float set_accel = 0.0f;
    bool task_done = true;

    float state_timer = 0.0f;
};

void TrajController::PlanTrajectory(float target_pos, float current_pos, float current_speed,
                                    float cruise_speed, float max_accel, float max_decel) // accel and decel should all be positive
{
    if(max_accel < 0.0f) max_accel = -max_accel;
    if(max_decel < 0.0f) max_decel = -max_decel; // make sure positive acc limits
    if(max_accel == 0.0f || max_decel == 0.0f) return;
    task_done = false;
    state_timer = 0.0f;
    float dX = target_pos - current_pos; // distance to travel
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
}

void TrajController::Preprocess(foc_state_input_t &in, foc_state_output_t &out, float Ts)
{
    if(!task_done)
    {
        state_timer += Ts;
        if(state_timer < accel_time) // accelerating phase
        {
            set_pos = init_pos + init_speed * state_timer + 0.5f * ref_accel * (state_timer * state_timer);
            set_speed = init_speed + ref_accel * state_timer;
            set_accel = ref_accel;
        }
        else if(state_timer < accel_time + cruise_time) // cruising phase, prevent cruise_time == 0
        {
            set_pos = start_cruise_pos + ref_speed * (state_timer - accel_time);
            set_speed = ref_speed;
            set_accel = 0.0f;
        }
        else if(state_timer < total_time) // decelerating phase
        {
            float Td = state_timer - total_time;
            set_pos = final_pos + 0.5f * ref_decel * (Td * Td);
            set_speed = ref_decel * Td;
            set_accel = ref_decel;
        }
        else
        {
            set_pos = final_pos;
            set_speed = 0.0f;
            set_accel = 0.0f;
            // reset state
            task_done = true;
            state_timer = 0.0f;
        }
        in.target_pos = set_pos;
    }
}

void TrajController::Reset()
{
    accel_time = 0.0f;
    decel_time = 0.0f;
    cruise_time = 0.0f;
    total_time = 0.0f;
    init_pos = 0.0f;
    init_speed = 0.0f;
    final_pos = 0.0f;
    start_cruise_pos = 0.0f;

    set_pos = 0.0f; 
    set_speed = 0.0f;
    set_accel = 0.0f;
    task_done = true;

    state_timer = 0.0f;
}

#endif