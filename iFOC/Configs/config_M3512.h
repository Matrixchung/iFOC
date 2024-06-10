#ifndef _FOC_MOTOR_CONFIG_M3512_H
#define _FOC_MOTOR_CONFIG_M3512_H

#include "foc_config_type.h"

static foc_config_t config_m3512 = 
{
    .motor = 
    {
        .Rs = 1.65f,
        .Ld = 0.0000102f,
        .flux = 0.0115585f,
        .gear_ratio = 1.0f,
        .max_mechanic_speed = 1800.0f,
        .pole_pair = 7,
    },
    .virtual_endstop = 
    {
        .current_limit = 0.0f,
        .stuck_time = 0.0f,
    },
    .mcu_temp_limit = 80.0f,
    .motor_temp_limit = 80.0f, 
    .align_current = 0.3f,
    .current_bandwidth = 0.0f,
    .current_damping_coefficient = 0.0f,
    .current_kp = 0.08f,
    .current_ki = 1500.0f,
    .Vphase_limit = 12.0f,
    .current_ramp_limit = 0.0f,
    .speed_kp = 0.01f,
    .speed_ki = 0.3f,
    .speed_kd = 0.0f,
    .speed_current_limit = 1.2f,
    .speed_ramp_limit = 0.0f,
    .position_kp = 50.0f,
    .position_ki = 0.0f,
    .position_kd = 0.0f,
    .position_speed_limit = 2000.0f,
    .position_ramp_limit = 20000.0f,
    .home_speed = 1000.0f,
    .max_speed = 15000.0f,
    .overspeed_detect_time = 0.001f,
    .traj_cruise_speed = 500.0f,
    .traj_max_accel = 100.0f,
    .traj_max_decel = 100.0f,
    .encoder_dir = FOC_DIR_NEG,
    .aux_encoder_dir = FOC_DIR_POS,
    .startup_state = false,
    .break_mode = BREAK_MODE_ASC,
    .startup_mode = MODE_POSITION,
};

#endif