#ifndef _FOC_MOTOR_CONFIG_M2804_H
#define _FOC_MOTOR_CONFIG_M2804_H

#include "foc_config_type.h"

static foc_config_t config_m2804 = 
{
    .motor = 
    {
        .Rs = 1.65f,
        .Ld = 0.0028f,
        .flux = 0.0f,
        .gear_ratio = 1.0f,
        .max_mechanic_speed = 1800.0f,
        .zero_elec_angle = 0.0f,
        .pole_pair = 7,
    },
    .mcu_temp_limit = 80.0f,
    .motor_temp_limit = 80.0f, 
    .align_current = 0.3f,
    .current_bandwidth = 0.0f,
    .current_damping_coefficient = 0.0f,
    .current_pid = {0.08f, 1500.0f, 0.0f, 12.0f, 0.0f},
    .speed_pid = {0.01f, 0.35f, 0.0f, 1.8f, 0.0f},
    .position_pid = {80.0f, 0.0f, 0.0f, 3000.0f, 0.0f},
    .home_speed = 1000.0f,
    .max_speed = 1000.0f,
    .overspeed_detect_time = 0.001f,
    .traj_cruise_speed = 700.0f,
    .traj_max_accel = 800.0f,
    .traj_max_decel = 500.0f,
    .encoder_dir = FOC_DIR_NEG,
    .aux_encoder_dir = FOC_DIR_POS,
    .startup_state = false,
    .break_mode = BREAK_MODE_ASC,
    .startup_mode = MODE_TORQUE,
    .apply_curr_feedforward = false,
    .use_speed_pll = false,
    .speed_pll_config = 
    {
        .pid_config = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .Tlp = 0.001f,
    }
};

#endif