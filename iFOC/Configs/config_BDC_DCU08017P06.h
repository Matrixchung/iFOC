#ifndef _FOC_MOTOR_CONFIG_DCU08017P06_H
#define _FOC_MOTOR_CONFIG_DCU08017P06_H

#include "foc_config_type.h"

static foc_config_t config_dcu08017p06 = 
{
    .motor = 
    {
        .Rs = 0.0f,
        .Ld = 0.0f,
        .flux = 0.0f,
        .gear_ratio = 256.0f,
        .max_mechanic_speed = 60.0f,
        .zero_elec_angle = 0.0f,
        .pole_pair = 1,
    },
    .mcu_temp_limit = 80.0f,
    .motor_temp_limit = 80.0f, 
    .align_current = 0.0f,
    .current_bandwidth = 0.0f,
    .current_damping_coefficient = 0.0f,
    .current_pid = {0.1f, 7000.0f, 0.0f, 8.0f, 0.0f},
    .speed_pid = {0.0000035f, 0.00002f, 0.00000003f, 0.1f, 0.0f}, // Kd = 0.00000003
    .position_pid = {120.0f, 0.0f, 0.0f, 15360.0f, 0.0f},
    .home_speed = 50.0f,
    .max_speed = 100.0f, // overspeed protection
    .overspeed_detect_time = 0.01f,
    .traj_cruise_speed = 60.0f,
    .traj_max_accel = 3000.0f,
    .traj_max_decel = 2000.0f,
    .encoder_dir = FOC_DIR_NEG,
    .aux_encoder_dir = FOC_DIR_POS,
    .startup_state = true,
    .break_mode = BREAK_MODE_ASC,
    .startup_mode = MODE_TRAJECTORY,
    .apply_curr_feedforward = false,
    .use_speed_pll = false,
    .speed_pll_config = 
    {
        .pid_config = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .Tlp = 0.0f,
    },
};

#endif