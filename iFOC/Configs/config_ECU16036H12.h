#ifndef _FOC_MOTOR_CONFIG_ECU16036H12_H
#define _FOC_MOTOR_CONFIG_ECU16036H12_H

#include "foc_config_type.h"

static foc_config_t config_ecu16036h12 = 
{
    .motor = 
    {
        .Rs = 1.65f,
        .Ld = 0.0000102f,
        .flux = 0.0115585f,
        .gear_ratio = 21.0f,
        .max_mechanic_speed = 650.0f,
        .zero_elec_angle = 0.0f,
        .pole_pair = 1,
    },
    .mcu_temp_limit = 80.0f,
    .motor_temp_limit = 80.0f, 
    .max_current = 2.0f,
    .align_current = 0.8f,
    .current_bandwidth = 0.0f,
    .current_damping_coefficient = 0.0f,
    // .current_pid = {0.051f, 825.0f, 0.0f, 12.0f / SQRT_3, 0.0f},
    .current_pid = {0.068f, 1000.0f, 0.0f, 12.0f / SQRT_3, 0.0f},
    .speed_pid = {0.0002f, 0.02f, 0.0f, 2.0f, 0.0f},
    .position_pid = {600.0f, 0.0f, 0.0f, 15000.0f, 0.0f},
    .home_speed = 100.0f,
    .max_speed = 750.0f, // overspeed protection
    .overspeed_detect_time = 0.01f,
    .traj_cruise_speed = 400.0f,
    .traj_max_accel = 2000.0f,
    .traj_max_decel = 2000.0f,
    .encoder_dir = FOC_DIR_NEG,
    .aux_encoder_dir = FOC_DIR_POS,
    .startup_state = false,
    .break_mode = BREAK_MODE_ASC,
    .startup_mode = MODE_TRAJECTORY,
    .debug_mode = FOC_DEBUG_NONE,
    .apply_curr_feedforward = false,
    .use_speed_pll = true,
    // .speed_pll_config = 
    // {
    //     .pid_config = {500.0f, 150000.0f, 0.0f, 0.0f, 0.0f},
    //     .Tlp = 0.001f,
    // },
    .speed_pll_config = 
    {
        .pid_config = {1495.0f, 150000.0f, 0.0f, 0.0f, 0.0f},
        .Tlp = 0.001f,
    },
};

#endif