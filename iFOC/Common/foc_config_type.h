#ifndef _FOC_CONFIG_TYPE_H
#define _FOC_CONFIG_TYPE_H

#include "stdint.h"
#include "stddef.h"
#include "foc_type.h"

#define BREAK_MODE_NORMAL 0
#define BREAK_MODE_SPO 1
#define BREAK_MODE_ASC 2

typedef struct motor_param_t
{
    float Rs; // phase_resistance
    float Ld; // phase_inductance, should between 2e-6f and 4000e-6f
    float flux;
    float gear_ratio;
    float max_mechanic_speed; // design-limited max mechanic speed
    float zero_elec_angle;    // zero electric angle(rad), could be more than one value
    uint8_t pole_pair;
}motor_param_t;

// typedef struct virtual_endstop_t
// {
//     float current_limit;
//     float stuck_time;
// }virtual_endstop_t;

typedef struct foc_config_t
{
    motor_param_t motor;
    // virtual_endstop_t virtual_endstop;
    float mcu_temp_limit;
    float motor_temp_limit;
    float align_current;
    float current_bandwidth;           // if current_kp and current_ki is not set,
    float current_damping_coefficient; // those will be used to calculate current loop Kp/Ki
    float current_kp;
    float current_ki;
    float Vphase_limit;
    float current_ramp_limit;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float speed_current_limit;
    float speed_ramp_limit;
    float position_kp;
    float position_ki;
    float position_kd;
    float position_speed_limit;
    float position_ramp_limit;
    float home_speed;
    float max_speed;  // max allowed speed, overspeed for *overspeed_detection_ms* cause OVERSPEED error.
    float overspeed_detect_time; // if speed above limit exceed this duration(ms), we will throw error. 
    float traj_cruise_speed;
    float traj_max_accel;
    float traj_max_decel;
    int8_t encoder_dir;
    int8_t aux_encoder_dir;
    bool startup_state;
    uint8_t break_mode;
    FOC_MODE startup_mode;
    bool apply_curr_feedforward; // requires accurate Ld measurement
}foc_config_t;

#endif