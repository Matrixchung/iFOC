#ifndef _FOC_TYPE_H
#define _FOC_TYPE_H

#include "stdint.h"
#include "stddef.h"
/*
    @brief FOC Function Return Codes
*/

#define FOC_DIR_POS (1)
#define FOC_DIR_NEG (-1)

typedef enum FOC_CMD_RET
{
    CMD_SUCCESS         = 0,
    CMD_FORBIDDEN       = 1,
    CMD_NOT_SUPPORTED   = 2,
    CMD_UNKNOWN_FAILURE = 3,
}FOC_CMD_RET;

typedef enum FOC_ERROR_FLAG
{
    FOC_ERROR_NONE           = 0x00,
    FOC_ERROR_INITIALIZE     = 0x01,
    FOC_ERROR_ALIGN          = 0x01 << 1,
    FOC_ERROR_OVER_VOLTAGE   = 0x01 << 2,
    FOC_ERROR_UNDER_VOLTAGE  = 0x01 << 3,
    FOC_ERROR_MOTOR_OVERTEMP = 0x01 << 4,
    FOC_ERROR_STARTUP        = 0x01 << 5,
    FOC_ERROR_FEEDBACK       = 0x01 << 6,
    FOC_ERROR_OVERSPEED      = 0x01 << 7,
    FOC_ERROR_OVER_CURRENT   = 0x01 << 8,
    FOC_ERROR_MCU_OVERTEMP   = 0x01 << 9,
}FOC_ERROR_FLAG;

typedef enum FOC_MODE
{
    // MODE_INIT = 0,
    MODE_TORQUE = 0,
    MODE_SPEED,
    MODE_POSITION,
    MODE_TRAJECTORY,
    MODE_CUSTOM,
    LAST_MODE_PLACEHOLDER,
}FOC_MODE;

typedef enum FOC_EST_TARGET
{
    EST_TARGET_NONE = 0,
    EST_TARGET_TORQUE = 1,
    EST_TARGET_SPEED = 2,
    EST_TARGET_POS = 3,
}FOC_EST_TARGET;

typedef struct qd_t
{
    float q;
    float d;
}qd_t;

typedef struct ab_t
{
    float a;
    float b;
}ab_t;

typedef struct abc_t
{
    float a;
    float b;
    float c;
}abc_t;

typedef struct alphabeta_t
{
    float alpha;
    float beta;
}alphabeta_t;

typedef struct pid_config_t
{
    float Kp, Ki, Kd, limit, ramp_limit;
}pid_config_t;

typedef struct svpwm_t
{
    alphabeta_t Ualphabeta;
    uint16_t max_compare;
    uint8_t sector;
    uint16_t compare_a;
    uint16_t compare_b;
    uint16_t compare_c;
}svpwm_t;

typedef struct foc_state_input_t
{
    qd_t Iqd_target;
    alphabeta_t Ialphabeta_fb;
    float target_speed;
    float target_pos;
    FOC_EST_TARGET target;
}foc_state_input_t;

typedef struct foc_state_output_t
{
    qd_t Iqd_set;
    qd_t Iqd_fb;
    qd_t Uqd;
    float electric_angle;
    float estimated_angle;
    float estimated_raw_angle;
    float set_speed;
    float estimated_speed;
}foc_state_output_t;

/**
  * @brief  Trigonometrical functions type definition
  */
typedef struct Trig_Components
{
  int16_t cos_i16;
  int16_t sin_i16;
}Trig_Components;

#endif