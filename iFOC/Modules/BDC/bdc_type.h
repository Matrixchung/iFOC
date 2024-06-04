#ifndef _BDC_TYPE_H
#define _BDC_TYPE_H

#include "foc_type.h"

typedef struct bdc_state_input_t
{
    float Idc;
    float set_speed;
    float set_abs_pos;
    bool output_state;
}bdc_state_input_t;

typedef struct bdc_state_output_t
{
    float Udc;
    float estimated_angle;
    float estimated_raw_angle;
    float out_speed;
    float estimated_speed;
    float estimated_acceleration;
}bdc_state_output_t;

#endif