#ifndef _BDC_TYPE_H
#define _BDC_TYPE_H

#include "foc_type.h"

typedef struct bdc_state_input_t
{
    float Idc;
    float target_speed;
    float target_pos;
    FOC_EST_TARGET target;
}bdc_state_input_t;

typedef struct bdc_state_output_t
{
    float Udc;
    float estimated_angle;
    float estimated_raw_angle;
    float set_speed;
    float estimated_speed;
}bdc_state_output_t;

#endif