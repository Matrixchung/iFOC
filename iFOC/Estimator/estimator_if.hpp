#ifndef _FOC_ESTIMATOR_IF_H
#define _FOC_ESTIMATOR_IF_H

#include "estimator_base.hpp"

#define FOC_IF_POSITION_PERIOD 150

class EstimatorIF : public EstimatorBase
{
public:
    // FOC_ESTIMATOR type = ESTIMATOR_IF;
    bool Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config) override;
    void Update(float Ts) override;
    float start_time = 1.0f;
private:
    float state_timer = 0.0f;
    float if_position_time = FOC_IF_POSITION_PERIOD * 1e-3f;
    typedef enum EstimatorIFState
    {
        POSITION,
        DRAG,
        CONSTANT,
    }EstimatorIFState;
    EstimatorIFState state = POSITION;
};

bool EstimatorIF::Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config)
{
    type = ESTIMATOR_IF;
    EstimatorBase::_Init(_in, _out, _config);
    if(start_time <= 0.0f) start_time = 1.0f;
    return true;
}

void EstimatorIF::Update(float Ts)
{
    output->Iqd_fb = FOC_Park(input->Ialphabeta_fb, output->electric_angle);
    if(input->target > EST_TARGET_NONE)
    {
        if(input->target == EST_TARGET_SPEED)
        {
            state_timer += Ts;
            if(state_timer < if_position_time && input->target_speed != 0.0f)
            {
                state = POSITION;
                output->electric_angle = 0.0f;
                output->estimated_angle = 0.0f;
                output->estimated_raw_angle = 0.0f;
                output->estimated_speed = 0.0f;
            }
            else
            {
                if(state_timer < (if_position_time + start_time) && (ABS(output->estimated_speed) < ABS(input->target_speed)))
                {
                    state = DRAG;
                    output->estimated_speed += input->target_speed / start_time * Ts;
                }
                else
                {
                    state = CONSTANT;
                    output->estimated_speed = input->target_speed;
                }
                output->electric_angle += RPM_speed_to_rad(output->estimated_speed, config->motor.pole_pair) * Ts;
                output->electric_angle = normalize_rad(output->electric_angle);
                output->estimated_raw_angle += RPM_speed_to_rad(output->estimated_speed, 1) * Ts;
                output->estimated_angle = normalize_rad(output->estimated_raw_angle);
            }
            output->Iqd_set = input->Iqd_target;
        }
        output->Uqd.q = Iq_PID.GetOutput(output->Iqd_set.q - output->Iqd_fb.q, Ts);
        output->Uqd.d = Id_PID.GetOutput(output->Iqd_set.d - output->Iqd_fb.d, Ts);
    }
    else
    {
        state = POSITION;
        state_timer = 0.0f;
        EstimatorBase::Reset();
    }
}

#endif