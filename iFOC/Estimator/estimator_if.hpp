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
    FOC_CMD_RET SetMode(FOC_MODE _mode) override;
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
    mode = MODE_SPEED;
    if(start_time <= 0.0f) start_time = 1.0f;
    return true;
}

void EstimatorIF::Update(float Ts)
{
    output->Iqd_fb = FOC_Park(input->Ialphabeta_fb, output->electric_angle);
    if(input->output_state)
    {
        if(mode == MODE_SPEED)
        {
            state_timer += Ts;
            if(state_timer < if_position_time)
            {
                state = POSITION;
                output->electric_angle = 0.0f;
                output->estimated_angle = 0.0f;
                output->estimated_raw_angle = 0.0f;
                output->estimated_speed = 0.0f;
            }
            else
            {
                if(state_timer < (if_position_time + start_time) && (ABS(output->estimated_speed) < ABS(input->set_speed)))
                {
                    state = DRAG;
                    output->estimated_acceleration = input->set_speed / start_time;
                    output->estimated_speed += output->estimated_acceleration * Ts;
                }
                else
                {
                    state = CONSTANT;
                    output->estimated_acceleration = 0.0f;
                    output->estimated_speed = input->set_speed;
                }
                output->electric_angle += RPM_speed_to_rad(output->estimated_speed, config->motor.pole_pair) * Ts;
                output->electric_angle = normalize_rad(output->electric_angle);
                output->estimated_raw_angle += RPM_speed_to_rad(output->estimated_speed, 1) * Ts;
                output->estimated_angle = normalize_rad(output->estimated_raw_angle);
            }
            output->Iqd_out = input->Iqd_set;
            output->out_speed = output->estimated_speed;
        }
        output->Uqd.q = Iq_PID.GetOutput(output->Iqd_out.q - output->Iqd_fb.q, Ts);
        output->Uqd.d = Id_PID.GetOutput(output->Iqd_out.d - output->Iqd_fb.d, Ts);
    }
    else
    {
        state = POSITION;
        state_timer = 0.0f;
        EstimatorBase::Reset();
    }
}

FOC_CMD_RET EstimatorIF::SetMode(FOC_MODE _mode)
{
    if(_mode == MODE_INIT || _mode == MODE_SPEED)
    {
        mode = _mode;
        return CMD_SUCCESS;
    }
    return CMD_NOT_SUPPORTED;
}

#endif