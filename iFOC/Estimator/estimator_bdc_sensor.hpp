#ifndef _FOC_ESTIMATOR_BDC_SENSOR_H
#define _FOC_ESTIMATOR_BDC_SENSOR_H

#include "estimator_base.hpp"
#include "encoder_base.h"
#include "limiter_base.hpp"

class EstimatorBDCSensor : public EstimatorBase
{
public:
    bool Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config) override;
    void AttachEncoder(EncoderBase *_encoder);
    void AttachAuxEncoder(EncoderBase *_aux_encoder);
    void AttachLimiter(LimiterBase *_limiter);
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    FOC_CMD_RET SetMode(FOC_MODE _mode) override;
    FOC_CMD_RET SetSubMode(FOC_SUBMODE _smode) override;
    FOC_ERROR_FLAG GetErrorFlag() override;
private:
    EncoderBase *encoder = nullptr;
    EncoderBase *aux_encoder = nullptr;
    LimiterBase *limiter = nullptr;
    float state_timer = 0.0f;
};

bool EstimatorBDCSensor::Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config)
{
    type = ESTIMATOR_SENSOR;
    if(encoder == nullptr) return false;
    EstimatorBase::_Init(_in, _out, _config);
    bool encoder1_init_state = encoder->Init(RPM_speed_to_rad(config->motor.max_mechanic_speed, config->motor.pole_pair));
    if(aux_encoder == nullptr) return encoder1_init_state;
    return encoder1_init_state & aux_encoder->Init(RPM_speed_to_rad(config->motor.max_mechanic_speed, config->motor.pole_pair));
}

FOC_CMD_RET EstimatorBDCSensor::SetMode(FOC_MODE _mode)
{
    mode = _mode;
    state_timer = 0.0f;
    return CMD_SUCCESS;
}

FOC_CMD_RET EstimatorBDCSensor::SetSubMode(FOC_SUBMODE _smode)
{
    if(_smode == SUBMODE_HOME && limiter == nullptr) return CMD_NOT_SUPPORTED;
    submode = _smode;
    state_timer = 0.0f;
    return CMD_SUCCESS;
}

void EstimatorBDCSensor::Update(float Ts)
{
    encoder->Update(Ts);
    output->estimated_angle = encoder->single_round_angle;
    output->estimated_raw_angle = encoder->raw_angle / config->motor.gear_ratio;
    output->estimated_speed = rad_speed_to_RPM(encoder->velocity, 1);
    if(input->output_state)
    {
        if(mode == MODE_SPEED || mode == MODE_POSITION)
        {
            if(mode == MODE_POSITION) output->out_speed = Position_PID.GetOutput(input->set_abs_pos - output->estimated_raw_angle, Ts);
            else output->out_speed = input->set_speed;
        }
    }
}

void EstimatorBDCSensor::UpdateMidInterval(float Ts)
{
    encoder->UpdateMidInterval(Ts);
    if(input->output_state)
    {

    }
    else
    {
        state_timer = 0.0f;
        EstimatorBase::Reset();
    }
}

void EstimatorBDCSensor::AttachEncoder(EncoderBase *_encoder)
{
    encoder = _encoder;
}

void EstimatorBDCSensor::AttachAuxEncoder(EncoderBase *_aux_encoder)
{
    aux_encoder = _aux_encoder;
}

void EstimatorBDCSensor::AttachLimiter(LimiterBase *_limiter)
{
    limiter = _limiter;
}

FOC_ERROR_FLAG EstimatorBDCSensor::GetErrorFlag()
{
    // if(encoder == nullptr || encoder->IsError()) return FOC_ERROR_FEEDBACK; // lost primary encoder
    // if(aux_encoder != nullptr && aux_encoder->IsError()) return FOC_ERROR_FEEDBACK; // lost auxiliary encoder
    // if(align_error_flag) return FOC_ERROR_STARTUP;
    // return FOC_ERROR_NONE;
    uint16_t ret = FOC_ERROR_NONE;
    if(encoder == nullptr || encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost primary encoder
    if(aux_encoder != nullptr && aux_encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost auxiliary encoder
    return static_cast<FOC_ERROR_FLAG>(ret);
}

#endif