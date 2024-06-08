#ifndef _FOC_ESTIMATOR_SENSOR_H
#define _FOC_ESTIMATOR_SENSOR_H

#include "estimator_base.hpp"
#include "encoder_base.h"
#include "limiter_base.hpp"

#define FOC_SENSOR_ALIGN_PERIOD 500
#define FOC_SENSOR_ALIGN_STEADY_PERIOD 250 // in this period(ms) we check for overspeed.

class EstimatorSensor : public EstimatorBase
{
public:
    bool Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config) override;
    void AttachEncoder(EncoderBase *_encoder);
    void AttachAuxEncoder(EncoderBase *_aux_encoder);
    // void AttachLimiter(LimiterBase *_limiter);
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    FOC_ERROR_FLAG GetErrorFlag() override;
private:
    EncoderBase *encoder = nullptr;
    EncoderBase *aux_encoder = nullptr;
    qd_t Iqd_align = {.q = 0.0f, .d = 0.0f,};
    float state_timer = 0.0f;
    float zero_electric_angle = 0.0f;
    const float align_time = FOC_SENSOR_ALIGN_PERIOD * 1e-3f;
    const float steady_time = FOC_SENSOR_ALIGN_STEADY_PERIOD * 1e-3f;
    uint8_t error_flag = 0;
    bool align_state = false;
};

bool EstimatorSensor::Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config)
{
    type = ESTIMATOR_SENSOR;
    if(encoder == nullptr) return false;
    EstimatorBase::_Init(_in, _out, _config);
    Iqd_align.q = config->align_current;
    Iqd_align.d = 0.0f;
    bool encoder1_init_state = encoder->Init(RPM_speed_to_rad(shaft_to_origin(config->motor.max_mechanic_speed, config->motor.gear_ratio), config->motor.pole_pair));
    if(aux_encoder == nullptr) return encoder1_init_state;
    return encoder1_init_state & aux_encoder->Init(RPM_speed_to_rad(shaft_to_origin(config->motor.max_mechanic_speed, config->motor.gear_ratio), config->motor.pole_pair));
}

void EstimatorSensor::Update(float Ts)
{
    encoder->Update(Ts);
    output->estimated_angle = encoder->single_round_angle;
    output->estimated_raw_angle = encoder->raw_angle;
    output->estimated_speed = rad_speed_to_RPM(encoder->velocity, 1);
    if(align_state && config->motor.pole_pair > 0) output->electric_angle = normalize_rad(output->estimated_angle * config->motor.pole_pair - zero_electric_angle);
    output->Iqd_fb = FOC_Park(input->Ialphabeta_fb, output->electric_angle);
    if(input->target > EST_TARGET_NONE)
    {
        if(align_state)
        {
            if(input->target >= EST_TARGET_SPEED)
            {
                if(input->target == EST_TARGET_POS) output->set_speed = Position_PID.GetOutput(input->target_pos - output->estimated_raw_angle, Ts);
                else output->set_speed = input->target_speed;
                output->Iqd_set.q = Speed_PID.GetOutput(input->target_speed - output->estimated_speed, Ts);
                output->Iqd_set.d = 0.0f;
            }
            else output->Iqd_set = input->Iqd_target;
        }
        output->Uqd.q = Iq_PID.GetOutput(output->Iqd_set.q - output->Iqd_fb.q, Ts);
        output->Uqd.d = Id_PID.GetOutput(output->Iqd_set.d - output->Iqd_fb.d, Ts);
    }
}

void EstimatorSensor::UpdateMidInterval(float Ts)
{
    encoder->UpdateMidInterval(Ts);
    // if(!encoder->UseSpeedPLL())
    // {
    //     output->estimated_speed = rad_speed_to_RPM(encoder->UpdateVelocity(Ts), 1);
    //     if(mode == MODE_SPEED || mode == MODE_POSITION)
    //     {
    //         if(mode == MODE_POSITION) output->out_speed = Position_PID.GetOutput(input->set_abs_pos - output->estimated_raw_angle, Ts);
    //         else output->out_speed = input->set_speed;
    //         output->Iqd_out.q = Speed_PID.GetOutput(output->out_speed - output->estimated_speed, Ts);
    //         output->Iqd_out.d = 0.0f;
    //     }
    // }
    // if(limiter != nullptr)
    // {
    //     limiter->Update();
    // }
    if(input->target > EST_TARGET_NONE)
    {
        // Pre-alignment process
        if(!align_state)
        {
            state_timer += Ts;
            if(state_timer < align_time)
            {
                output->electric_angle = _3PI_2;
                output->Iqd_set = Iqd_align;
            }
            else if(state_timer < align_time + steady_time)
            {
                // if(ABS(output->estimated_speed) >= 10.0f) // align failure check logic
                // {
                //     error_flag = 1;
                //     output->Iqd_out.q = 0.0f;
                //     output->Iqd_out.d = 0.0f;
                // }
            }
            else if(!error_flag)
            {
                if(encoder->IsCalibrated())
                {
                    if(output->electric_angle != _3PI_2)
                    {
                        state_timer = 0.0f;
                    }
                    else
                    {
                        input->target_pos = output->estimated_raw_angle;
                        zero_electric_angle = normalize_rad(encoder->single_round_angle * config->motor.pole_pair);
                        align_state = true;
                        output->Iqd_set = {0.0f, 0.0f};
                        state_timer = 0.0f;
                    }
                }
                else
                {
                    output->electric_angle += 0.001f;
                    output->electric_angle = normalize_rad(output->electric_angle);
                }
            }
        }
        
        // Limiter class
        // if(limiter != nullptr)
        // {
        //     if(submode == SUBMODE_HOME)
        //     {
        //         Speed_PID.limit = 0.4f;
        //         mode = MODE_SPEED;
        //         input->target_speed = config->home_speed * limiter->direction;
        //         if(ABS(output->Iqd_fb.q) >= 0.3f)
        //         {
        //             state_timer += Ts;
        //             if(state_timer >= 0.5f)
        //             {
        //                 limiter->direction = -1 * limiter->direction;
        //                 state_timer = 0.0f;
        //             }
        //         }
        //         else state_timer = 0.0f;
        //         if(limiter->IsActivated())
        //         {
        //             Speed_PID.limit = config->speed_current_limit;
        //             mode = MODE_POSITION;
        //             input->target_speed = 0.0f;
        //             // input->set_abs_pos = 0.0f;
        //             input->set_abs_pos = output->estimated_raw_angle;
        //             submode = SUBMODE_NONE;
        //         }
        //     }
        // }
    }
    else 
    {
        align_state = false;
        state_timer = 0.0f;
        error_flag = 0;
        EstimatorBase::Reset();
    }
}

void EstimatorSensor::AttachEncoder(EncoderBase *_encoder)
{
    encoder = _encoder;
}

void EstimatorSensor::AttachAuxEncoder(EncoderBase *_aux_encoder)
{
    aux_encoder = _aux_encoder;
}

// void EstimatorSensor::AttachLimiter(LimiterBase *_limiter)
// {
//     limiter = _limiter;
// }

FOC_ERROR_FLAG EstimatorSensor::GetErrorFlag()
{
    // if(encoder == nullptr || encoder->IsError()) return FOC_ERROR_FEEDBACK; // lost primary encoder
    // if(aux_encoder != nullptr && aux_encoder->IsError()) return FOC_ERROR_FEEDBACK; // lost auxiliary encoder
    // if(align_error_flag) return FOC_ERROR_STARTUP;
    // return FOC_ERROR_NONE;
    uint16_t ret = FOC_ERROR_NONE;
    if(encoder == nullptr || encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost primary encoder
    if(aux_encoder != nullptr && aux_encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost auxiliary encoder
    if(error_flag == 1) ret |= FOC_ERROR_ALIGN;
    return static_cast<FOC_ERROR_FLAG>(ret);
}

#endif