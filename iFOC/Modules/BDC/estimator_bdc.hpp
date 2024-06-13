#ifndef _FOC_ESTIMATOR_BDC_H
#define _FOC_ESTIMATOR_BDC_H

#include "encoder_base.h"
#include "limiter_base.hpp"
#include "trajectory_controller.hpp"

class EstimatorBDC
{
public:
    EstimatorBDC(foc_state_input_t& _input, foc_state_output_t& _output, foc_config_t& _config)
    : input(_input), output(_output), config(_config) {};
    bool Init();
    void AttachEncoder(EncoderBase *_encoder) { encoder = _encoder; };
    void AttachAuxEncoder(EncoderBase *_aux_encoder) { aux_encoder = _aux_encoder; };
    // void AttachLimiter(LimiterBase *_limiter) { limiter = _limiter; };
    void Update(float Ts);
    void UpdateMidInterval(float Ts);
    FOC_ERROR_FLAG GetErrorFlag();
    PID Idc_PID = PID(0.0f, 0.0f, 0.0f, 0.0f);
    PID Speed_PID = PID(0.0f, 0.0f, 0.0f, 0.0f);
    PID Position_PID = PID(0.0f, 0.0f, 0.0f, 0.0f);
    TrajController trajController;
private:
    EncoderBase *encoder = nullptr;
    EncoderBase *aux_encoder = nullptr;
    // LimiterBase *limiter = nullptr;
    foc_state_input_t& input;
    foc_state_output_t& output;
    foc_config_t& config;
    SlidingFilter Idc_sf = SlidingFilter(10);
    LowpassFilter Idc_lpf = LowpassFilter(0.001f);
    float state_timer = 0.0f;
    void Reset();
};

bool EstimatorBDC::Init()
{
    if(encoder == nullptr) return false;
    Idc_PID.Kp = config.current_kp;
    Idc_PID.Ki = config.current_ki;
    Idc_PID.ramp_limit = config.current_ramp_limit;
    Idc_PID.limit = config.Vphase_limit;
    Speed_PID.Kp = config.speed_kp;
    Speed_PID.Ki = config.speed_ki;
    Speed_PID.Kd = config.speed_kd;
    Speed_PID.ramp_limit = config.speed_ramp_limit;
    Speed_PID.limit = config.speed_current_limit;
    Position_PID.Kp = config.position_kp;
    Position_PID.Ki = config.position_ki;
    Position_PID.Kd = config.position_kd;
    Position_PID.ramp_limit = config.position_ramp_limit;
    Position_PID.limit = config.position_speed_limit;
    bool encoder1_init_state = encoder->Init(RPM_speed_to_rad(config.motor.max_mechanic_speed, config.motor.pole_pair));
    if(aux_encoder == nullptr) return encoder1_init_state;
    return encoder1_init_state & aux_encoder->Init(RPM_speed_to_rad(config.motor.max_mechanic_speed, config.motor.pole_pair));
}

// FOC_CMD_RET EstimatorBDC::SetMode(FOC_MODE _mode)
// {
//     mode = _mode;
//     state_timer = 0.0f;
//     return CMD_SUCCESS;
// }

// FOC_CMD_RET EstimatorBDC::SetSubMode(FOC_SUBMODE _smode)
// {
//     if(_smode == SUBMODE_HOME && limiter == nullptr) return CMD_NOT_SUPPORTED;
//     submode = _smode;
//     state_timer = 0.0f;
//     return CMD_SUCCESS;
// }

void EstimatorBDC::Update(float Ts)
{
    input.Ialphabeta_fb.alpha = Idc_lpf.GetOutput(input.Ialphabeta_fb.alpha, Ts);
    output.Iqd_fb = {input.Ialphabeta_fb.alpha, 0.0f};
    encoder->Update(Ts);
    output.estimated_angle = encoder->single_round_angle;
    output.estimated_raw_angle = encoder->raw_angle;
    output.estimated_speed = rad_speed_to_RPM(encoder->velocity, 1);
    // if(input.output_state)
    // {
        
    // }
}

void EstimatorBDC::UpdateMidInterval(float Ts)
{
    encoder->UpdateMidInterval(Ts);
    if(input.target > EST_TARGET_NONE)
    {
        if(input.target >= EST_TARGET_SPEED)
        {
            if(input.target >= EST_TARGET_POS) 
            {
                // if(mode == MODE_TRAJECTORY)
                // {
                //     trajController.Update(Ts);
                //     input.set_abs_pos = trajController.GetPos();
                // }
                // else
                // {
                //     trajController.Reset();
                // }
                output.set_speed = Position_PID.GetOutput(input.target_pos - output.estimated_raw_angle, Ts);
            }
            else output.set_speed = input.target_speed;
            output.Uqd.q = Speed_PID.GetOutput(output.set_speed - output.estimated_speed, Ts);
        }
    }
    else
    {
        state_timer = 0.0f;
        Reset();
    }
}

void EstimatorBDC::Reset()
{
    // input.target_speed = 0.0f;
    // input.set_abs_pos = 0.0f;
    // output.estimated_acceleration
    output.Uqd = {0.0f, 0.0f};
    Idc_PID.Reset();
    Speed_PID.Reset();
    Position_PID.Reset();
}

FOC_ERROR_FLAG EstimatorBDC::GetErrorFlag()
{
    uint16_t ret = FOC_ERROR_NONE;
    if(encoder == nullptr || encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost primary encoder
    if(aux_encoder != nullptr && aux_encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost auxiliary encoder
    return static_cast<FOC_ERROR_FLAG>(ret);
}

#endif