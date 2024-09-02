#ifndef _FOC_ESTIMATOR_BDC_H
#define _FOC_ESTIMATOR_BDC_H

#include "estimator_base.hpp"
#include "encoder_base.hpp"
#include "speed_pll.hpp"

class EstimatorBDC : public EstimatorBase
{
public:
    EstimatorBDC(foc_state_input_t& _in, foc_config_t& _config) : EstimatorBase(_in, _config), speed_pll(config.speed_pll_config){};
    bool Init() final;
    void AttachEncoder(EncoderBase *_encoder) { encoder = _encoder; };
    void Update(float Ts) final;
    void UpdateMidInterval(float Ts) final;
    FOC_ERROR_FLAG GetErrorFlag() final;
private:
    SpeedPLL speed_pll;
    EncoderBase *encoder = nullptr;
    // SlidingFilter Idc_sf = SlidingFilter(10);
    // LowpassFilter Idc_lpf = LowpassFilter(0.001f);
    float state_timer = 0.0f;
#ifdef FOC_SENSORED_USING_AUX_ENCODER
public:
    void AttachAuxEncoder(EncoderBase *_aux_encoder) { aux_encoder = _aux_encoder; };
private:
    EncoderBase *aux_encoder = nullptr;
#endif
};

bool EstimatorBDC::Init()
{
    if(encoder == nullptr) return false;
    bool encoder1_init_state = encoder->Init();
#ifdef FOC_SENSORED_USING_AUX_ENCODER
    if(aux_encoder == nullptr) return encoder1_init_state;
    return encoder1_init_state & aux_encoder->Init();
#else
    return encoder1_init_state;
#endif
}

void EstimatorBDC::Update(float Ts)
{
    // input.Ialphabeta_fb.alpha = Idc_lpf.GetOutput(input.Ialphabeta_fb.alpha, Ts);
    output.Iqd_fb = {input.Ialphabeta_fb.alpha, 0.0f};
    encoder->Update(Ts);
    output.estimated_angle = encoder->single_round_angle;
    output.estimated_raw_angle = encoder->raw_angle;
    if(config.use_speed_pll) output.estimated_speed = rad_speed_to_RPM(speed_pll.Calculate(output.estimated_angle, Ts), 1);
    if(input.target > EST_TARGET_NONE)
    {
        
    }
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
        EstimatorBase::Reset();
    }
}

FOC_ERROR_FLAG EstimatorBDC::GetErrorFlag()
{
    uint16_t ret = FOC_ERROR_NONE;
    if(encoder == nullptr || encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost primary encoder
#ifdef FOC_SENSORED_USING_AUX_ENCODER
    if(aux_encoder != nullptr && aux_encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost auxiliary encoder
#endif
    return static_cast<FOC_ERROR_FLAG>(ret);
}

#endif