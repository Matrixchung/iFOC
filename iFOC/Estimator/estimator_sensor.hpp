#ifndef _FOC_ESTIMATOR_SENSOR_H
#define _FOC_ESTIMATOR_SENSOR_H

#include "estimator_base.hpp"
#include "encoder_base.hpp"
#include "speed_pll.hpp"

#define FOC_SENSOR_ALIGN_PERIOD 850
#define FOC_SENSOR_ALIGN_STEADY_PERIOD 150 // in this period(ms) we check for overspeed.

constexpr float align_time = FOC_SENSOR_ALIGN_PERIOD * 1e-3f;
constexpr float steady_time = FOC_SENSOR_ALIGN_STEADY_PERIOD * 1e-3f;

class EstimatorSensor : public EstimatorBase
{
public:
    EstimatorSensor(foc_state_input_t& _in, foc_config_t& _config) : EstimatorBase(_in, _config), speed_pll(config.speed_pll_config){};
    bool Init() final;
    void AttachEncoder(EncoderBase *_encoder) { encoder = _encoder; };
    void Update(float Ts) final;
    void UpdateMidInterval(float Ts) final;
    FOC_ERROR_FLAG GetErrorFlag() final;
private:
    SpeedPLL speed_pll;
    EncoderBase *encoder = nullptr;
    qd_t Iqd_align = {.q = 0.0f, .d = 0.0f,};
    float state_timer = 0.0f;
    float zero_electric_angle = 0.0f;
    uint8_t error_flag = 0;
    bool align_state = false;
#ifdef FOC_SENSORED_USING_AUX_ENCODER
public:
    void AttachAuxEncoder(EncoderBase *_aux_encoder) { aux_encoder = _aux_encoder; };
private:
    EncoderBase *aux_encoder = nullptr;
#endif
};

bool EstimatorSensor::Init()
{
    if(encoder == nullptr) return false;
    Iqd_align.q = 0.0f;
    Iqd_align.d = config.align_current;
    if(config.motor.zero_elec_angle > 0.0f)
    {
        align_state = true;
        zero_electric_angle = config.motor.zero_elec_angle;
    }
    bool encoder1_init_state = encoder->Init();
#ifdef FOC_SENSORED_USING_AUX_ENCODER
    if(aux_encoder == nullptr) return encoder1_init_state;
    return encoder1_init_state & aux_encoder->Init();
#else
    return encoder1_init_state;
#endif
}

void EstimatorSensor::Update(float Ts)
{
    encoder->Update(Ts);
    output.estimated_angle = encoder->single_round_angle;
    output.estimated_raw_angle = encoder->raw_angle - abs_raw_angle_offset;
    if(config.use_speed_pll) output.estimated_speed = rad_speed_to_RPM(speed_pll.Calculate(output.estimated_angle, Ts), 1);
    if(align_state) output.electric_angle = normalize_rad(output.estimated_angle * config.motor.pole_pair - zero_electric_angle);
    else output.estimated_speed = 0.0f; // pll calculated, but not used
    output.Iqd_fb = FOC_Park(input.Ialphabeta_fb, output.electric_angle);
    if(input.target > EST_TARGET_NONE)
    {
        if(align_state)
        {
            output.Iqd_set = input.Iqd_target; // Iqd_target acts as current bias
            if(input.target >= EST_TARGET_SPEED)
            {
                if(input.target == EST_TARGET_POS) output.set_speed = Position_PID.GetOutput(input.target_pos - output.estimated_raw_angle, Ts);
                else output.set_speed = input.target_speed;
                output.Iqd_set.q += Speed_PID.GetOutput(output.set_speed - output.estimated_speed, Ts);
                // output.Iqd_set.d += 0.0f;
            }
        }
        output.Uqd.q = Iq_PID.GetOutput(output.Iqd_set.q - output.Iqd_fb.q, Ts);
        output.Uqd.d = Id_PID.GetOutput(output.Iqd_set.d - output.Iqd_fb.d, Ts);
    }
    else abs_raw_angle_offset = encoder->raw_angle;
}

void EstimatorSensor::UpdateMidInterval(float Ts)
{
    if(!config.use_speed_pll)
    {
        encoder->UpdateMidInterval(Ts);
        if(align_state) output.estimated_speed = rad_speed_to_RPM(encoder->velocity, 1);
    }
    if(input.target > EST_TARGET_NONE && config.debug_mode != FOC_DEBUG_GATE_DRIVER)
    {
        if(align_state && zero_electric_angle != config.motor.zero_elec_angle) // need recalibration
        {
            align_state = false;
            state_timer = 0.0f;
            zero_electric_angle = 0.0f;
        }
        // Pre-alignment process
        if(!align_state && config.motor.pole_pair > 0)
        {
            state_timer += Ts;
            if(state_timer < align_time)
            {
                output.electric_angle = 0.0f;
                float Iq_step = Iqd_align.q * Ts / align_time; // SOFT START
                float Id_step = Iqd_align.d * Ts / align_time;
                output.Iqd_set.q += Iq_step;
                output.Iqd_set.d += Id_step;
                output.Iqd_set.q = _constrain(output.Iqd_set.q, 0.0f, Iqd_align.q);
                output.Iqd_set.d = _constrain(output.Iqd_set.d, 0.0f, Iqd_align.d);
            }
            else if(state_timer < align_time + steady_time)
            {
                output.Iqd_set = Iqd_align;
                if(euclid_distance(output.Iqd_fb.q, output.Iqd_fb.d, output.Iqd_set.q, output.Iqd_set.d) >= 
                   euclid_distance(0.0f, 0.0f, output.Iqd_set.q, output.Iqd_set.d) * 0.75f) // current response not works as expected, or current feedback route unavailable
                {
                    error_flag = 1;
                }
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
                    // encoder->Update(Ts);
                    if(zero_electric_angle == 0.0f)
                    {
                        zero_electric_angle = normalize_rad(encoder->single_round_angle * config.motor.pole_pair);
                        config.motor.zero_elec_angle = zero_electric_angle;
                        output.Iqd_set = {0.0f, 0.0f};
                    }
                    if(state_timer > align_time + steady_time * 2.0f)
                    {
                        abs_raw_angle_offset = encoder->raw_angle;
                        align_state = true;
                        state_timer = 0.0f;
                    }
                }
                else
                {
                    output.electric_angle -= 0.01f;
                    output.electric_angle = normalize_rad(output.electric_angle);
                    if(output.electric_angle < 0.01f) error_flag = 1; // search for over one round, still no zero signal
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
        state_timer = 0.0f;
        error_flag = 0;
        EstimatorBase::Reset();
    }
}

// void EstimatorSensor::AttachLimiter(LimiterBase *_limiter)
// {
//     limiter = _limiter;
// }

FOC_ERROR_FLAG EstimatorSensor::GetErrorFlag()
{
    uint16_t ret = FOC_ERROR_NONE;
    if(encoder == nullptr || encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost primary encoder
#ifdef FOC_SENSORED_USING_AUX_ENCODER
    if(aux_encoder != nullptr && aux_encoder->IsError()) ret |= FOC_ERROR_FEEDBACK; // lost auxiliary encoder
#endif
    if(error_flag == 1) ret |= FOC_ERROR_ALIGN;
    return static_cast<FOC_ERROR_FLAG>(ret);
}

#endif