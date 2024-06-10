#ifndef _FOC_MODULE_PARAM_IDENT_HPP
#define _FOC_MODULE_PARAM_IDENT_HPP

#include "module_base.hpp"

class ParamIdentModule : public ModuleBase
{
public:
    typedef enum IdentStage
    {
        IDENTSTAGE_INIT,
        IDENTSTAGE_POLE_PAIR, // using TORQUE mode, and bypass estimator current loop
        
    }IdentStage;
    IdentStage stage = IDENTSTAGE_INIT;
    motor_param_t ident_result =
    {
        .Rs = 0.0f, // phase_resistance
        .Ld = 0.0f, // phase_inductance, should between 2e-6f and 4000e-6f
        .flux = 0.0f,
        .gear_ratio = 1.0f,
        .max_mechanic_speed = 0.0f,
        .pole_pair = 0,
    };
    bool stage_complete = true;
    void BeginStage(IdentStage _stage);
    void Preprocess(foc_state_input_t &in, foc_state_output_t &out, float Ts) override;
    void Postprocess(foc_state_input_t &in, foc_state_output_t &out, float Ts) override;
    typedef void (*ParamIdentCallback) (ParamIdentModule* ptr);
    ParamIdentCallback callback = nullptr;
private:
    uint8_t init = 0;
    float state_timer = 0.0f;
    // POLE_PAIR: [0]-search_angle, [1]-curr_elec_angle, [2]-encoder_begin_angle, [3]-encoder_end_angle
    float temp[4] = {0.0f};
};

void ParamIdentModule::BeginStage(IdentStage _stage)
{
    state_timer = 0.0f;
    stage = _stage;
    init = 1;
    stage_complete = false;
}

void ParamIdentModule::Preprocess(foc_state_input_t &in, foc_state_output_t &out, float Ts)
{
    switch(stage)
    {
        default: break;
    }
}

void ParamIdentModule::Postprocess(foc_state_input_t &in, foc_state_output_t &out, float Ts)
{
    switch(stage)
    {
        case IDENTSTAGE_POLE_PAIR:
            if(!stage_complete)
            {
                out.Uqd = {3.0f, 0.0f};
                state_timer += Ts;
                if(init)
                {
                    ident_result.pole_pair = 0;
                    temp[0] = 6.0f*PI;
                    temp[1] = 0.0f;
                    temp[2] = out.estimated_raw_angle;
                    temp[3] = 0.0f;
                    if(state_timer >= 0.3f) // keep steady for 0.3s
                    {
                        state_timer = 0.0f;
                        init = 0;
                    }
                }
                else
                {
                    if(temp[1] >= temp[0])
                    {
                        temp[3] = out.estimated_raw_angle;
                        if(state_timer >= 0.3f)
                        {
                            if(temp[2] == temp[3]) ident_result.pole_pair = 0;
                            else ident_result.pole_pair = (uint8_t)(roundf(temp[1] / (temp[3] - temp[2])));
                            stage_complete = true;
                            state_timer = 0.0f;
                            out.electric_angle = 0.0f;
                            if(callback != nullptr) callback(this);
                        }
                    }
                    else if(state_timer >= 0.001f)
                    {
                        temp[1] += 0.01f;
                        state_timer = 0.0f;
                    }
                }
                out.electric_angle = temp[1];
            }
            break;
        default: break;
    }
}

#endif