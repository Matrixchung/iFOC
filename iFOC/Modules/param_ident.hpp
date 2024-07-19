#ifndef _FOC_MODULE_PARAM_IDENT_HPP
#define _FOC_MODULE_PARAM_IDENT_HPP

#include "module_base.hpp"

#define RESISTANCE_KI 50.0f
#define RESISTANCE_RAMP_TIME 2.0f // seconds
#define RESISTANCE_SAMPLE_TIME 1.0f

#define RESISTANCE_LAMBDA 1.0f
#define INDUCTANCE_LAMBDA 1.0f

class ParamIdentModule : public ModuleBase
{
public:
    typedef enum IdentStage
    {
        IDENTSTAGE_INIT,
        IDENTSTAGE_POLE_PAIR,    // using TORQUE mode, and bypass estimator current loop
        IDENTSTAGE_RESISTANCE,   // ResistanceMeasurementControlLaw，https://blog.csdn.net/linzhe_deep/article/details/118067983 (not accurate)
        IDENTSTAGE_R_L_F_ONLINE, // Using ordinary least squares, https://blog.csdn.net/qq_28149763/article/details/136848807 (accurate)
        IDENTSTAGE_INDUCTANCE,   // A Robust DPCC for IPMSM Based on a Full Parameter Identification Method, IEEE TIE
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
    void SetTestCurrent(float current);
    bool Preprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts) override;
    bool Postprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts) override;
    typedef void (*ParamIdentCallback) (ParamIdentModule* ptr);
    ParamIdentCallback callback = nullptr;
// private:
    uint8_t init = 0;
    float state_timer = 0.0f;
    // POLE_PAIR: [0]-search_angle, [1]-curr_elec_angle, [2]-encoder_begin_angle, [3]-encoder_end_angle
    // RESISTANCE: [0]-test_current(target_Ialpha), [1]-test_voltage(Ualpha), [2]-curr_current(curr_Ialpha), [3]-avg_stable_current, [4]-sample_cnt
    // R_ONLINE: [0 - 3]: P_K_1[x], [4 - 5]: X_K_1[x], [6]: L_P_K_1, [7]: L_X_K_1
    float temp[16] = {0.0f};
    LowpassFilter lp_filter1 = LowpassFilter(0.0001f);
    LowpassFilter lp_filter2 = LowpassFilter(0.0001f);
    LowpassFilter lp_filter3 = LowpassFilter(0.0001f);
};

void ParamIdentModule::BeginStage(IdentStage _stage)
{
    stage = _stage;
    for(uint8_t i = 1; i < 16; i++) temp[i] = 0.0f;
    if(stage <= IDENTSTAGE_POLE_PAIR)
    {
        temp[0] = 0.0f;
    }
    else
    {
        if(stage == IDENTSTAGE_R_L_F_ONLINE)
        {
            temp[0] = 10000.0f;
            temp[3] = 10000.0f;
            temp[6] = 1e+6f;
        }
    }
    state_timer = 0.0f;
    init = 1;
    stage_complete = false;
}

void ParamIdentModule::SetTestCurrent(float current)
{
    temp[0] = current;
}

bool ParamIdentModule::Preprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts)
{
    switch(stage)
    {
        case IDENTSTAGE_R_L_F_ONLINE:
            in->target = EST_TARGET_SPEED;
            return false;
        default: break;
    }
    return true;
}

bool ParamIdentModule::Postprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts)
{
    switch(stage)
    {
        case IDENTSTAGE_POLE_PAIR:
            {
                if(!stage_complete)
                {
                    out->Uqd = {3.0f, 0.0f};
                    state_timer += Ts;
                    if(init)
                    {
                        ident_result.pole_pair = 0;
                        temp[0] = 6.0f * PI;
                        temp[1] = 0.0f;
                        temp[2] = out->estimated_raw_angle;
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
                            temp[3] = out->estimated_raw_angle;
                            if(state_timer >= 0.3f)
                            {
                                if(temp[2] == temp[3]) ident_result.pole_pair = 0;
                                else ident_result.pole_pair = (uint8_t)(roundf(temp[1] / (temp[3] - temp[2])));
                                stage_complete = true;
                                state_timer = 0.0f;
                                out->electric_angle = 0.0f;
                                if(callback != nullptr) callback(this);
                            }
                        }
                        else if(state_timer >= 0.001f)
                        {
                            temp[1] += 0.01f;
                            state_timer = 0.0f;
                        }
                    }
                    out->electric_angle = temp[1];
                }
            }
            break;
        case IDENTSTAGE_RESISTANCE:
            // Using pre-defined Ki to track the output voltage based on given test current
            // and transform output voltage to Alpha axis voltage
            // we can only control Uqd through Module interface
            // Reverse Park Transform:
            // output.alpha = _cos * input.d - _sin * input.q;
            // output.beta = _cos * input.q + _sin * input.d;
            // we set electric angle to 270 degree (3/2 * PI), then make Uq = target_Ualpha, Ud = 0
            // Ualpha = Uq = target_Ualpha, Ubeta = Ud = 0
            // filtered Ibeta can detect unbalanced phase or short phase
            if(!stage_complete)
            {
                if(temp[0] == 0.0f)
                {
                    stage_complete = true;
                    break; // invalid start arguments
                }
                state_timer += Ts;
                out->electric_angle = 3.0f * M_PI_2;
                if(state_timer < 1.0f) // reset Uqd output, keep it stable at 0V
                {
                    out->Uqd.q = 0.0f;
                    out->Uqd.d = 0.0f;
                }
                else if(state_timer < RESISTANCE_RAMP_TIME + 1.0f)
                {
                    float step = Ts * temp[0] / RESISTANCE_RAMP_TIME;
                    temp[2] += step;
                    temp[1] += (RESISTANCE_KI * Ts) * (temp[2] - in->Ialphabeta_fb.alpha);
                    out->Uqd.q = temp[1];
                    out->Uqd.d = 0.0f;
                }
                else if(state_timer < RESISTANCE_RAMP_TIME + RESISTANCE_SAMPLE_TIME + 1.0f)
                {
                    out->Uqd.q = temp[1];
                    out->Uqd.d = 0.0f; // keep current Uqd settings
                    temp[4] += 1.0f;
                    temp[3] += in->Ialphabeta_fb.alpha;
                }
                else
                {
                    // take average
                    temp[3] /= temp[4];
                    // Here we got line resistance
                    ident_result.Rs = temp[1] / temp[3];
                    state_timer = 0.0f;
                    stage_complete = true;
                    out->Uqd.q = 0.0f;
                    out->Uqd.d = 0.0f;
                    if(callback != nullptr) callback(this);
                }
                return false;
            }
            break;
        case IDENTSTAGE_R_L_F_ONLINE:
            if(!stage_complete)
            {
                float Uq_filtered = lp_filter1.GetOutput(out->Uqd.q, Ts);
                float Ud_filtered = lp_filter2.GetOutput(out->Uqd.d, Ts);
                float Iq_filtered = lp_filter3.GetOutput(out->Iqd_fb.q, Ts);
                // R, flux
                float omega = RPM_speed_to_rad(out->estimated_speed, 1);
                float f = omega * temp[0];
                float f1 = Iq_filtered * temp[3];
                float phi = (f + Iq_filtered * temp[1]) * omega + (omega * temp[2] + f1) * Iq_filtered;
                float K_idx_0 = (f + temp[2] * Iq_filtered) / (phi + RESISTANCE_LAMBDA);
                float K_idx_1 = (temp[1] * omega + f1) / (phi + RESISTANCE_LAMBDA);
                if(isnanf(K_idx_0) || isnanf(K_idx_1))
                {
                    temp[0] = 10000.0f;
                    temp[3] = 10000.0f;
                    temp[4] = 0.0f;
                    temp[5] = 0.0f;
                    return true;
                }
                float fv[4];
                fv[0] = 1.0f - K_idx_0 * omega;
                fv[1] = -K_idx_1 * omega;
                fv[2] = -K_idx_0 * Iq_filtered;
                fv[3] = 1.0f - K_idx_1 * Iq_filtered;
                f = temp[0];
                f1 = temp[1];
                phi = temp[2];
                float f2 = temp[3];
                float fv1[4];
                for(uint8_t i = 0; i < 2; i++)
                {
                    fv1[i] = (fv[i] * f + fv[i + 2] * f1) / RESISTANCE_LAMBDA;
                    fv1[i + 2] = (fv[i] * phi + fv[i + 2] * f2) / RESISTANCE_LAMBDA;
                }
                temp[0] = fv1[0];
                temp[1] = fv1[1];
                temp[2] = fv1[2];
                temp[3] = fv1[3];
                phi = Uq_filtered - (omega * temp[4] + Iq_filtered * temp[5]);
                temp[4] += K_idx_0 * phi;
                temp[5] += K_idx_1 * phi;
                ident_result.Rs = temp[5];
                ident_result.flux = temp[4];
                // Ld
                phi = -omega * Iq_filtered;
                float K = temp[6] * phi;
                K /= K * phi + INDUCTANCE_LAMBDA;
                temp[6] = (1.0f - K * phi) * temp[6] / INDUCTANCE_LAMBDA;
                temp[7] += K * (Ud_filtered - phi * temp[7]);
                ident_result.Ld = temp[7];
            }
            break;
        case IDENTSTAGE_INDUCTANCE:
            break;
        default: break;
    }
    return true;
}

#endif