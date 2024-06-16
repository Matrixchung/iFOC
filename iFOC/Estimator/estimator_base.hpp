#ifndef _FOC_ESTIMATOR_BASE_H
#define _FOC_ESTIMATOR_BASE_H

#include "foc_header.h"
#include "pid.h"

// An estimator needs: state input
// ... and outputs: estimated angle, estimated speed, etc.
// Estimator needs to calculate Iqd_fb based on estimated elec angle and Uqd.
// For motor with gear ratio, the estimator only needs to calculate angle(rad) and speed(RPM) at the drive output(called origin).
// All inputs to the estimator also represent origin speed.

class EstimatorBase
{
public:
    EstimatorBase(){};
    FOC_ESTIMATOR type = ESTIMATOR_NONE;
    virtual bool Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config) = 0;
    virtual void Update(float Ts) = 0;
    virtual void UpdateMidInterval(float Ts) {};
    virtual void UpdateIdleTask(float Ts) {};
    virtual FOC_ERROR_FLAG GetErrorFlag() {return FOC_ERROR_NONE;};
// protected:
    PID Iq_PID = PID(0.0f, 0.0f, 0.0f, 0.0f);
    PID Id_PID = PID(0.0f, 0.0f, 0.0f, 0.0f);
    PID Speed_PID = PID(0.0f, 0.0f, 0.0f, 0.0f);
    PID Position_PID = PID(0.0f, 0.0f, 0.0f, 0.0f);
    // TrajController trajController;
protected:
    foc_state_input_t *input;
    foc_state_output_t *output;
    foc_config_t *config;
    void _Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config);
    void Reset();
};

void EstimatorBase::_Init(foc_state_input_t *_in, foc_state_output_t *_out, foc_config_t *_config)
{
    input = _in;
    output = _out;
    config = _config;
    if(config->motor.gear_ratio == 0.0f) config->motor.gear_ratio = 1.0f;
    if(config->current_kp > 0.0f && config->current_ki > 0.0f) // if set current Kp/Ki in config, then use it
    {
        Iq_PID.Kp = config->current_kp;
        Iq_PID.Ki = config->current_ki;
        Id_PID.Kp = config->current_kp;
        Id_PID.Ki = config->current_ki;
    }
    else if(config->current_damping_coefficient > 0.0f) // we use current_bandwidth and motor params to calculate
    {
        // https://blog.csdn.net/weixin_45182459/article/details/136971188
        // Kp = current_loop_bandwidth * Lq / (6 * xi^2) (xi = 0.707, for damping), Ki = Rs * bandwidth / (6 * xi^2)
        float damping_coeff = 1.0f / (6.0f * config->current_damping_coefficient * config->current_damping_coefficient);
        float Kp = config->current_bandwidth * config->motor.Ld * damping_coeff;
        float Ki = config->current_bandwidth * config->motor.Rs * damping_coeff;
        Iq_PID.Kp = Kp;
        Iq_PID.Ki = Ki;
        Id_PID.Kp = Kp;
        Id_PID.Ki = Ki;
    }
    
    Iq_PID.Kd = 0.0f;
    // Iq_PID.limit = config->Vphase_limit;
    // Iq_PID.limit = config->Vphase_limit * 0.57735026918962576450914878050195f; // 1/sqrt(3), SPWM
    Iq_PID.limit = config->Vphase_limit * 0.6666666667f; // Vbus * 2/3, SVPWM
    Iq_PID.ramp_limit = config->current_ramp_limit;

    Id_PID.Kd = 0.0f;
    // Id_PID.limit = config->Vphase_limit;
    // Id_PID.limit = config->Vphase_limit * 0.57735026918962576450914878050195f;
    Id_PID.limit = config->Vphase_limit * 0.6666666667f; 
    Id_PID.ramp_limit = config->current_ramp_limit;

    Speed_PID.Kp = config->speed_kp;
    Speed_PID.Ki = config->speed_ki;
    Speed_PID.Kd = config->speed_kd;
    Speed_PID.limit = config->speed_current_limit;
    Speed_PID.ramp_limit = config->speed_ramp_limit;

    Position_PID.Kp = config->position_kp;
    Position_PID.Ki = config->position_ki;
    Position_PID.Kd = config->position_kd;
    Position_PID.limit = config->position_speed_limit;
    Position_PID.ramp_limit = config->position_ramp_limit;
}

void EstimatorBase::Reset()
{
    // output->electric_angle = 0.0f;
    // output->estimated_speed = 0.0f;
    output->Iqd_set.q = 0.0f;
    output->Iqd_set.d = 0.0f;
    output->Uqd.q = 0.0f;
    output->Uqd.d = 0.0f;
    Iq_PID.Reset();
    Id_PID.Reset();
    Speed_PID.Reset();
    Position_PID.Reset();
}

#endif