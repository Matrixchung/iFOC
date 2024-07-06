#ifndef _FOC_ESTIMATOR_BASE_H
#define _FOC_ESTIMATOR_BASE_H

#include "foc_header.h"
#include "direct_param_pid.hpp"

// An estimator needs: state input
// ... and outputs: estimated angle, estimated speed, etc.
// Estimator needs to calculate Iqd_fb based on estimated elec angle and Uqd.
// For motor with gear ratio, the estimator only needs to calculate angle(rad) and speed(RPM) at the drive output(called origin).
// All inputs to the estimator also represent origin speed.

class EstimatorBase
{
public:
    EstimatorBase(foc_state_input_t& _in, foc_config_t& _config)
    : input(_in), config(_config), Iq_PID(_config.current_pid), Id_PID(_config.current_pid), Speed_PID(_config.speed_pid), Position_PID(_config.position_pid) {};
    virtual bool Init() = 0;
    virtual void Update(float Ts) = 0;
    virtual void UpdateMidInterval(float Ts) {};
    virtual void UpdateIdleTask(float Ts) {};
    virtual FOC_ERROR_FLAG GetErrorFlag() {return FOC_ERROR_NONE;};
    foc_state_output_t output;
protected:
    foc_state_input_t& input;
    foc_config_t& config;
    DP_PID Iq_PID;
    DP_PID Id_PID;
    DP_PID Speed_PID;
    DP_PID Position_PID;
    void Reset();
};

void EstimatorBase::Reset()
{
    output.Iqd_set.q = 0.0f;
    output.Iqd_set.d = 0.0f;
    output.Uqd.q = 0.0f;
    output.Uqd.d = 0.0f;
    Iq_PID.Reset();
    Id_PID.Reset();
    Speed_PID.Reset();
    Position_PID.Reset();
}

#endif