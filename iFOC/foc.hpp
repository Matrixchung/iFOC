#ifndef _FOC_H
#define _FOC_H

// Brushless DC Motor(BLDC)

#include "foc_header.h"
#include "driver_base.hpp"

// Estimator Base Classes
#include "estimator_base.hpp"
#include "estimator_if.hpp"
#include "estimator_sensor.hpp"

// Sense Base Classes
#include "current_sense_base.hpp"
#include "bus_sense_base.hpp"
#include "bus_sense_mirror.hpp"
#include "bus_sense_static.hpp"

// iFOC Modules
#include "sound_injector.hpp"

// Unit agreements: all angle are radius, while speed is RPM at output shaft of motor
// Using CRTP for zero-overhead polymorphism at *compile-time*
// See: https://stackoverflow.com/questions/18174441/crtp-and-multilevel-inheritance/18174442#18174442

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
class FOC
{
public:
    FOC(T_DriverBase& _driver, T_CurrentSenseBase& _curr, T_BusSenseBase& _bus);
    bool Init();
    void AttachEstimator(EstimatorBase *_estimator);
    void AttachMCUTempProbe(float *ptr);
    void AttachMotorTempProbe(float *ptr);
    void Update(float Ts);
    void UpdateMidInterval(float Ts);
    void SetOutputState(bool state);
    void EmergencyStop();
    foc_config_t config;
    SoundInjector soundInjector;
private:
    template<typename U>
    friend class BaseProtocol;
    T_DriverBase& driver;
    T_CurrentSenseBase& current_sense;
    T_BusSenseBase& bus_sense;
    EstimatorBase *estimator = nullptr; // we need to change the estimator type dynamically
    foc_state_input_t est_input;
    foc_state_output_t est_output;
    svpwm_t svpwm;
    uint16_t error_code = FOC_ERROR_NONE;
    float *mcu_temp = nullptr;   // in degree
    float *motor_temp = nullptr; // in degree
};

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
bool FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::Init()
{
    if( driver.DriverInit() == false ||
       (estimator == nullptr || !estimator->Init(&est_input, &est_output, &config)) ||
       (estimator->SetMode(config.startup_mode)))
    {
        error_code = FOC_ERROR_INITIALIZE;
        return false;
    }
    svpwm.max_compare = driver.GetMaxCompare();
    current_sense.CurrentSenseUpdate();
    bus_sense.BusSenseUpdate();
    est_input.output_state = config.startup_state;
    error_code = FOC_ERROR_NONE;
    return true;
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::AttachMCUTempProbe(float *ptr)
{
    mcu_temp = ptr;
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::AttachMotorTempProbe(float *ptr)
{
    motor_temp = ptr;
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::Update(float Ts)
{
    current_sense.CurrentSenseUpdate();
    est_input.Ialphabeta_fb = FOC_Clark_ABC(current_sense.Iabc);
    estimator->Update(Ts);

    soundInjector.Update(est_output.Uqd, Ts);
    svpwm.Ualphabeta = FOC_Rev_Park(est_output.Uqd, est_output.electric_angle);
    if(est_input.output_state) 
    {
        error_code |= estimator->GetErrorFlag();
        if(error_code == FOC_ERROR_NONE)
        {
            FOC_SVPWM(&svpwm, bus_sense.Vbus);
            driver.SetOutput(svpwm.compare_a, svpwm.compare_b, svpwm.compare_c);
        }
        else
        {
            // we should emergency break
            EmergencyStop();
        }
    }
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::SetOutputState(bool state)
{
    est_input.output_state = state;
    if(state)
    {
        error_code = 0;
        driver.EnableOutput();
    }
    else
    {
        EmergencyStop();
    }
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::EmergencyStop()
{
    est_input.output_state = false;
    switch(config.break_mode)
    {
        case BREAK_MODE_SPO: // SPO
            driver.SetLSIdleState(0);
            driver.DisableOutput();
            break;
        case BREAK_MODE_ASC: // ASC
            driver.SetLSIdleState(1);
            driver.DisableOutput();
            break;
        default: 
            driver.SetOutputPct(0.0f, 0.0f, 0.0f);
            break;
    }
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::UpdateMidInterval(float Ts)
{
    bus_sense.BusSenseUpdate();
    estimator->UpdateMidInterval(Ts);
    if(mcu_temp != nullptr)
    {
        if(*mcu_temp > config.mcu_temp_limit) error_code |= FOC_ERROR_MCU_OVERTEMP;
    }
    if(motor_temp != nullptr)
    {
        if(*motor_temp > config.motor_temp_limit) error_code |= FOC_ERROR_MOTOR_OVERTEMP;
    }
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::AttachEstimator(EstimatorBase *_estimator)
{
    estimator = _estimator;
}

// init structs
template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
FOC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::FOC(T_DriverBase& _driver, T_CurrentSenseBase& _curr, T_BusSenseBase& _bus) 
: driver(_driver), current_sense(_curr), bus_sense(_bus)
{
    config = {
        .motor = {
            .Rs = 1.0f,
            .Ld = 0.001f,  // default 1mH
            .flux = 0.0f,
            .gear_ratio = 1.0f,
            .max_mechanic_speed = 0.0f,
            .pole_pair = 1,
        },
        .virtual_endstop = {
            .current_limit = 0.3f,
            .stuck_time = 0.5f,
        },
        .mcu_temp_limit = 80.0f,  // limit MCU max temp to 80 degree
        .motor_temp_limit = 80.0f, 
        .align_current = 0.6f,
        .current_bandwidth = 10000.0f,
        .current_damping_coefficient = 0.707f,
        .current_kp = 0.0f,
        .current_ki = 0.0f,
        .Vphase_limit = 12.0f,
        .current_ramp_limit = 0.0f,
        .speed_kp = 0.0f,
        .speed_ki = 0.0f,
        .speed_kd = 0.0f,
        .speed_current_limit = 0.0f,
        .speed_ramp_limit = 0.0f,
        .position_kp = 0.0f,
        .position_ki = 0.0f,
        .position_kd = 0.0f,
        .position_speed_limit = 0.0f,
        .position_ramp_limit = 0.0f,
        .home_speed = 0.0f,
        .max_speed = 0.0f,
        .overspeed_detect_time = 0.001f,
        .encoder_dir = FOC_DIR_POS,
        .aux_encoder_dir = FOC_DIR_POS,
        .startup_state = false,
        .break_mode = BREAK_MODE_ASC,
        .startup_mode = MODE_INIT,
    };

    est_output = {
        .Iqd_out = {.q = 0.0f, .d = 0.0f},
        .Iqd_fb = {.q = 0.0f, .d = 0.0f,},
        .Uqd = {.q = 0.0f, .d = 0.0f,},
        .electric_angle = 0.0f,
        .estimated_angle = 0.0f,
        .estimated_raw_angle = 0.0f,
        .out_speed = 0.0f,
        .estimated_speed = 0.0f,
        .estimated_acceleration = 0.0f,
    };
    est_input = {
        .Iqd_set = {.q = 0.0f, .d = 0.0f},
        .Ialphabeta_fb = {.alpha = 0.0f, .beta = 0.0f},
        .set_speed = 0.0f,
        .set_abs_pos = 0.0f,
        .output_state = false,
    };
    svpwm = {
        .Ualphabeta = {.alpha = 0.0f, .beta = 0.0f,},
        .max_compare = 0,
        .sector = 0,
        .compare_a = 0,
        .compare_b = 0,
        .compare_c = 0,
    };
}

#include "base_protocol.hpp"

#endif