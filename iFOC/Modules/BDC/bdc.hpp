#ifndef _BDC_H
#define _BDC_H

// Brushed DC Motor(BDC)

#include "foc_header.h"
#include "bdc_type.h"
#include "foc_config_type.h"
#include "driver_bdc_base.hpp"

// BDC Estimator Base Classes
#include "estimator_bdc.hpp"

// Sense Base Classes
#include "current_sense_base.hpp"
#include "bus_sense_base.hpp"
#include "bus_sense_mirror.hpp"
#include "bus_sense_static.hpp"

// Unit agreements: all angle are radius, while speed is RPM at output shaft of motor

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
class BDC
{
public:
    BDC(T_DriverBase& _driver, T_CurrentSenseBase& _curr, T_BusSenseBase& _bus);
    bool Init();
    void Update(float Ts);
    void UpdateMidInterval(float Ts);
    void SetOutputState(bool state);
    void EmergencyStop();
    foc_config_t config;
    EstimatorBDC estimator = EstimatorBDC(est_input, est_output, config);
private:
    template<typename U>
    friend class BaseProtocol;
    T_DriverBase& driver;
    T_CurrentSenseBase& current_sense;
    T_BusSenseBase& bus_sense;
    bdc_state_input_t est_input;
    bdc_state_output_t est_output;
    uint16_t error_code = FOC_ERROR_NONE;
};

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
bool BDC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::Init()
{
    if( driver.DriverInit() == false ||
       (estimator.Init() == false) ||
       (estimator.SetMode(config.startup_mode)))
    {
        error_code = FOC_ERROR_INITIALIZE;
        return false;
    }
    current_sense.CurrentSenseUpdate();
    bus_sense.BusSenseUpdate();
    est_input.output_state = config.startup_state;
    error_code = FOC_ERROR_NONE;
    return true;
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void BDC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::Update(float Ts)
{
    current_sense.CurrentSenseUpdate();
    est_input.Idc = current_sense.Iabc.a;
    estimator.Update(Ts);
    if(est_input.output_state) 
    {
        error_code = estimator.GetErrorFlag();
        if(error_code == FOC_ERROR_NONE && bus_sense.Vbus > 0.0f)
        {
            driver.SetOutputPct(est_output.Udc / bus_sense.Vbus);
        }
        else
        {
            // we should emergency break
            EmergencyStop();
        }
    }
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void BDC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::SetOutputState(bool state)
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
void BDC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::EmergencyStop()
{
    est_input.output_state = false;
    switch(config.break_mode)
    {
        default: 
            driver.SetOutputPct(0.0f);
            break;
    }
}

template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
void BDC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::UpdateMidInterval(float Ts)
{
    bus_sense.BusSenseUpdate();
    estimator.UpdateMidInterval(Ts);
}

// init structs
template<typename T_DriverBase, typename T_CurrentSenseBase, typename T_BusSenseBase>
BDC<T_DriverBase, T_CurrentSenseBase, T_BusSenseBase>::BDC(T_DriverBase& _driver, T_CurrentSenseBase& _curr, T_BusSenseBase& _bus) 
: driver(_driver), current_sense(_curr), bus_sense(_bus)
{
    config = {
        .motor = {
            .Rs = 1.0f,
            .Ld = 0.000f,
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
        .current_kp = 1.0f, // if we dont use current loop, just pass Kp = 1
        .current_ki = 0.0f,
        .Vphase_limit = 12.0f,
        .current_ramp_limit = 1000.0f,
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
        .traj_cruise_speed = 0.0f,
        .traj_max_accel = 0.0f,
        .traj_max_decel = 0.0f,
        .encoder_dir = FOC_DIR_POS,
        .aux_encoder_dir = FOC_DIR_POS,
        .startup_state = false,
        .break_mode = BREAK_MODE_ASC,
        .startup_mode = MODE_INIT,
    };

    est_output = {
        .Udc = 0.0f,
        .estimated_angle = 0.0f,
        .estimated_raw_angle = 0.0f,
        .out_speed = 0.0f,
        .estimated_speed = 0.0f,
        .estimated_shaft_speed = 0.0f,
        .estimated_acceleration = 0.0f,
    };
    est_input = {
        .Idc = 0.0f,
        .set_speed = 0.0f,
        .set_abs_pos = 0.0f,
        .output_state = false,
    };
}

#include "bdc_base_protocol.hpp"

#endif