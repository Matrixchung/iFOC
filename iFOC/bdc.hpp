#ifndef _BDC_H
#define _BDC_H

#include "foc.hpp"
#include "driver_bdc_base.hpp"
#include "estimator_bdc.hpp"

#pragma GCC push_options
#pragma GCC optimize (3)

template<class T1, class T_CurrentSenseBase, class T_BusSenseBase>
class FOC<DriverBDCBase<T1>, T_CurrentSenseBase, T_BusSenseBase>
{
public:
    FOC(DriverBDCBase<T1>& _driver, T_CurrentSenseBase& _curr, T_BusSenseBase& _bus);
    bool Init(bool initTIM = true);
    void Update(float Ts);
    void UpdateMidInterval(float Ts);
    void UpdateIdleTask(float Ts);
    void SetOutputState(bool state);
    void EmergencyStop();
    bool GetTrajPosState();
    foc_config_t config;
    TrajController trajController;
private:
    template<class U> friend class BaseProtocol;
    template<class ... T> friend void SyncStartTimer(T&... inst);
    DriverBDCBase<T1>& driver;
    T_CurrentSenseBase& current_sense;
    T_BusSenseBase& bus_sense;
    foc_state_input_t est_input;
    foc_state_output_t* est_output;
    float overspeed_timer = 0.0f;
    float overcurrent_timer = 0.0f;
    uint16_t error_code = FOC_ERROR_NONE;
    FOC_MODE mode = MODE_TORQUE;
    bool output_state = false;
public:
    EstimatorBDC estimator = EstimatorBDC(est_input);
#ifdef FOC_USING_INDICATOR
public:
    std::unique_ptr<Indicator<GPIOBase>> indicator = nullptr;
    template <class T = GPIOBase> void SetIndicator(T& gpio);
#endif
#ifdef FOC_USING_TEMP_PROBE
public:
    void AttachMCUTempProbe(float *ptr) { mcu_temp = ptr; };
    void AttachMotorTempProbe(float *ptr) { motor_temp = ptr; };
    void AttachFETTempProbe(float *ptr) { mosfet_temp = ptr; };
private:
    float *mcu_temp = nullptr;    // in degree
    float *motor_temp = nullptr;  // in degree
    float *mosfet_temp = nullptr; // in degree
#endif
#ifdef FOC_USING_EXTRA_MODULE
public:
    template<class T> void AppendModule();
    template<class T> T* GetModule();
    void ResetModule() { extra_module.reset(); };
private:
    std::unique_ptr<ModuleBase> extra_module = nullptr;
#endif
};  

template<class A, class B, class C>
bool FOC<DriverBDCBase<A>, B, C>::Init(bool initTIM)
{
    if(driver.DriverInit(initTIM) == false || estimator.Init() == false)
    {
        error_code = FOC_ERROR_INITIALIZE;
        return false;
    }
    est_output = &estimator.output;
    current_sense.CurrentSenseUpdate();
    bus_sense.BusSenseUpdate();
    output_state = config.startup_state;
    mode = config.startup_mode;
    error_code = FOC_ERROR_NONE;
    return true;
}

template<class A, class B, class C>
void FOC<DriverBDCBase<A>, B, C>::Update(float Ts)
{
    current_sense.CurrentSenseUpdate();
    est_input.Ialphabeta_fb.alpha = current_sense.Iabc.a;
    if(output_state)
    {
        switch(mode)
        {
            case MODE_SPEED:
                est_input.target = EST_TARGET_SPEED;
                est_input.Iqd_target = {0.0f, 0.0f};
                break;
            case MODE_TRAJECTORY:
                trajController.Preprocess(&est_input, est_output, Ts);
                est_input.target = EST_TARGET_POS;
                break;
            case MODE_POSITION:
                est_input.target = EST_TARGET_POS;
                est_input.Iqd_target = {0.0f, 0.0f};
                break;
            default:
                est_input.target = EST_TARGET_TORQUE;
                break;
        }
    }
    else est_input.target = EST_TARGET_NONE;
    estimator.Update(Ts);
    error_code |= estimator.GetErrorFlag();
    if(output_state)
    {
        if(error_code == FOC_ERROR_NONE && bus_sense.Vbus > 0.0f)
        {
            driver.SetOutputPct(est_output->Uqd.q / bus_sense.Vbus);
        }
        else EmergencyStop();
    }
}

template<class A, class B, class C>
void FOC<DriverBDCBase<A>, B, C>::UpdateMidInterval(float Ts)
{
    estimator.UpdateMidInterval(Ts);
}

template<class A, class B, class C>
void FOC<DriverBDCBase<A>, B, C>::UpdateIdleTask(float Ts)
{
    bus_sense.BusSenseUpdate();
}

template<class A, class B, class C>
void FOC<DriverBDCBase<A>, B, C>::SetOutputState(bool state)
{
    output_state = state;
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

template<class A, class B, class C>
void FOC<DriverBDCBase<A>, B, C>::EmergencyStop()
{
    output_state = false;
    switch(config.break_mode)
    {
        default: 
            driver.SetOutputPct(0.0f);
            break;
    }
}

template<class T1, class T_CurrentSenseBase, class T_BusSenseBase>
FOC<DriverBDCBase<T1>, T_CurrentSenseBase, T_BusSenseBase>::FOC(DriverBDCBase<T1>& _driver, T_CurrentSenseBase& _curr, T_BusSenseBase& _bus)
: driver(_driver), current_sense(_curr), bus_sense(_bus)
{
    config = {
        .motor = {
            .Rs = 1.0f,
            .Ld = 0.001f,
            .flux = 0.0f,
            .gear_ratio = 1.0f,
            .max_mechanic_speed = 0.0f,
            .zero_elec_angle = 0.0f,
            .pole_pair = 1,
        },
        .mcu_temp_limit = 80.0f,
        .motor_temp_limit = 80.0f, 
        .align_current = 0.0f,
        .current_bandwidth = 0.0f,
        .current_damping_coefficient = 0.0f,
        .current_pid = {0.0f, 0.0f, 0.0f, 12.0f, 0.0f},
        .speed_pid = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .position_pid = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
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
        .startup_mode = MODE_TORQUE,
        .apply_curr_feedforward = false,
        .use_speed_pll = false,
        .speed_pll_config = 
        {
            .pid_config = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            .Tlp = 0.001f,
        },
    };
    est_input = {
        .Iqd_target = {.q = 0.0f, .d = 0.0f},
        .Ialphabeta_fb = {.alpha = 0.0f, .beta = 0.0f},
        .target_speed = 0.0f,
        .target_pos = 0.0f,
        .target = EST_TARGET_NONE,
    };
}

#include "bdc_base_protocol.hpp"

#pragma GCC pop_options

#endif