#ifndef _FOC_H
#define _FOC_H

#pragma GCC push_options
#pragma GCC optimize (3)

#include "foc_conf.h"

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
#include "module_base.hpp"
#include "sound_injector.hpp"
#include "trajectory_controller.hpp"
#include "param_ident.hpp"

// Unit agreements: all angle are radius, while speed is RPM at output shaft of motor
// Using CRTP for zero-overhead polymorphism at *compile-time*
// See: https://stackoverflow.com/questions/18174441/crtp-and-multilevel-inheritance/18174442#18174442

template<class T_DriverBase, class T_CurrentSenseBase, class T_BusSenseBase>
class FOC
{
public:
    FOC(T_DriverBase& _driver, T_CurrentSenseBase& _curr, T_BusSenseBase& _bus);
    bool Init(bool initTIM = true);
    template<class T> void AttachMainEstimator();
    template<class T> T* GetMainEstimator();
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
    T_DriverBase& driver;
    T_CurrentSenseBase& current_sense;
    T_BusSenseBase& bus_sense;
    foc_state_input_t est_input;
    foc_state_output_t *est_output;
    std::unique_ptr<EstimatorBase> main_estimator = nullptr;
    svpwm_t svpwm;
    float overspeed_timer = 0.0f;
    float overcurrent_timer = 0.0f;
    uint16_t error_code = FOC_ERROR_NONE;
    FOC_MODE mode = MODE_TORQUE;
    bool output_state = false;
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
#ifdef FOC_USING_AUX_ESTIMATOR
public:
    template<class T> void AttachAuxEstimator();
    template<class T> T* GetAuxEstimator();
    void SwitchEstimator(uint8_t index);
    bool IsMainEstimatorActive() { return is_main_est; };
private:
    std::unique_ptr<EstimatorBase> aux_estimator = nullptr;
    bool is_main_est = true;
#else
public:
    bool IsMainEstimatorActive() { return true; };
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
bool FOC<A, B, C>::Init(bool initTIM) // if initTIM set to false, then we will synchronously start all timers later.
{
    if(config.motor.gear_ratio <= 0.0f) config.motor.gear_ratio = 1.0f; // fix gear ratio
    if((config.current_pid.Kp == 0.0f && config.current_pid.Ki == 0.0f) && 
    config.current_bandwidth > 0.0f && config.current_damping_coefficient > 0.0f) // if curr bandwidth and damping coeff set, we use motor params to calculate
    {
        // https://blog.csdn.net/weixin_45182459/article/details/136971188
        // Kp = current_loop_bandwidth * Lq / (6 * xi^2) (xi = 0.707, for damping), Ki = Rs * bandwidth / (6 * xi^2)
        float damping_coeff = 1.0f / (6.0f * config.current_damping_coefficient * config.current_damping_coefficient);
        config.current_pid.Kp = config.current_bandwidth * config.motor.Ld * damping_coeff;
        config.current_pid.Ki = config.current_bandwidth * config.motor.Rs * damping_coeff;
    }
    config.current_pid.Kd = 0.0f;
    config.speed_pll_config.pid_config.limit = RPM_speed_to_rad(shaft_to_origin(config.motor.max_mechanic_speed * 1.2f, config.motor.gear_ratio), config.motor.pole_pair);
    if(driver.DriverInit(initTIM) == false
       || (main_estimator == nullptr || !main_estimator->Init())
#ifdef FOC_USING_AUX_ESTIMATOR
       || (aux_estimator != nullptr && !aux_estimator->Init())
#endif
    )
    {
        error_code = FOC_ERROR_INITIALIZE;
        return false;
    }
    est_output = &main_estimator.get()->output;
    svpwm.max_compare = driver.GetMaxCompare();
    current_sense.CurrentSenseUpdate();
    bus_sense.BusSenseUpdate();
    output_state = config.startup_state;
    mode = config.startup_mode;
    error_code = FOC_ERROR_NONE;
    // soundInjector.inject_voltage = config.current_pid.limit / 3.0f;
    return true;
}

template<class A, class B, class C>
void FOC<A, B, C>::Update(float Ts)
{
    current_sense.CurrentSenseUpdate();
    est_input.Ialphabeta_fb = FOC_Clark_ABC(current_sense.Iabc);
    if(output_state)
    {
#ifdef FOC_USING_EXTRA_MODULE
        if(extra_module.get() == nullptr || (extra_module.get() != nullptr && extra_module->Preprocess(&est_input, est_output, Ts)))
#endif 
        {
            switch(mode)
            {
                case MODE_SPEED:
                    est_input.target = EST_TARGET_SPEED;
                    est_input.Iqd_target = {0.0f, 0.0f}; // Iqd_target now acts as bias
                    break;
                case MODE_TRAJECTORY:
                    trajController.Preprocess(&est_input, est_output, Ts);
                    est_input.target = EST_TARGET_POS;
                    est_input.Iqd_target = {0.0f, 0.0f};
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
    }
#ifdef FOC_USING_EXTRA_MODULE
    else est_input.target = EST_TARGET_NONE;
#endif
    main_estimator->Update(Ts);
    error_code |= main_estimator->GetErrorFlag();
    if(output_state)
    {
        if(ABS(est_output->Iqd_fb.q) >= config.speed_pid.limit * 0.95 || ABS(est_output->Iqd_fb.d) >= config.speed_pid.limit * 0.95)
        {
            overcurrent_timer += Ts;
            if(overcurrent_timer > 1.0f)
            {
                error_code = FOC_ERROR_OVER_CURRENT;
                config.motor.zero_elec_angle = 0.0f; // need recalibration
                overcurrent_timer = 0.0f;
                EmergencyStop();
                return;
            }
        }
#ifdef FOC_USING_EXTRA_MODULE
        if(extra_module.get() == nullptr || (extra_module.get() != nullptr && extra_module->Postprocess(&est_input, est_output, Ts)))
#endif        
        {
            // soundInjector.Postprocess(&est_input, est_output, Ts);
            if(config.apply_curr_feedforward) // apply current loop feedforward
            {
                float omega = RPM_speed_to_rad(est_output->estimated_speed, 1);
                est_output->Uqd.q += omega * config.motor.Ld * est_output->Iqd_fb.d;
                est_output->Uqd.d -= omega * config.motor.Ld * est_output->Iqd_fb.q;
            }
        }
        svpwm.Ualphabeta = FOC_Rev_Park(est_output->Uqd, est_output->electric_angle);
        if(error_code == FOC_ERROR_NONE)
        {
            FOC_SVPWM(&svpwm, bus_sense.Vbus);
            driver.SetOutput(svpwm.compare_a, svpwm.compare_b, svpwm.compare_c);
        }
        else EmergencyStop();
    }
}

template<class A, class B, class C>
void FOC<A, B, C>::UpdateMidInterval(float Ts)
{
    main_estimator->UpdateMidInterval(Ts);
#ifdef FOC_USING_TEMP_PROBE
    if(mcu_temp != nullptr)
    {
        if(*mcu_temp > config.mcu_temp_limit) error_code |= FOC_ERROR_MCU_OVERTEMP;
    }
    if(motor_temp != nullptr)
    {
        if(*motor_temp > config.motor_temp_limit) error_code |= FOC_ERROR_MOTOR_OVERTEMP;
    }
    if(mosfet_temp != nullptr)
    {
        if(*mosfet_temp > config.motor_temp_limit) error_code |= FOC_ERROR_MOTOR_OVERTEMP;
    }
#endif
    // Overspeed protection
    float temp = shaft_to_origin(config.motor.max_mechanic_speed, config.motor.gear_ratio);
    est_input.target_speed = _constrain(est_input.target_speed, -temp, temp);
    if(config.max_speed > 0.0f)
    {
        if(ABS(est_output->estimated_speed) > shaft_to_origin(config.max_speed, config.motor.gear_ratio))
        {
            overspeed_timer += Ts;
            if(overspeed_timer > config.overspeed_detect_time) error_code |= FOC_ERROR_OVERSPEED;
        }
        else overspeed_timer = 0.0f;
    }
    // Sync target with current pos
    if(!output_state)
    {
        trajController.task_done = true;
        est_input.target_pos = est_output->estimated_raw_angle;
        trajController.final_pos = est_output->estimated_raw_angle;
    }
}

template<class A, class B, class C>
void FOC<A, B, C>::UpdateIdleTask(float Ts)
{
    bus_sense.BusSenseUpdate();
    main_estimator->UpdateIdleTask(Ts);
}

template<class A, class B, class C>
bool FOC<A, B, C>::GetTrajPosState()
{
    if(output_state)
    {
        if(mode >= MODE_POSITION)
        {
            if(ABS(est_input.target_pos - est_output->estimated_raw_angle) <= DEG2RAD(1.0f) && ABS(est_output->estimated_speed) <= 1.0f)
            {
                if(mode == MODE_TRAJECTORY) 
                {
                    if(trajController.GetState()) return true;
                }
                else return true;
            }
        }
    }
    return false;
}

template<class A, class B, class C>
void FOC<A, B, C>::SetOutputState(bool state)
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
void FOC<A, B, C>::EmergencyStop()
{
    output_state = false;
    est_output->Uqd.q = 0.0f;
    est_output->Uqd.d = 0.0f;
    svpwm.Ualphabeta.alpha = 0.0f;
    svpwm.Ualphabeta.beta = 0.0f;
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

template<class A, class B, class C>
template<class T>
void FOC<A, B, C>::AttachMainEstimator()
{
    static_assert(std::is_base_of<EstimatorBase, T>::value, "Estimator must be derived from EstimatorBase");
    main_estimator = std::make_unique<T>(est_input, config);
}

template<class A, class B, class C>
template<class T>
T* FOC<A, B, C>::GetMainEstimator()
{
    static_assert(std::is_base_of<EstimatorBase, T>::value, "Estimator must be derived from EstimatorBase");
    return static_cast<T*>(main_estimator.get());
}

#ifdef FOC_USING_AUX_ESTIMATOR
template<class A, class B, class C>
template<class T>
void FOC<A, B, C>::AttachAuxEstimator()
{
    static_assert(std::is_base_of<EstimatorBase, T>::value, "Estimator must be derived from EstimatorBase");
    aux_estimator = std::make_unique<T>(est_input, config);
}

template<class A, class B, class C>
template<class T>
T* FOC<A, B, C>::GetAuxEstimator()
{
    static_assert(std::is_base_of<EstimatorBase, T>::value, "Estimator must be derived from EstimatorBase");
    return static_cast<T*>(aux_estimator.get());
}

template<class A, class B, class C>
void FOC<A, B, C>::SwitchEstimator(uint8_t index)
{
    if(index == 0)
    {
        is_main_est = true;
        est_output = &main_estimator.get()->output;
    }
    else if(aux_estimator != nullptr)
    {
        is_main_est = false;
        est_output = &main_estimator.get()->output;
    }
}
#endif

#ifdef FOC_USING_EXTRA_MODULE
template<class A, class B, class C>
template<class T>
void FOC<A, B, C>::AppendModule()
{
    static_assert(std::is_base_of<ModuleBase, T>::value, "Extra module must be derived from ModuleBase");
    extra_module = std::make_unique<T>();
}

template<class A, class B, class C>
template<class T>
T* FOC<A, B, C>::GetModule()
{
    static_assert(std::is_base_of<ModuleBase, T>::value, "Extra module must be derived from ModuleBase");
    return static_cast<T*>(extra_module.get());
}
#endif

// init structs
template<class T_DriverBase, class T_CurrentSenseBase, class T_BusSenseBase>
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
            .zero_elec_angle = 0.0f,
            .pole_pair = 1,
        },
        .mcu_temp_limit = 80.0f,  // limit MCU max temp to 80 degree
        .motor_temp_limit = 80.0f, 
        .align_current = 0.6f,
        .current_bandwidth = 10000.0f,
        .current_damping_coefficient = 0.707f,
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
#include "ascii_protocol.hpp"

#pragma GCC pop_options

#endif