#pragma once

#include "../motor_base.hpp"
#include "foc_driver_base.hpp"
#include "../../DataType/Headers/Config/Motor/foc_motor_config.h"
#include "Task/foc_task_state_machine.hpp"

namespace iFOC
{
namespace FOC
{
    class CurrLoopBase;
    class SpeedLoopBase;
}
class FOCMotor final : public MotorBase<3>
{
public:
    using MotorBase<3>::MotorBase;
    /// Initialize the FOCMotor instance. You must have THREE processes completed before: \n
    /// 1) LinkDriver() \n
    /// 2) LinkCurrSense() \n
    /// 3) LinkBusSense() \n
    /// \param initTIM whether init the driver's TIMER instance. If choose not, \n
    ///                all timers could be later initialized synchronously.
    /// \return FuncRetCode, OK means init successful.
    FuncRetCode Init(bool initTIM) final;

    MotorError Arm() final;
    void Disarm() final;
    void DisarmWithError(MotorError e) final;
    void GetCurrentMotion(Motion& ret, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) final;
    void GetTargetMotion(Motion& ret, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) final;
    void SetTargetMotion(Motion& motion) final;
    void LinkDriver(Driver::FOCDriverImpl auto *drv) { driver = drv; }
    void LinkCurrSense(Sense::FOCCurrSenseImpl auto *curr) { curr_sense = curr; }
    __fast_inline Driver::FOCDriverBase *GetDriver() { return (Driver::FOCDriverBase *) driver; };
    __fast_inline Sense::CurrSenseBase<3> *GetCurrSense() { return (Sense::CurrSenseBase<3> *) curr_sense; };
    __fast_inline auto& GetConfig() { return config.GetConfig(); };
    void ResetDefaultConfig();
    std::optional<FOC::CurrLoopBase*> GetCurrLoop();
    std::optional<FOC::SpeedLoopBase*> GetSpeedLoop();
// private:
    // friend class iFOC::Task; // Friendship is neither inherited nor transitive.
    FOC::StateMachineTask state_machine;
    DataType::ConfigNVMWrapper<DataType::Config::Motor::FOCMotorConfig> config{(ProtoHeader)(to_underlying(ProtoHeader::FOC_MOTOR_CONFIG_M1) + GetInternalID()),
                                                                               (uint8_t)(internal_id + _const::MOTOR_CONFIG_STORE_SECTOR_BEGIN)};
    /// READ_ONLY ///
    alphabeta_t Ialphabeta_measured{}; // [A], Given by: UpdateSenseTask
    qd_t Iqd_measured{}; // [A], Given by: EncoderArbiter
    qd_t Iqd_target{};   // [A], Given by: Speed Loop
    qd_t Uqd_target{};   // [V], Given by: Current Loop
    real_t elec_angle_rad = 0.0f; // [rad], Given by: EncoderArbiter
    real_t elec_omega_rad_s = 0.0f; // [rad/s], Given by: EncoderArbiter
    real_t config_max_current = 0.0f; // [A], Stored at Init() to prevent modifying max current at runtime
    real_t config_max_base_speed_rad_s = 0.0f; // [rad/s], Stored at Init() to prevent modifying max speed at runtime
private:
    Motion current_target{.ref = Motion::Ref::BASE};
    std::optional<FOC::CurrLoopBase*> curr_loop;
    std::optional<FOC::SpeedLoopBase*> speed_loop;
};
}
