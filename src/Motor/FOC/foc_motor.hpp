#pragma once

#include "../motor_base.hpp"
#include "foc_driver_base.hpp"
#include "../../DataType/lookup_table.hpp"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif

#include "../../DataType/Headers/Config/Motor/foc_motor_config.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "Task/foc_task_state_machine.hpp"

#if (FLASH_USER_AREA_SIZE) >= (32 * 1024) && (defined (USE_FLASHDB) || defined (USE_EASYFLASH)) // also requires configTOTAL_HEAP_SIZE >= 64K
#define FOC_ANTICOGGING_AVAILABLE (true)
#else
#define FOC_ANTICOGGING_AVAILABLE (false)
#endif

namespace iFOC
{
namespace FOC
{
    class CurrLoopBase;
    class SpeedLoopBase;
    struct TaskTimeSet
    {
        TaskTimer rt_main_task{};             // Low side on -> |main task| -> task ended -> low side off -> remaining task
        TaskTimer rt_waiting_for_remaining{}; // Low side on -> main task -> |task ended -> low side off| -> remaining task
        TaskTimer rt_remaining_task{};        // Low side on -> main task -> task ended -> low side off -> |remaining task|
        TaskTimer mid_interval_task{};        // |medium interval task|
    };
    constexpr char ANTICOGGING_LUT_DB_KEY_PREFIX[] = "atc";
    constexpr char ANTICOGGING_FRICTION_DB_KEY_PREFIX[] = "atf";
    constexpr uint16_t ANTICOGGING_LUT_POINTS = 1801; // segments = 1801 - 1, 0.2°/LSB
}
#pragma pack(push, 4)
class FOCMotor final : public MotorBase<3>
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(FOCMotor);
public:
    using MotorBase::MotorBase;
    /// Initialize the FOCMotor instance. You must have THREE processes completed before: \n
    /// 1) LinkDriver() \n
    /// 2) LinkCurrSense() \n
    /// 3) LinkBusSense() \n
    /// \param initTIM whether init the driver's TIMER instance. If choose not, \n
    ///                all timers could be later initialized synchronously.
    /// \return FuncRetCode, OK means init successful.
    FuncRetCode Init(bool initTIM) override;

    bool Arm() override;
    void Disarm() override;
    void DisarmWithError(MotorError e) override;
    void GetCurrentMotion(Motion& ret, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) override;
    void GetTargetMotion(Motion& ret, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) override;
    void SetTargetMotion(Motion& motion) override;
    void SetTrajectoryTargetMotion(Motion& motion, bool is_s_curve) override;
    FuncRetCode AppendEncoder(Encoder::EncoderBase* encoder) override;
    [[nodiscard]] __fast_inline MotorState GetCurrentState() const override { return state_machine.GetState(); }
    __fast_inline void LinkDriver(Driver::FOCDriverImpl auto *drv) { driver = drv; }
    __fast_inline void LinkCurrSense(Sense::FOCCurrSenseImpl auto *curr) { curr_sense = curr; }
    __fast_inline Driver::FOCDriverBase *GetDriver() const { return static_cast<Driver::FOCDriverBase *>(driver); };
    __fast_inline Sense::CurrSenseBase<3> *GetCurrSense() const { return static_cast<Sense::CurrSenseBase<3> *>(curr_sense); };
    __fast_inline auto& GetConfig() { return config.GetConfig(); };
    void ResetDefaultConfig();
    FOC::CurrLoopBase* GetCurrLoop();
    FOC::SpeedLoopBase* GetSpeedLoop();
    FuncRetCode ToggleBeepIdentify();
// private:
    // friend class iFOC::Task; // Friendship is neither inherited nor transitive.
    FOC::StateMachineTask state_machine;
    DataType::ConfigNVMWrapper<DataType::Config::Motor::FOCMotorConfig> config{(ProtoHeader)(to_underlying(ProtoHeader::FOC_MOTOR_CONFIG_M1) + GetInternalID()),
                                                                               (uint8_t)(internal_id + _const::MOTOR_CONFIG_STORE_SECTOR_BEGIN)};
    /// READ_ONLY ///
#if FOC_ANTICOGGING_AVAILABLE
    DataType::LookupTable anticogging_lut{};
    // DataType::LookupTable anticogging_friction_lut{}; // currently we're not using friction compensation
#endif
    alphabeta_t Ialphabeta_measured{}; // [A], Given by: UpdateSenseTask
    qd_t Iqd_measured{}; // [A], Given by: EncoderArbiter
    qd_t Iqd_target{};   // [A], Given by: Speed Loop
    qd_t Uqd_target{};   // [V], Given by: Current Loop
    real_t elec_angle_rad = 0.0f; // [rad], Given by: EncoderArbiter
    real_t elec_omega_rad_s = 0.0f; // [rad/s], Given by: EncoderArbiter
    real_t config_max_current = 0.0f; // [A], Stored at Init() to prevent modifying max current at runtime
    real_t config_max_base_speed_rad_s = 0.0f; // [rad/s], Stored at Init() to prevent modifying max speed at runtime
    FOC::TaskTimeSet task_times{};
private:
    Motion current_target{.ref = Motion::Ref::BASE};
};
#pragma pack(pop)
}
