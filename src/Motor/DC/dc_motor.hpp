#pragma once

#include "../motor_base.hpp"
#include "dc_driver_base.hpp"
#include "../../DataType/Headers/Config/Motor/dc_motor_config.h"

namespace iFOC
{
namespace DC
{
    // class CurrLoopBase;
    class SpeedLoopPI;
    struct TaskTimeSet
    {
        TaskTimer rt_main_task{};
        TaskTimer mid_interval_task{};
    };
}

class DCMotor final : public MotorBase<1>
{
public:
    using MotorBase::MotorBase;

    FuncRetCode Init(bool initTIM) override;

    bool Arm() override;
    void Disarm() override;
    void DisarmWithError(MotorError e) override;
    void GetCurrentMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) override;
    void GetTargetMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) override;
    void SetTargetMotion(Motion& motion) override;
    __fast_inline void LinkDriver(Driver::DCDriverImpl auto *drv) { driver = drv; }
    // __fast_inline void LinkCurrSense(Sense)
    [[nodiscard]] __fast_inline Driver::DCDriverBase *GetDriver() const { return (Driver::DCDriverBase *) driver; };
    [[nodiscard]] __fast_inline Sense::CurrSenseBase<1> *GetCurrSense() const { return curr_sense; };
    __fast_inline auto& GetConfig() { return config.GetConfig(); }
    void ResetDefaultConfig();

    DC::SpeedLoopPI* GetSpeedLoop();

    DataType::ConfigNVMWrapper<DataType::Config::Motor::DCMotorConfig> config{(ProtoHeader)(to_underlying(ProtoHeader::DC_MOTOR_CONFIG_M1) + GetInternalID()),
                                                                               (uint8_t)(internal_id + _const::MOTOR_CONFIG_STORE_SECTOR_BEGIN)};

    /// READ_ONLY ///
    real_t Idc_measured = 0.0f; // [A], Given by: UpdateSenseTask
    real_t Idc_target = 0.0f;   // [A], Given by: Speed Loop
    real_t Udc_target = 0.0f;   // [V], Given by: Current Loop
    DC::TaskTimeSet task_times{};
private:
    Motion current_target{.ref = Motion::Ref::BASE}; // For DC motor, ELEC ref == BASE
};

}