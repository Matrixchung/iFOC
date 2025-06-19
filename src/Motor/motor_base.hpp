#pragma once

#include "../Common/task_processor.hpp"
#include "driver_base.hpp"
#include "../Sense/curr_sense_base.hpp"
#include "../Sense/bus_sense_base.hpp"
#include "../Encoder/encoder_base.hpp"
#include "../Encoder/task_update_encoder.hpp"
#include "../DataType/Headers/Base/motor_error.h"
#include "../DataType/Headers/Base/motor_state.h"
#include "../DataType/Headers/Base/motor_control_mode.h"
#include "../DataType/board_config.hpp"
#include "../Protocol/protocol_base.hpp"
#include "motion.hpp"
#include "config_nvm_wrapper.hpp"
#include "queue.h"

namespace iFOC
{
namespace _const
{
    static constexpr uint8_t MOTOR_CONFIG_STORE_SECTOR_BEGIN = NVM_BOARD_CONFIG_STORE_SECTOR + 1;
}

class MotorTaskTimes
{
public:

};

template<uint8_t shunt_count>
class MotorBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(MotorBase);
protected:
    using MotorError = iFOC::DataType::Base::MotorError;
public:
    explicit MotorBase(uint8_t _id) : internal_id(_id) {};
    MotorBase() : MotorBase(SYSTEM_MOTOR_NUM++) {};
    /// Initialize the motor instance. You must have at least TWO processes completed before: \n
    /// 1) LinkDriver() \n
    /// 2) motor.config.LinkNVMInterface() \n
    /// \param initTIM whether init the driver's TIMER instance. If choose not, \n
    ///                all timers could be later initialized synchronously.
    /// \return FuncRetCode, OK means init successful.
    virtual FuncRetCode Init(bool initTIM) = 0;

    /// Attempt to arm the motor (put everything into work) \n
    /// If failed, a specific MotorError enum will be returned.
    /// \return MotorError, if attempt to arm the motor was failed.
    virtual MotorError Arm() = 0;

    /// Disarm the motor.
    virtual void Disarm() = 0;

    /// Disarm the motor with given MotorError.
    /// \param e MotorError type
    virtual void DisarmWithError(MotorError e) = 0;

    /// Get current motion info, with transformation to different references and unit agreements.
    /// \param dest destination to store the transformed Motion info
    /// \param ref_frame reference frame selection, options: {ELEC, BASE, OUTPUT}
    /// \param torque_unit torque unit selection, options: {AMP, NM}
    /// \param speed_unit speed unit selection, options: {RADS, DEGS, REVS, RPM}
    /// \param pos_unit pos unit selection, options: {RAD, DEG, REV}
    virtual void GetCurrentMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) = 0;

    /// Get target motion info, with transformation to different references and unit agreements. \n
    /// Target motion are handled as ref == BASE internally. \n
    /// Note that a ref_frame == ELEC will result in the same output as BASE.
    /// \param dest destination to store the transformed Motion info
    /// \param ref_frame reference frame selection, options: {BASE, OUTPUT}
    /// \param torque_unit torque unit selection, options: {AMP, NM}
    /// \param speed_unit speed unit selection, options: {RADS, DEGS, REVS, RPM}
    /// \param pos_unit pos unit selection, options: {RAD, DEG, REV}
    virtual void GetTargetMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) = 0;

    /// Set target motion, with transformation to different references and unit agreements. \n
    /// Target motion with ref == ELEC will be IGNORED!
    /// \param motion target Motion struct, with reference frame and units stored in std::pair.second
    virtual void SetTargetMotion(Motion& motion) = 0;

    [[nodiscard]] Motion GetCurrentMotionStruct(Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit);
    [[nodiscard]] Motion GetTargetMotionStruct(Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit);

    __fast_inline void DispatchRTTasks(float Ts);
    __fast_inline void DispatchMidTasks(float Ts);
    FuncRetCode AppendTask(Task* task);
    FuncRetCode PushFrontTask(Task* task);
    FuncRetCode InsertTaskBeforeName(const char* next, Task* task);
    FuncRetCode InsertTaskAfterName(const char* prev, Task* task);
    FuncRetCode RemoveTaskByName(const char* name);
    std::optional<Task*> GetTaskByName(const char* name);
    template<typename... Args>
    void BypassTaskByName(Args... args);
    template<typename... Args>
    void UnbypassTaskByName(Args... args);
    [[nodiscard]] __fast_inline Sense::BusSenseBase * GetBusSense();
    __fast_inline void LinkBusSense(Sense::BusSenseBase *bus);
    template<uint8_t other_shunt> void LinkBusSense(MotorBase<other_shunt>& other);
    [[nodiscard]] __fast_inline uint8_t GetInternalID();
    // void ThrowError(MotorError e);
    __fast_inline void ClearError();
    [[nodiscard]] __fast_inline MotorError GetError();
    [[nodiscard]] __fast_inline bool CheckError(MotorError e);
    FuncRetCode AppendEncoder(Encoder::EncoderBase* encoder);
    FuncRetCode RemoveEncoderByName(const char* name);
    FuncRetCode RemoveEncoderByIndex(uint8_t index);
    [[nodiscard]] std::optional<Encoder::EncoderBase*> GetEncoderByName(const char* name);
    [[nodiscard]] __fast_inline uint8_t GetPrimaryEncoderIndex();
    [[nodiscard]] __fast_inline std::optional<Encoder::EncoderBase*> GetPrimaryEncoder();
    [[nodiscard]] __fast_inline const Vector<Encoder::EncoderBase*>& GetEncoders();
    __fast_inline void SetPrimaryEncoderIndex(uint8_t idx);
    __fast_inline void RegisterProtocol(ProtocolBase* protocol);
    [[nodiscard]] __fast_inline MotorState GetCurrentState();
    [[nodiscard]] __fast_inline bool IsArmed();
    [[nodiscard]] __fast_inline MotorControlMode GetControlMode();
    __fast_inline void SetControlMode(MotorControlMode mode);
    __fast_inline TaskProcessor& GetTaskProcessor();
    __fast_inline void UpdateWatchdog();
protected:
    TaskProcessor tasks;
    /// \brief used to store all associated encoders (primary encoder, auxiliary encoder, sensorless...)
    Vector<Encoder::EncoderBase*> encoders{};
    /// \brief used to store all registered communication protocol
    Vector<ProtocolBase*> protocols{};

    Driver::DriverBase* driver{};
    Sense::CurrSenseBase<shunt_count>* curr_sense{};
    Sense::BusSenseBase* bus_sense{};

    /// \brief Watchdog feature: if enabled, watchdog_cnt should be periodically updated by any of the user input (set to 0),
    /// otherwise, when the counter (added up in Mid task) exceeds preset limit, a motor shutdown will be immediately triggered.
    uint32_t watchdog_cnt = 0;

    /// \brief used to determine internal index in case of a CPU handling multiple motor instances \n
    ///        used in following areas: NVM config R/W, updating BusSense, handling bus communication...
    uint8_t internal_id = 0;
    bool is_armed = false;
    /// \brief Primary Encoder index which has been selected as data source
    uint8_t primary_encoder_idx = 0;
    MotorError error = MotorError::NONE;
    MotorState current_state = MotorState::IDLE;
    MotorControlMode control_mode = MotorControlMode::CTRL_MODE_POSITION;
};

template<uint8_t shunt_count>
Motion MotorBase<shunt_count>::GetCurrentMotionStruct(Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit)
{
    Motion ret{};
    GetCurrentMotion(ret, ref_frame, torque_unit, speed_unit, pos_unit);
    return ret;
}

template<uint8_t shunt_count>
Motion MotorBase<shunt_count>::GetTargetMotionStruct(Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit)
{
    Motion ret{};
    GetTargetMotion(ret, ref_frame, torque_unit, speed_unit, pos_unit);
    return ret;
}

template<uint8_t shunt_count>
__fast_inline void MotorBase<shunt_count>::DispatchRTTasks(float Ts)
{
    tasks.RTTaskScheduler(Ts);
}

template<uint8_t shunt_count>
__fast_inline void MotorBase<shunt_count>::DispatchMidTasks(float Ts)
{
    watchdog_cnt++;
    tasks.MidTaskScheduler(Ts);
}

template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::AppendTask(Task *task)
{
    task->_motor = (void*)this;
    return tasks.AppendTask(task);
}

template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::PushFrontTask(Task *task)
{
    task->_motor = (void*)this;
    return tasks.PushFrontTask(task);
}

template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::InsertTaskBeforeName(const char *next, Task *task)
{
    task->_motor = (void*)this;
    return tasks.InsertTaskBeforeName(next, task);
}

template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::InsertTaskAfterName(const char *prev, Task *task)
{
    task->_motor = (void*)this;
    return tasks.InsertTaskAfterName(prev, task);
}

// we should prevent UpdateEncoderTask get removed from here
template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::RemoveTaskByName(const char *name)
{
    if(GetEncoderByName(name)) return FuncRetCode::INVALID_INPUT;
    return tasks.RemoveTaskByName(name);
}

// we should prevent UpdateEncoderTask get found from here
template<uint8_t shunt_count>
std::optional<Task*> MotorBase<shunt_count>::GetTaskByName(const char *name)
{
    if(GetEncoderByName(name)) return std::nullopt;
    return tasks.GetTaskByName(name);
}

template<uint8_t shunt_count>
template<typename... Args>
void MotorBase<shunt_count>::BypassTaskByName(Args... args)
{
    tasks.BypassTaskByName<Args ...>(args ...);
}

template<uint8_t shunt_count>
template<typename... Args>
void MotorBase<shunt_count>::UnbypassTaskByName(Args... args)
{
    tasks.UnbypassTaskByName<Args ...>(args ...);
}

template<uint8_t shunt_count>
Sense::BusSenseBase *MotorBase<shunt_count>::GetBusSense()
{
    return (Sense::BusSenseBase *) bus_sense;
}

template<uint8_t shunt_count>
void MotorBase<shunt_count>::LinkBusSense(Sense::BusSenseBase *bus)
{
    bus_sense = bus;
}

template<uint8_t shunt_count>
template<uint8_t other_shunt>
void MotorBase<shunt_count>::LinkBusSense(MotorBase<other_shunt> &other)
{
    bus_sense = other.bus_sense;
    if(internal_id == 0) internal_id = other.internal_id + 1;
}

template<uint8_t shunt_count>
uint8_t MotorBase<shunt_count>::GetInternalID()
{
    return internal_id;
}

// template<uint8_t shunt_count>
// void MotorBase<shunt_count>::ThrowError(MotorBase::MotorError e)
// {
//     if(e < MotorError::_NON_CRITICAL_ERROR_ABOVE_) error = e;
//     else DisarmWithError(e);
// }

template<uint8_t shunt_count>
void MotorBase<shunt_count>::ClearError()
{
    error = MotorError::NONE;
}

template<uint8_t shunt_count>
MotorError MotorBase<shunt_count>::GetError()
{
    return error;
}

template<uint8_t shunt_count>
bool MotorBase<shunt_count>::CheckError(MotorError e)
{
    return to_underlying(error) & to_underlying(e);
}

template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::AppendEncoder(Encoder::EncoderBase *encoder)
{
    auto ret = AppendTask(new Encoder::UpdateEncoderTask(encoder));
    if(ret == FuncRetCode::OK) encoders.push_back(encoder);
    return ret;
}

template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::RemoveEncoderByName(const char *name)
{
    // Remove update task first
    auto ret = tasks.RemoveTaskByName(name);
    if(ret == FuncRetCode::OK)
    {
        for(size_t i = 0; i < encoders.size(); i++)
        {
            if(*(encoders[i]) == name)
            {
                delete encoders[i]; // call destructor
                encoders.erase(encoders.begin() + i);
                return FuncRetCode::OK;
            }
        }
        return FuncRetCode::INVALID_INPUT;
    }
    return ret;
}

template<uint8_t shunt_count>
FuncRetCode MotorBase<shunt_count>::RemoveEncoderByIndex(uint8_t index)
{
    if(encoders.size() <= index) return FuncRetCode::PARAM_NOT_EXIST;
    auto ret = tasks.RemoveTaskByName(encoders[index]->GetName());
    if(ret == FuncRetCode::OK)
    {
        delete encoders[index];
        encoders.erase(encoders.begin() + index);
        return FuncRetCode::OK;
    }
    return ret;
}

template<uint8_t shunt_count>
std::optional<Encoder::EncoderBase*> MotorBase<shunt_count>::GetEncoderByName(const char *name)
{
    for(auto enc : encoders)
    {
        if(*enc == name) return std::make_optional(enc);
    }
    return std::nullopt;
}

template<uint8_t shunt_count>
uint8_t MotorBase<shunt_count>::GetPrimaryEncoderIndex()
{
    return primary_encoder_idx;
}

template<uint8_t shunt_count>
std::optional<Encoder::EncoderBase*> MotorBase<shunt_count>::GetPrimaryEncoder()
{
    if(encoders.size() <= primary_encoder_idx) return std::nullopt;
    return encoders[primary_encoder_idx];
}

template<uint8_t shunt_count>
const Vector<Encoder::EncoderBase*>& MotorBase<shunt_count>::GetEncoders()
{
    return encoders;
}

template<uint8_t shunt_count>
__fast_inline void MotorBase<shunt_count>::SetPrimaryEncoderIndex(uint8_t idx)
{
    primary_encoder_idx = idx;
}

template<uint8_t shunt_count>
void MotorBase<shunt_count>::RegisterProtocol(ProtocolBase *protocol)
{
    protocol->_motor = (void*)this;
    protocol->Init();
    protocols.push_back(protocol);
}

template<uint8_t shunt_count>
__fast_inline MotorState MotorBase<shunt_count>::GetCurrentState()
{
    return current_state;
}

template<uint8_t shunt_count>
__fast_inline bool MotorBase<shunt_count>::IsArmed()
{
    return is_armed;
}

template<uint8_t shunt_count>
__fast_inline MotorControlMode MotorBase<shunt_count>::GetControlMode()
{
    return control_mode;
}

template<uint8_t shunt_count>
__fast_inline void MotorBase<shunt_count>::SetControlMode(MotorControlMode mode)
{
    control_mode = mode;
}

template<uint8_t shunt_count>
__fast_inline TaskProcessor& MotorBase<shunt_count>::GetTaskProcessor()
{
    return tasks;
}

template<uint8_t shunt_count>
__fast_inline void MotorBase<shunt_count>::UpdateWatchdog()
{
    watchdog_cnt = 0;
}

}