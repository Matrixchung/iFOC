#include "ascii_protocol.hpp"
#include "ascii_tiny_printf.hpp"

#define foc GetMotor<FOCMotor>()

template<typename T>
using MapType = std::map<T, const char*>;

using MotorState = iFOC::DataType::Base::MotorState;

const static MapType<MotorState> stateNameMap = {
        {MotorState::IDLE, "IDLE"},
        {MotorState::STARTUP_SEQUENCE, "STARTUP_SEQUENCE"},
        {MotorState::BASIC_PARAM_CALIBRATION, "BASIC_PARAM_CALIBRATION"},
        {MotorState::ENCODER_INDEX_SEARCH, "ENCODER_INDEX_SEARCH"},
        {MotorState::ENCODER_CALIBRATION, "ENCODER_CALIBRATION"},
        {MotorState::EXTEND_PARAM_CALIBRATION, "EXTEND_PARAM_CALIBRATION"},
        {MotorState::SENSORED_CLOSED_LOOP_CONTROL, "SENSORED_CLOSED_LOOP_CONTROL"},
        {MotorState::SENSORLESS_CLOSED_LOOP_CONTROL, "SENSORLESS_CLOSED_LOOP_CONTROL"}
};

using MotorControlMode = iFOC::DataType::Base::MotorControlMode;

const static MapType<MotorControlMode> controlNameMap = {
        {MotorControlMode::CTRL_MODE_POSITION, "CTRL_MODE_POSITION"},
        {MotorControlMode::CTRL_MODE_VELOCITY, "CTRL_MODE_VELOCITY"},
        {MotorControlMode::CTRL_MODE_CURRENT, "CTRL_MODE_CURRENT"},
        {MotorControlMode::CTRL_MODE_HYBRID, "CTRL_MODE_HYBRID"},
};

using Motion = iFOC::Motion;

const static MapType<Motion::Ref> motionRefNameMap = {
        {Motion::Ref::ELEC, "ELEC"},
        {Motion::Ref::BASE, "BASE"},
        {Motion::Ref::OUTPUT, "OUTPUT"},
};

const static MapType<Motion::TorqueUnit> motionTorqueNameMap = {
        {Motion::TorqueUnit::AMP, "AMP"},
        {Motion::TorqueUnit::NM, "NM"},
};

const static MapType<Motion::SpeedUnit> motionSpeedNameMap = {
        {Motion::SpeedUnit::RADS, "RADS"},
        {Motion::SpeedUnit::DEGS, "DEGS"},
        {Motion::SpeedUnit::REVS, "REVS"},
        {Motion::SpeedUnit::RPM, "RPM"},
        {Motion::SpeedUnit::HZ, "HZ"},
};

const static MapType<Motion::PosUnit> motionPosNameMap = {
        {Motion::PosUnit::RAD, "RAD"},
        {Motion::PosUnit::DEG, "DEG"},
        {Motion::PosUnit::REV, "REV"},
};

template<typename T>
const char* get_string_from_state(MapType<T> map, T s)
{
    if(const auto& it = map.find(s); it != map.end()) return it->second;
    return "UNKNOWN";
}

template<typename T>
std::optional<T> get_state_from_string(MapType<T> map, const char* str)
{
    for(const auto& pair : map)
    {
        if(!strcmp(str, pair.second)) return std::make_optional<T>(pair.first);
    }
    return std::nullopt;
}


namespace iFOC::Protocol
{
size_t splitStr(const std::string_view& source, Vector<String>& dest, const char split);
void splitData_f(char* pSource, const uint16_t Lsource, float* pDest, uint8_t* Ldest, const uint8_t max_dest_len, const uint8_t split);

ASCIIProtocolFOC::ASCIIProtocolFOC(iFOC::HAL::UARTBase *base) : uart(base)
{
    rx_buffer.fill(0);
    uart->RegisterRxHandler(std::bind(&ASCIIProtocolFOC::OnRxEvent, this, std::placeholders::_1, std::placeholders::_2));
}

void ASCIIProtocolFOC::CmdPosition(uint8_t *data, uint16_t len, bool use_checksum)
{
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id))
    {
        uint8_t target_count = 0;
        float target[4]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 4, ' ');
        if(target_count < 1 || target_count > 3)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {0.0f, target_count == 3 ? target[2] : 0.0f, io_torque_unit},
            .speed = {0.0f, target_count >= 2 ? target[1] : 0.0f, io_speed_unit},
            .pos = {target[0], 0.0f, io_pos_unit}
        };
        foc->UpdateWatchdog();
        foc->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
        foc->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

void ASCIIProtocolFOC::CmdPositionWithFF(uint8_t *data, uint16_t len, bool use_checksum)
{
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id))
    {
        uint8_t target_count = 0;
        float target[4]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 4, ' ');
        if(target_count < 1 || target_count > 3)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {target_count == 3 ? target[2] : 0.0f, 0.0f, io_torque_unit},
            .speed = {target_count >= 2 ? target[1] : 0.0f, 0.0f, io_speed_unit},
            .pos = {target[0], 0.0f, io_pos_unit}
        };
        foc->UpdateWatchdog();
        foc->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
        foc->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

void ASCIIProtocolFOC::CmdVelocity(uint8_t *data, uint16_t len, bool use_checksum)
{
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id))
    {
        uint8_t target_count = 0;
        float target[3]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 3, ' ');
        if(target_count < 1 || target_count > 2)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {target_count == 2 ? target[1] : 0.0f, 0.0f, io_torque_unit},
            .speed = {target[0], 0.0f, io_speed_unit},
            .pos = {0.0f, 0.0f, io_pos_unit}
        };
        foc->UpdateWatchdog();
        foc->SetControlMode(MotorControlMode::CTRL_MODE_VELOCITY);
        foc->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

void ASCIIProtocolFOC::CmdTorque(uint8_t *data, uint16_t len, bool use_checksum)
{
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id) && id == foc->GetInternalID())
    {
        uint8_t target_count = 0;
        float target = 0.0f;
        splitData_f((char*)(data + 4), len - 4, &target, &target_count, 1, ' ');
        if(target_count != 1)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {target, 0.0f, io_torque_unit},
            .speed = {0.0f, 0.0f, io_speed_unit},
            .pos = {0.0f, 0.0f, io_pos_unit}
        };
        foc->UpdateWatchdog();
        foc->SetControlMode(MotorControlMode::CTRL_MODE_CURRENT);
        foc->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

void ASCIIProtocolFOC::CmdFeedback(uint8_t *data, uint16_t len, bool use_checksum)
{
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 3, id) && id == foc->GetInternalID())
    {
        const auto& current = foc->GetCurrentMotionStruct(io_ref, io_torque_unit, io_speed_unit, io_pos_unit);
        GenerateResponse(use_checksum, false, "%.3f %.3f", current.pos.value, current.speed.value);
    }
}

void ASCIIProtocolFOC::CmdUpdateWatchdog(uint8_t *data, uint16_t len, bool use_checksum)
{
    if(len < 3) return;
    uint8_t dst = *(data + 2);
    if(dst - '0' == foc->GetInternalID())
    {
        foc->UpdateWatchdog();
    }
}

void ASCIIProtocolFOC::CmdReadConfig(uint8_t *data, uint16_t len, bool use_checksum)
{
    if(len < 3)
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    uint8_t dst = *(data + 2);
    if((!isdigit(dst) && dst != 'b') || (isdigit(dst) && dst - '0' > SYSTEM_MOTOR_NUM - 1))
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    if(dst == 'b')
    {
        if(foc->GetInternalID() != 0) return; // board config should be only handled by motor #0
    }
    else if(dst - '0' != foc->GetInternalID()) return;
    auto reflect = dst == 'b' ? BoardConfig.GetConfig().GetReflectMap() : foc->GetConfig().GetReflectMap();
    uint8_t *start_ptr = dst == 'b' ? (uint8_t*)(&BoardConfig.GetConfig()) : (uint8_t*)(&foc->GetConfig());
    if(len < 5) // print all config
    {
        if(dst == 'b') GenerateResponse(use_checksum, true, "Board Config:");
        else GenerateResponse(use_checksum, true, "Motor %c Config:", dst);
        for(const auto& [name, info] : reflect)
        {
            uint8_t *ptr = start_ptr + info.second;
            PrintReflectedVariable(name, ptr, info.first, use_checksum, true);
        }
        uart->StartTransmit(false);
    }
    else
    {
        uint8_t arg_len = len - 4;
        Vector<char> temp(arg_len + 2);
        memcpy(temp.data(), data + 4, arg_len);
        if(temp[arg_len - 1] != '_')
        {
            temp[arg_len] = '_';
            arg_len++;
        }
        temp[arg_len] = '\0';
        if(const auto& it = reflect.find(temp.data()); it != reflect.end())
        {
            const auto& info = it->second;
            uint8_t *ptr = start_ptr + info.second;
            PrintReflectedVariable(it->first, ptr, info.first, use_checksum, false);
            uart->StartTransmit(false);
        }
        else GenerateResponse(use_checksum, false, "invalid property");
    }
}

void ASCIIProtocolFOC::CmdWriteConfig(uint8_t *data, uint16_t len, bool use_checksum)
{
    if(len < 5)
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    uint8_t dst = *(data + 2);
    if((!isdigit(dst) && dst != 'b') || (isdigit(dst) && dst - '0' > SYSTEM_MOTOR_NUM - 1) || *(data + 3) != ' ')
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    if(dst == 'b')
    {
        if(foc->GetInternalID() != 0) return; // board config should be only handled by motor #0)
    }
    else if(dst - '0' != foc->GetInternalID()) return;
    auto reflect = dst == 'b' ? BoardConfig.GetConfig().GetReflectMap() : foc->GetConfig().GetReflectMap();
    uint8_t *start_ptr = dst == 'b' ? (uint8_t*)(&BoardConfig.GetConfig()) : (uint8_t*)(&foc->GetConfig());
    uint8_t arg_len = 1;
    while(arg_len < (len - 4) - 1)
    {
        char temp = *(data + 3 + arg_len + 1);
        if(temp == ' ' || temp < 'A' || temp > 'z') break;
        arg_len++;
    }
    float target = 0.0f;
    uint8_t target_count = 0;
    splitData_f((char*)(data + 5 + arg_len), len - 5 - arg_len, &target, &target_count, 1, ' ');
    if(target_count != 1)
    {
        GenerateResponse(use_checksum, false, "invalid argument input");
        return;
    }
    Vector<char> temp(arg_len + 2);
    memcpy(temp.data(), data + 4, arg_len);
    if(temp[arg_len - 1] != '_')
    {
        temp[arg_len] = '_';
        arg_len++;
    }
    temp[arg_len] = '\0';
    if(const auto& it = reflect.find(temp.data()); it != reflect.end())
    {
        const auto& info = it->second;
        uint8_t *ptr = start_ptr + info.second;
        switch(info.first)
        {
            case Reflection::ProtoFieldType::FLOAT:
                *(float*)ptr = target; break;
            case Reflection::ProtoFieldType::INT32:
            {
                if(target > INT32_MAX || target < INT32_MIN) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(int32_t*)ptr = (int32_t)target;
                break;
            }
            case Reflection::ProtoFieldType::INT64:
            {
                if(target > INT64_MAX || target < INT64_MIN) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(int64_t*)ptr = (int64_t)target;
                break;
            }
            case Reflection::ProtoFieldType::UINT32:
            {
                if(target > UINT32_MAX || target < 0) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(uint32_t*)ptr = (uint32_t)target;
                break;
            }
            case Reflection::ProtoFieldType::UINT64:
            {
                if(target > UINT64_MAX || target < 0) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(uint64_t*)ptr = (uint64_t)target;
                break;
            }
            case Reflection::ProtoFieldType::BOOL:
            {
                if(target > 1 || target < 0) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(uint8_t*)ptr = (uint8_t)target;
                break;
            }
            default:
            {
                GenerateResponse(use_checksum, false, "unsupported property type");
                return;
            }
        }
        GenerateResponse(use_checksum, false, "ok");
    }
    else GenerateResponse(use_checksum, false, "invalid property");
}

/*
 * Macros used by vListTask to indicate which state a task is in.
 */
#define tskRUNNING_CHAR		( 'X' )
#define tskBLOCKED_CHAR		( 'B' )
#define tskREADY_CHAR		( 'R' )
#define tskDELETED_CHAR		( 'D' )
#define tskSUSPENDED_CHAR	( 'S' )
#define tskINVALID_CHAR 	( '?' )

void ASCIIProtocolFOC::CmdSysInfo(uint8_t *data, uint16_t len, bool use_checksum)
{
    if(foc->GetInternalID() == 0)
    {
        if(len > 1)
        {
            GenerateResponse(use_checksum, false, "invalid command format");
            return;
        }
#if configUSE_TRACE_FACILITY == 1
        TaskStatus_t *pxTaskStatusArray;
        UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
        uint32_t ulTotalRunTime = 0;
        pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
        if(pxTaskStatusArray)
        {
            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
            ulTotalRunTime /= 100UL;
            char cStatus = tskINVALID_CHAR;
#if configGENERATE_RUN_TIME_STATS == 1
            uint32_t ulStatsAsPercentage = 0;
            if(ulTotalRunTime > 0)
                uart->Print(false, "   TaskName\tState\tPrio.\tMin.Stack\tCPUTime\tTaskID\r\n");
            else
#endif
                uart->Print(false, "   TaskName\tState\tPrio.\tMin.Stack\tTaskID\r\n");
            for(UBaseType_t x = 0; x < uxArraySize; x++)
            {
                switch(pxTaskStatusArray[x].eCurrentState)
                {
                    case eRunning: cStatus = tskRUNNING_CHAR; break;
                    case eReady: cStatus = tskREADY_CHAR; break;
                    case eBlocked: cStatus = tskBLOCKED_CHAR; break;
                    case eSuspended: cStatus = tskSUSPENDED_CHAR; break;
                    case eDeleted: cStatus = tskDELETED_CHAR; break;
                    case eInvalid:
                    default:
                        break;
                }
                uart->Print(false, " - %s", pxTaskStatusArray[x].pcTaskName);
#if configGENERATE_RUN_TIME_STATS == 1
                if(ulTotalRunTime > 0)
                {
                    ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;
                    if(ulStatsAsPercentage > 0UL)
                    {
                        uart->Print(false, "\t%c\t%u\t%u\t%lu%%\t%u\r\n", cStatus,
                            (uint32_t)pxTaskStatusArray[x].uxCurrentPriority,
                            (uint32_t)pxTaskStatusArray[x].usStackHighWaterMark,
                            ulStatsAsPercentage,
                            (uint32_t)pxTaskStatusArray[x].xTaskNumber);
                    }
                    else
                    {
                        uart->Print(false, "\t%c\t%u\t%u\t<1%%\t%u\r\n", cStatus,
                            (uint32_t)pxTaskStatusArray[x].uxCurrentPriority,
                            (uint32_t)pxTaskStatusArray[x].usStackHighWaterMark,
                            (uint32_t)pxTaskStatusArray[x].xTaskNumber);
                    }
                }
                else
#endif
                uart->Print(false, "\t%c\t%u\t%u\t%u\r\n", cStatus,
                            (uint32_t)pxTaskStatusArray[x].uxCurrentPriority,
                            (uint32_t)pxTaskStatusArray[x].usStackHighWaterMark,
                            (uint32_t)pxTaskStatusArray[x].xTaskNumber);
            }
            uart->StartTransmit(false);
        }
        vPortFree(pxTaskStatusArray);
#endif
        float mem_usage_now = 1.0f - ((float)xPortGetFreeHeapSize() / (float)(configTOTAL_HEAP_SIZE));
        float mem_usage_max = 1.0f - ((float)xPortGetMinimumEverFreeHeapSize() / (float)(configTOTAL_HEAP_SIZE));
        GenerateResponse(use_checksum, true, "Compile Time: %d-%d-%d %d:%d", YEAR(), MONTH(), DAY(), HOUR(), MINUTE());
        GenerateResponse(use_checksum, true, "Memory: %d/%d Bytes (%.2f,Max:%.2f)", configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize(),
                                                                                            configTOTAL_HEAP_SIZE,
                                                                                            mem_usage_now,
                                                                                            mem_usage_max);
#if defined(USE_FLASHDB)
        auto used_size = BoardConfig.GetNVMUsedSize();
        auto total_size = BoardConfig.GetNVMTotalSize();
        if(total_size > 0)
            GenerateResponse(use_checksum, true, "KVDB: %d/%d Bytes (%.2f)", used_size, total_size, ((float)used_size / (float)total_size));
#endif
        GenerateResponse(use_checksum, true, "Core clock: %d MHz", HAL::GetCoreClockHz() / 1000000);
        GenerateResponse(use_checksum, false, "HW S/N: %llu", HAL::GetSerialNumber());
    }
}

void ASCIIProtocolFOC::CmdMotorInfo(uint8_t *data, uint16_t len, bool use_checksum)
{
    if(len < 3)
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    uint8_t dst = *(data + 2);
    if(!isdigit(dst) || (isdigit(dst) && dst - '0' > SYSTEM_MOTOR_NUM - 1))
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    if(dst - '0' == foc->GetInternalID())
    {
        if(len < 4) // print motor info
        {
            GenerateResponse(use_checksum, true, "Motor %c Info: ", dst);
            GenerateResponse(use_checksum, true, " - Vbus: %.1f", foc->GetBusSense()->voltage);
            GenerateResponse(use_checksum, true, " - Ibus: %.1f", foc->GetBusSense()->current);
            GenerateResponse(use_checksum, true, " - curr_state: %s", get_string_from_state(stateNameMap, foc->state_machine.GetState()));
            GenerateResponse(use_checksum, true, " - curr_ctrl_mode: %s", get_string_from_state(controlNameMap, foc->GetControlMode()));
            GenerateResponse(use_checksum, false, " - curr_error: %d", to_underlying(foc->GetError()));
            const auto& current_motion = foc->GetCurrentMotionStruct(io_ref, io_torque_unit, io_speed_unit, io_pos_unit);
            GenerateResponse(use_checksum, false, " - c_m: %.1f/%.1f, %.1f/%.1f, %.1f/%.1f", current_motion.torque.value,
                                                                                                            current_motion.torque.limit,
                                                                                                            current_motion.speed.value,
                                                                                                            current_motion.speed.limit,
                                                                                                            current_motion.pos.value,
                                                                                                            current_motion.pos.limit);
            const auto& encoders = foc->GetEncoders();
            GenerateResponse(use_checksum, true, " - Encoders (%d):", encoders.size());
            for(size_t i = 0; i < encoders.size(); i++)
            {
                const auto& enc = encoders[i];
                switch(enc->GetEncoderType())
                {
                    case Encoder::Type::ABSOLUTE_ENCODER: GenerateResponse(use_checksum, true, "    - #%d (Abs.) %d", i + 1, enc->IsResultValid()); break;
                    case Encoder::Type::INCREMENTAL_ENCODER: GenerateResponse(use_checksum, true, "    - #%d (Inc.) %d", i + 1, enc->IsResultValid()); break;
                    case Encoder::Type::SENSORLESS_ENCODER: GenerateResponse(use_checksum, true, "    - #%d (Est.) %d", i + 1, enc->IsResultValid()); break;
                    default: GenerateResponse(use_checksum, true, "    - #%d (N/A) %d", i + 1, enc->IsResultValid()); break;
                }
                GenerateResponse(use_checksum, true, "    - sing_rad: %.5f", enc->single_round_angle_rad);
                GenerateResponse(use_checksum, false, "    - mult_rad: %.5f", enc->multi_round_angle_rad);
            }
            // print RT & mid & normal tasks call list
            const auto& task_list = foc->GetTaskProcessor().GetTaskList();
            String rt_tasks_list;
            String mid_tasks_list;
            // Vector<std::pair<UBaseType_t, const char*>> normal_tasks_vector(task_list.size());
            for(const auto& task : task_list)
            {
                if(task->IsTaskRegistered(Task::TaskType::RT_TASK))
                {
                    if(!rt_tasks_list.empty()) rt_tasks_list.append(" -> ");
                    rt_tasks_list.append(task->GetName());
                }
                if(task->IsTaskRegistered(Task::TaskType::MID_TASK))
                {
                    if(!mid_tasks_list.empty()) mid_tasks_list.append(" -> ");
                    mid_tasks_list.append(task->GetName());
                }
                // if(task->IsTaskRegistered(Task::TaskType::NORMAL_TASK))
                // {
                //     normal_tasks_vector.emplace_back(task->GetRTOSPriority(), task->GetName());
                // }
            }
            if(!rt_tasks_list.empty()) GenerateResponse(use_checksum, true, " - RT Tasks: %s", rt_tasks_list.c_str());
            if(!mid_tasks_list.empty()) GenerateResponse(use_checksum, false, " - Mid Tasks: %s", mid_tasks_list.c_str());
            // sort the normal task by priority. (higher one is prioritized)
            // std::sort(normal_tasks_vector.begin(), normal_tasks_vector.end(), [](const std::pair<UBaseType_t, const char*>& a, const std::pair<UBaseType_t, const char*>& b){
            //     return a.first > b.first;
            // });
            // if(!normal_tasks_vector.empty())
            // {
            //     uart->Print(false, " - Normal Tasks: ");
            //     for(size_t i = 0; i < normal_tasks_vector.size(); i++)
            //     {
            //         const auto& pair = normal_tasks_vector[i];
            //         if(i == normal_tasks_vector.size() - 1) uart->Print(true, "%s(%d)\r\n", pair.second, pair.first);
            //         else uart->Print(false, "%s(%d) -> ", pair.second, pair.first);
            //     }
            // }
        }
        else // request a state change
        {
            uint8_t arg_len = len - 4;
            Vector<char> temp(arg_len + 1);
            memcpy(temp.data(), data + 4, arg_len);
            temp[arg_len] = '\0';
            if(const auto result = get_state_from_string(stateNameMap, temp.data()))
            {
                GenerateResponse(use_checksum, true, "Requested state: %s", temp.data());
                GenerateResponse(use_checksum, false, "New state: %s", get_string_from_state(stateNameMap, foc->state_machine.RequestState(result.value())));
            }
            else if(const auto r = get_state_from_string(controlNameMap, temp.data()))
            {
                foc->SetControlMode(r.value());
                GenerateResponse(use_checksum, true, "Requested mode: %s", temp.data());
                GenerateResponse(use_checksum, false, "New mode: %s", get_string_from_state(controlNameMap, foc->GetControlMode()));
            }
            else GenerateResponse(use_checksum, false, "invalid state input");
        }
    }
}

void ASCIIProtocolFOC::CmdHelp(uint8_t *data, uint16_t len, bool use_checksum)
{
    // For help commands, only first motor instance will respond.
    if(foc->GetInternalID() == 0)
    {
        GenerateResponse(use_checksum, true,  "Avail cmds syntax, <optional>:");
        GenerateResponse(use_checksum, true,  " - Trajectory: t motor_id pos");
        GenerateResponse(use_checksum, true,  " - Position: q motor_id pos <vel-lim> <curr-lim>");
        GenerateResponse(use_checksum, true,  " - Position: p motor_id pos <vel-ff> <curr-ff>");
        GenerateResponse(use_checksum, true,  " - Velocity: v motor_id vel <curr-ff>");
        GenerateResponse(use_checksum, true,  " - Torque: c motor_id set_torque");
        GenerateResponse(use_checksum, true,  " - Feedback: f motor_id");
        GenerateResponse(use_checksum, true,  " - Update Watchdog: u motor_id");
        GenerateResponse(use_checksum, true,  " - Read config: r motor_id/b <property>");
        GenerateResponse(use_checksum, true,  " - Write config: w motor_id/b property float_value");
        GenerateResponse(use_checksum, true,  " - System info: i");
        GenerateResponse(use_checksum, true,  " - Motor info: m motor_id <request_state/mode>");
        GenerateResponse(use_checksum, true,  " - Save cfg: ss motor_id/b");
        GenerateResponse(use_checksum, true,  " - Erase cfg: se motor_id/b (erase only)");
        GenerateResponse(use_checksum, true,  " - Set I/O ref: sf motor_id Ref TorqueUnit SpeedUnit PosUnit");
        GenerateResponse(use_checksum, true,  " - Clear error: sc motor_id");
        GenerateResponse(use_checksum, false, " - Reboot: sr");
    }
}

void ASCIIProtocolFOC::CmdSystem(uint8_t *data, uint16_t len, bool use_checksum)
{
    if(len < 2)
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    switch(*(data + 1))
    {
        case 's': // Save cfg: ss motor_id / ss b
        {
            if(len != 4 || (!isdigit(*(data + 3)) && *(data + 3) != 'b') || (isdigit(*(data + 3)) && *(data + 3) - '0' > SYSTEM_MOTOR_NUM - 1))
            {
                if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                return;
            }
            if(foc->state_machine.GetState() != MotorState::IDLE) // must in IDLE state to save the config
            {
                GenerateResponse(use_checksum, false, "state not in IDLE");
                return;
            }
            if(foc->GetInternalID() == 0)
            {
                if(*(data + 3) == 'b')
                {
                    auto ret = BoardConfig.SaveNVMConfig();
                    if(ret == FuncRetCode::OK) GenerateResponse(use_checksum, false, "board config save ok");
                    else GenerateResponse(use_checksum, false, "board config save failed:%d", to_underlying(ret));
                    return;
                }
            }
            if(*(data + 3) - '0' == foc->GetInternalID())
            {
                auto ret = foc->config.SaveNVMConfig();
                if(ret == FuncRetCode::OK) GenerateResponse(use_checksum, false, "motor %d config save ok", foc->GetInternalID());
                else GenerateResponse(use_checksum, false, "motor %d config save failed:%d", foc->GetInternalID(), to_underlying(ret));
                return;
            }
            break;
        }
        case 'e': // Erase cfg: se motor_id / se b (only erase, not save)
        {
            if(len != 4 || (!isdigit(*(data + 3)) && *(data + 3) != 'b') || (isdigit(*(data + 3)) && *(data + 3) - '0' > SYSTEM_MOTOR_NUM - 1))
            {
                if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                return;
            }
            if(foc->state_machine.GetState() != MotorState::IDLE) // must in IDLE state to erase the config
            {
                GenerateResponse(use_checksum, false, "state not in IDLE");
                return;
            }
            if(foc->GetInternalID() == 0)
            {
                if(*(data + 3) == 'b')
                {
                    BoardConfig.GetConfig().clear();
                    GenerateResponse(use_checksum, false, "board config erase ok");
                    return;
                }
            }
            if(*(data + 3) - '0' == foc->GetInternalID())
            {
                foc->GetConfig().clear();
                GenerateResponse(use_checksum, false, "motor %d config erase ok", foc->GetInternalID());
                return;
            }
            break;
        }
        case 'c': // clear error
        {
            if(len != 4)
            {
                if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                break;
            }
            if(*(data + 3) - '0' == foc->GetInternalID())
            {
                foc->ClearError();
                GenerateResponse(use_checksum, false, "ok");
            }
            break;
        }
        case 'r': // Reboot: sr
        {
            if(foc->GetInternalID() == 0)
            {
                if(len == 2) HAL::SystemReboot();
                else GenerateResponse(use_checksum, false, "invalid command format");
            }
            break;
        }
        case 'f': // Set I/O Reference frame: sf motor_id Ref TorqueUnit SpeedUnit PosUnit
        {
            if(len < 4)
            {
                if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                break;
            }
            if(*(data + 3) - '0' == foc->GetInternalID())
            {
                if(len < 6)
                {
                    GenerateResponse(use_checksum, true, "Set I/O ref: sf motor_id Ref TorqueUnit SpeedUnit PosUnit");
                    GenerateResponse(use_checksum, true, "usage: sf id ELEC/BASE/OUTPUT AMP/NM RADS/DEGS/REVS/RPM/HZ RAD/DEG/REV");
                    GenerateResponse(use_checksum, false, "Current ref:%s, torque:%s, speed:%s, pos: %s",
                                     get_string_from_state(motionRefNameMap, io_ref),
                                     get_string_from_state(motionTorqueNameMap, io_torque_unit),
                                     get_string_from_state(motionSpeedNameMap, io_speed_unit),
                                     get_string_from_state(motionPosNameMap, io_pos_unit));
                }
                else
                {
                    std::string_view view((const char*)(data + 5), len - 5);
                    Vector<String> result;
                    result.reserve(4);
                    size_t arg_count = splitStr(view, result, ' ');
                    if(arg_count != 4)
                    {
                        GenerateResponse(use_checksum, false, "invalid reference count: %d", arg_count);
                        break;
                    }
                    if(const auto& ret = get_state_from_string(motionRefNameMap, result[0].c_str())) io_ref = ret.value();
                    else
                    {
                        GenerateResponse(use_checksum, false, "invalid ref frame: %s", result[0].c_str());
                        break;
                    }
                    if(const auto& ret = get_state_from_string(motionTorqueNameMap, result[1].c_str())) io_torque_unit = ret.value();
                    else
                    {
                        GenerateResponse(use_checksum, false, "invalid torque unit: %s", result[1].c_str());
                        break;
                    }
                    if(const auto& ret = get_state_from_string(motionSpeedNameMap, result[2].c_str())) io_speed_unit = ret.value();
                    else
                    {
                        GenerateResponse(use_checksum, false, "invalid speed unit: %s", result[2].c_str());
                        break;
                    }
                    if(const auto& ret = get_state_from_string(motionPosNameMap, result[3].c_str())) io_pos_unit = ret.value();
                    else
                    {
                        GenerateResponse(use_checksum, false, "invalid pos unit: %s", result[3].c_str());
                        break;
                    }
                    GenerateResponse(use_checksum, false, "ok");
                }
            }
            break;
        }
        default:
        {
            if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
            break;
        }
    }
}

void ASCIIProtocolFOC::CmdUnknown(uint8_t *data, uint16_t len, bool use_checksum)
{
    // For unknown commands, only first motor instance will respond.
    if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "unknown command");
}

void ASCIIProtocolFOC::ProcessEachValidLine(uint8_t *data, uint16_t len)
{
    uint8_t calc_checksum = 0;
    uint16_t checksum_start_len = 0xFFFF;
    // Step #1: Calculate the data checksum, find the OPTIONAL checksum asterisk
    //          and filter out the comments (starts with ';')
    for(uint16_t valid_len = 1; valid_len <= len; valid_len++)
    {
        uint8_t c = *(data + valid_len - 1);
        if(c == ';')
        {
            len = valid_len - 1;
            break;
        }
        if(checksum_start_len > valid_len)
        {
            if(c == '*') checksum_start_len = valid_len;
            else calc_checksum ^= c;
        }
    }

    // // Step #2: Copy to local buffer, and insert '\0' termination for sscanf()
    // // scanf() is a pretty slow and dangerous function in embedded systems
    // len = MIN(len, line_buffer.size() - 1);
    // memcpy(line_buffer.data(), data, len);
    // line_buffer[len] = '\0';

    // Step #2: if the command used checksum, we should read out and check
    bool use_checksum = (checksum_start_len < len);
    unsigned int readout_checksum = 0;
    if(use_checksum)
    {
        float temp = 0.0f;
        uint8_t numscan = 0;
        splitData_f((char*)(data + checksum_start_len), len - checksum_start_len, &temp, &numscan, 2, ' ');
        readout_checksum = (unsigned int)temp;
        if(numscan != 1 || readout_checksum != calc_checksum) return;
        len = checksum_start_len - 1;
        // line_buffer[len] = '\0'; // insert termination at new position
    }

    // Step #3: trim the head and tail, removing useless spaces
    // NOTE: this might cause problem when using checksums
    while(len)
    {
        if(data[0] == ' ')
        {
            data++;
            len--;
        }
        else if(data[len - 1] == ' ')
        {
            len--;
        }
        else break;
    }

    // Step #4: handle real command type
    switch(data[0])
    {
        case 'q': CmdPosition(data, len, use_checksum); break;
        case 'p': CmdPositionWithFF(data, len, use_checksum); break;
        case 'v': CmdVelocity(data, len, use_checksum); break;
        case 'c': CmdTorque(data, len, use_checksum); break;
        case 'f': CmdFeedback(data, len, use_checksum); break;
        case 'u': CmdUpdateWatchdog(data, len, use_checksum); break;
        case 'r': CmdReadConfig(data, len, use_checksum); break;
        case 'w': CmdWriteConfig(data, len, use_checksum); break;
        case 'i': CmdSysInfo(data, len, use_checksum); break;
        case 'm': CmdMotorInfo(data, len, use_checksum); break;
        case 'h': CmdHelp(data, len, use_checksum); break;
        case 's': CmdSystem(data, len, use_checksum); break;
        default: CmdUnknown(data, len, use_checksum); break;
    }

    // // Test print of the buffer
    // char *ptr = reinterpret_cast<char*>(data);
    // do {
    //     auto write = uart->WriteBytes(reinterpret_cast<const uint8_t *>(ptr), len);
    //     if(write < len) uart->StartTransmit(false);
    //     ptr += write;
    //     len -= write;
    // } while(len > 0);
    // if(use_checksum)
    // {
    //     uart->Print(0, "cc:%d,rc:%d", calc_checksum, readout_checksum);
    // }
    // uart->Print(0, "\n");
    // uart->StartTransmit(false);
}

/// Note 1: New data comes in, maybe containing multiple EOLs or no EOL. So we need to append it to our own
///         buffer, and search the whole section. If EOF matched, we can pop them out. If no match, we do nothing. \n
/// Note 2: If our buffer is full the next time new data in, we should pop out the same size of data from
///         the head, and push new data into the buffer, and repeat (1). \n
/// Note 3: Typically, those processes mentioned above suit dual-end data structures like std::deque, however
///         we will do this in a FIXED-SIZE std::array, to minimalize potential memory fragments or leaks. \n
bool ASCIIProtocolFOC::OnRxEvent(uint8_t *data, uint16_t len)
{
    do // if income len > rx_buffer.size(), we will process multiple times
    {
        uint16_t actual_len = MIN(len, rx_buffer.max_size());
        // Step #1: refers to Note 2
        if(rx_ptr + actual_len > rx_buffer.max_size())
        {
            uint16_t len_to_be_removed = rx_ptr + actual_len - rx_buffer.max_size();
            rx_ptr -= len_to_be_removed; // rx_len after removal
            memmove(rx_buffer.data(), rx_buffer.data() + len_to_be_removed, rx_ptr);
        }
        memcpy(rx_buffer.data() + rx_ptr, data, actual_len);
        rx_ptr += actual_len;
        // Step #2: we have received the datas into buffer. Here we can do a test print (for debug)
        //          and now its time to search our buffer. The whole length is rx_ptr.
        while(true)
        {
            uint16_t first_eol_section_ptr = 0; // including EOL character.
            while(first_eol_section_ptr < rx_ptr)
            {
                uint8_t c = *(rx_buffer.data() + first_eol_section_ptr);
                if(c == '\r' || c == '\n' || c == '!') break;
                first_eol_section_ptr++;
            }
            if(first_eol_section_ptr >= rx_ptr) break; // searched to end
            uint16_t valid_line_len = first_eol_section_ptr + 1;
            // no empty payload will be processed.
            // this also helps when meet consistent EOLs (like: foo 0.1!!!!!!! will only be processed as foo 0.1)
            if(valid_line_len > 1) ProcessEachValidLine(rx_buffer.data(), first_eol_section_ptr);
            // Step #3: refers to Note 1
            uint16_t size_remaining = rx_ptr - valid_line_len;
            memmove(rx_buffer.data(), rx_buffer.data() + valid_line_len, size_remaining);
            rx_ptr = size_remaining;
        }
        data += actual_len;
        len -= actual_len;
    } while(len > 0);

    // // Test print of the buffer
    // uint16_t temp = rx_ptr;
    // uint8_t* ptr = rx_buffer.data();
    // do {
    //     auto write = uart->WriteBytes(ptr, temp);
    //     if(write < temp) uart->StartTransmit(false);
    //     ptr += write;
    //     temp -= write;
    // } while(temp > 0);
    // uart->StartTransmit(false);
    return false;
}

template<typename... TArgs>
void ASCIIProtocolFOC::GenerateResponse(bool use_checksum, bool continuous, const char *fmt, TArgs &&... args)
{
    char tx_buf[128];
    size_t len = snprintf_(tx_buf, sizeof(tx_buf), fmt, std::forward<TArgs>(args)...);
    if(len > uart->GetTxAvailable()) uart->StartTransmit(false); // initiate a transmission
    uart->WriteBytes((const uint8_t*)tx_buf, len);
    if(use_checksum)
    {
        uint8_t checksum = 0;
        for(size_t i = 0; i < len; ++i) checksum ^= tx_buf[i];
        uart->Print(!continuous, "*%d\r\n", checksum);
    }
    else uart->Print(!continuous, "\r\n");
}

void ASCIIProtocolFOC::PrintReflectedVariable(const char* name, uint8_t *ptr, Reflection::ProtoFieldType type, bool use_checksum,
                                              bool verbose)
{
    switch(type)
    {
        // case Reflection::ProtoFieldType::DOUBLE:
        // {
        //     if(verbose) GenerateResponse(use_checksum, true, "%s(double): %.5f", name, *(double*)ptr);
        //     else GenerateResponse(use_checksum, true, "%.5f", *(double*)ptr);
        //     break;
        // }
        case Reflection::ProtoFieldType::FLOAT:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(float): %.6f", name, *(float*)ptr);
            else GenerateResponse(use_checksum, true, "%.6f", *(float*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::INT32:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(int32): %d", name, *(int32_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(int32_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::INT64:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(int64): %d", name, *(int64_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(int64_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::UINT32:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(uint32): %d", name, *(uint32_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(uint32_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::UINT64:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(uint64): %d", name, *(uint64_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(uint64_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::BOOL:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(bool): %d", name, *(uint8_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(uint8_t*)ptr);
            break;
        }
        default:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(unknown)", name);
            break;
        }
    }
}

bool ASCIIProtocolFOC::CheckAndGetID(uint8_t *data, uint16_t len, bool use_checksum, uint8_t expected_len, uint8_t &id)
{
    id = 0;
    if(len < expected_len)
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return false;
    }
    uint8_t dst = *(data + 2);
    if(!isdigit(dst) || (isdigit(dst) && dst - '0' > SYSTEM_MOTOR_NUM - 1) || *(data + 3) != ' ')
    {
        if(foc->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return false;
    }
    id = dst;
    return true;
}

void splitData_f(char* pSource, const uint16_t Lsource, float* pDest, uint8_t* Ldest, const uint8_t max_dest_len, const uint8_t split)
{
    uint8_t temp = 0, point = 0, negative = 0;
    uint16_t numCount = 0;
    *(Ldest) = 0;
    // for(i = 0; i < maxLen; i++) *(pDest+i) = 0; // flush the dest buffer, maxLen represents maximum length of pDest (prevent overflow)
    for(uint16_t i = 0; i < Lsource && *(Ldest) <= max_dest_len; i++)
    {
        float& dest = *(pDest + *(Ldest));
        temp = *(pSource + i);
        if(temp != split)
        {
            if(numCount == 0 && dest != 0) dest = 0.0f; // flush the used pDest buffer
            if(temp == '-')
            {
                if(dest != 0) dest = 0.0f; // error: '-' after numbers, ignoring forward nums
                negative = 1;
            }
            else if(temp == '.')
            { // point
                if(point) point = 0; // error: multiple points in one continous number
                else point = 1;
            }
            else if(temp >= '0' && temp <= '9')
            {
                if(point)
                {
                    // *(pDest + *(Ldest)) += (float)((temp - '0') / (std::powf(10, point++)));
                    dest += (float)((temp - '0') / quick_powf<10>(point++));
                }
                else
                {
                    dest *= 10.0f;
                    dest += (float)(temp - '0');
                }
                numCount++;
            }
        }
        if(temp == split || i == Lsource - 1)
        {
            if(dest != 0 || (dest == 0 && numCount > 0))
            { // (pDest[Ldest] != 0 || (pDest[Ldest] == 0 && numCount >0) && temp == split, check negative and move Ldest pointer back
                if(negative) dest = (float)(-1.0f * dest);
                *(Ldest) += 1;
            }
            negative = point = numCount = 0;
        }
    }
}

size_t splitStr(const std::string_view& source, Vector<String>& dest, const char split)
{
    dest.clear();
    size_t start = 0, end = 0;
    while((end = source.find(split, start)) != std::string::npos)
    {
        dest.emplace_back(source.substr(start, end - start));
        start = end + 1;
    }
    dest.emplace_back(source.substr(start));
    return dest.size();
}

}
