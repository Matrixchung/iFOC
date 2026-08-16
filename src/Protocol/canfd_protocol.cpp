#include "canfd_protocol.hpp"

#include "foc_motor.hpp"

#define HOST_ID (0)
#define BROADCAST_ID (15)

#define HEARTBEAT_BROADCAST_MIN_INTERVAL_MS (500)
#define HEARTBEAT_BROADCAST_MAX_INTERVAL_MS (2000)
#define HEARTBEAT_BROADCAST_DEFAULT_INTERVAL_MS (1000)

#define MISC_FB_BROADCAST_DEFAULT_INTERVAL_MS (20)
#define MISC_FB_BROADCAST_MAX_INTERVAL_MS (500)

#define CLASS_REALTIME (0x00)
#define CLASS_HEARTBEAT (0x01)
#define CLASS_ASYNC_MSG (0x02)
#define CLASS_BACKGROUND (0x03)

uint8_t FIDToClass(const uint8_t frame_id)
{
    switch(frame_id)
    {
        case 3: // Execute Opcode
        case 5:
        case 6:
        case 7:
        case 8: // Velocity/Torque Control
        case 9:
        case 13: // Motion Sync
        case 15: // RT Feedback
            return CLASS_REALTIME;

        case 1: // Node Status Upload
        case 14: // Misc Feedback
            return CLASS_HEARTBEAT;

        case 0: // Node ID Allocation (DNA)
        case 2: // Node Info Acquirement
        case 4: // Param Get/Set
        case 12: // Serial Oscilloscope
            return CLASS_ASYNC_MSG;

        case 10: // File Download
        case 11: // File Upload
        default: return CLASS_BACKGROUND;
    }
}

namespace iFOC::Protocol
{
    CANFDProtocol::CANFDProtocol(HAL::CANFDBase* base) : can(base), periodic_task(this)
    {
    }

    CANFDProtocol::~CANFDProtocol()
    {
        const auto motor = GetMotor<FOCMotor>();
        motor->RemoveTaskByName("CANFD");
    }

    void CANFDProtocol::Init()
    {
        const auto motor = GetMotor<FOCMotor>();
        SetNodeID(motor->GetConfig().node_id());
        BoardConfig().GetConfig().GetReflectMap(); // generate reflect map first, to avoid generate in interrupt
        motor->GetConfig().GetReflectMap();
        PrepareNodeInfo();
        motor->AppendTask(&periodic_task);
        can->RegisterRxHandler(std::bind(&CANFDProtocol::OnRxEvent, this, std::placeholders::_1));
    }

    CANFDProtocol::PeriodicTask::PeriodicTask(CANFDProtocol* p) : Task("CANFD"), parent(p)
    {
        RegisterTask(TaskType::NORMAL_TASK, TaskType::MID_TASK);
        config.rtos_priority = configMAX_PRIORITIES - 3;
        config.stack_depth = 1024;
    }

    void CANFDProtocol::PeriodicTask::InitNormal()
    {
        const auto current_tick = xTaskGetTickCount();
        xLastWakeTick = current_tick;
        param_session_last_update_tick = current_tick;
        observed_param_session_activity = parent->param_session.activity_seq;
        if(parent->node_id == BROADCAST_ID)
        {
            ifoc_srand(current_tick);
            next_send_tick.heartbeat = current_tick + pdMS_TO_TICKS(HEARTBEAT_BROADCAST_MIN_INTERVAL_MS) +
                pdMS_TO_TICKS(ifoc_rand() % HEARTBEAT_BROADCAST_DEFAULT_INTERVAL_MS);
        }
        else next_send_tick.heartbeat = 0; // will immediately send heartbeat during first UpdateNormal()
#if (configGENERATE_RUN_TIME_STATS == 1) && (INCLUDE_xTaskGetIdleTaskHandle == 1)
        next_sample_cpu_usage = xTaskGetTickCount();
        last_runtime_counter = portGET_RUN_TIME_COUNTER_VALUE();
        last_idle_runtime = ulTaskGetIdleRunTimeCounter();
#endif
    }

    void CANFDProtocol::PeriodicTask::UpdateNormal()
    {
        const auto motor = GetMotor<FOCMotor>();
        const bool anonymous = parent->node_id == BROADCAST_ID;
        const TickType_t current_tick = xTaskGetTickCount();
        // param
        const uint32_t activity = parent->param_session.activity_seq;
        if(parent->param_session.state == 0)
        {
            observed_param_session_activity = activity;
            param_session_last_update_tick = current_tick;
        }
        else if(activity != observed_param_session_activity)
        {
            observed_param_session_activity = activity;
            param_session_last_update_tick = current_tick;
        }
        else if(current_tick - param_session_last_update_tick >= pdMS_TO_TICKS(500))
        {
            if(parent->param_session.state != 0 &&
               parent->param_session.activity_seq == activity)
            {
                parent->ResetParamSession();
                observed_param_session_activity = activity;
                param_session_last_update_tick = current_tick;
            }
        }
        // heartbeat
        if(xTaskGetTickCount() >= next_send_tick.heartbeat)
        {
            auto interval_ms = motor->GetConfig().can_heartbeat_interval_ms();
            if(interval_ms < HEARTBEAT_BROADCAST_MIN_INTERVAL_MS ||
                interval_ms > HEARTBEAT_BROADCAST_MAX_INTERVAL_MS)
            {
                interval_ms = HEARTBEAT_BROADCAST_DEFAULT_INTERVAL_MS;
                motor->GetConfig().set_can_heartbeat_interval_ms(interval_ms);
            }
            // send heartbeat
            parent->SendNodeStatus();
            const auto current_tick = xTaskGetTickCount();
            if(anonymous)
            {
                ifoc_srand(current_tick);
                next_send_tick.heartbeat = current_tick + pdMS_TO_TICKS(HEARTBEAT_BROADCAST_MIN_INTERVAL_MS) +
                    pdMS_TO_TICKS(ifoc_rand() % HEARTBEAT_BROADCAST_DEFAULT_INTERVAL_MS);
            }
            else next_send_tick.heartbeat = current_tick + pdMS_TO_TICKS(interval_ms);
        }
        if(!anonymous && xTaskGetTickCount() >= next_send_tick.misc_feedback)
        {
            auto interval_ms = motor->GetConfig().can_misc_fdbk_interval_ms();
            if(interval_ms < MISC_FB_BROADCAST_DEFAULT_INTERVAL_MS ||
                interval_ms > MISC_FB_BROADCAST_MAX_INTERVAL_MS)
            {
                interval_ms = HEARTBEAT_BROADCAST_DEFAULT_INTERVAL_MS;
                motor->GetConfig().set_can_misc_fdbk_interval_ms(interval_ms);
            }
            parent->SendMiscFeedback();
            next_send_tick.misc_feedback = xTaskGetTickCount() + pdMS_TO_TICKS(interval_ms);
        }
        // CPU usage update
#if (configGENERATE_RUN_TIME_STATS == 1) && (INCLUDE_xTaskGetIdleTaskHandle == 1)
        // FreeRTOS CPU usage: https://jishuzhan.net/article/2058043231099260930
        if(xTaskGetTickCount() >= next_sample_cpu_usage)
        {
            const uint32_t curr_runtime_counter = portGET_RUN_TIME_COUNTER_VALUE();
            const uint32_t curr_idle_runtime = ulTaskGetIdleRunTimeCounter();
            const uint32_t delta_total = curr_runtime_counter - last_runtime_counter;
            const uint32_t delta_idle = curr_idle_runtime - last_idle_runtime;
            const float cpu_usage = 100.0f - ((float)delta_idle / delta_total) * 100.0f;
            parent->cpu_usage_pct = (uint8_t)_constrain(cpu_usage, 0.0f, 100.0f);
            last_runtime_counter = curr_runtime_counter;
            last_idle_runtime = curr_idle_runtime;
            next_sample_cpu_usage = xTaskGetTickCount() + pdMS_TO_TICKS(100); // 100ms
        }
#endif
        vTaskDelayUntil(&xLastWakeTick, pdMS_TO_TICKS(10));
    }

    void CANFDProtocol::PeriodicTask::UpdateMid(const float Ts)
    {
        const auto motor = GetMotor<FOCMotor>();
        parent->task_time_avg.mid_task_avg.GetOutput(motor->task_times.mid_interval_task.elapsed_time_us);
        parent->task_time_avg.rt_task_avg.GetOutput(motor->task_times.rt_main_task.elapsed_time_us);
    }

    void CANFDProtocol::HandleNodeAllocation(const uint8_t* payload, uint8_t len)
    {
        if(len != 5) return;
        const uint32_t target_uuid = payload[0] | ((uint32_t)payload[1] << 8) | ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
        if(target_uuid != HAL::GetSerialNumber()) return;
        const uint8_t target_node_id = payload[4];
        if(target_node_id > BROADCAST_ID) return;
        SetNodeID(target_node_id);
    }

    void CANFDProtocol::HandleGetNodeInfo(const uint8_t* payload, uint8_t len)
    {
        if(len != 0) return;
        TransmitPacket(2, node_info_packet.data_len, node_info_packet.data);
    }

    void CANFDProtocol::HandleExecuteOpcode(const uint8_t* payload, uint8_t len)
    {
        if(len < 2) return;
        const uint8_t session_id = payload[0];
        const uint8_t opcode = payload[1];
        const uint8_t argument_len = len - 2;
        if(opcode == 0)
        {
            opcode_session.session_id = session_id;
            opcode_session.opcode = 0;
            opcode_session.status = 0; // NOT_SUPPORTED
            opcode_session.argument_len = 0;
            opcode_session.argument_echo_len = 0;
            SendOpcodeSession();
            return;
        }
        if(session_id == opcode_session.session_id)
        {
            if(opcode_session.opcode != 0)
            {
                if(opcode_session.opcode == opcode && opcode_session.argument_len == argument_len)
                {
                    if(memcmp(payload + 2, &opcode_session.argument[0], argument_len) == 0)
                    {
                        // Send retransmission echo
                        SendOpcodeSession();
                        return;
                    }
                }
                // error
                opcode_session.opcode = opcode;
                opcode_session.argument_echo_len = 0;
                opcode_session.status = 1; // failed
                SendOpcodeSession();
                return;
            }
        }
        // new opcode
        const auto motor = GetMotor<FOCMotor>();
        motor->UpdateWatchdog();
        opcode_session.session_id = session_id;
        opcode_session.opcode = opcode;
        opcode_session.status = 1; // failed first
        opcode_session.argument_len = argument_len;
        opcode_session.argument_echo_len = 0;
        memcpy(opcode_session.argument, payload + 2, argument_len);
        switch(opcode)
        {
            case 1: // SAVE_PARAM
            {
                if(argument_len != 1) break;
                opcode_session.argument_echo_len = 1;
                opcode_session.argument_echo[0] = 0;
                if(opcode_session.argument[0] == 0)
                {
                    if(BoardConfig().SaveNVMConfig() == FuncRetCode::OK)
                    {
                        opcode_session.status = 2; // success
                        opcode_session.argument_echo[0] = 0;
                    }
                }
                else if(opcode_session.argument[0] == motor->GetInternalID() + 1)
                {
                    if(motor->config.SaveNVMConfig() == FuncRetCode::OK)
                    {
                        opcode_session.status = 2; // success
                        opcode_session.argument_echo[0] = motor->GetInternalID() + 1;
                    }
                }
                break;
            }
            case 2: // ERASE_PARAM
            {
                if(argument_len != 1) break;
                opcode_session.argument_echo_len = 1;
                opcode_session.argument_echo[0] = 0;
                if(opcode_session.argument[0] == 0)
                {
                    if(BoardConfig().ClearNVMConfig() == FuncRetCode::OK)
                    {
                        opcode_session.status = 2; // success
                        opcode_session.argument_echo[0] = 0;
                    }
                }
                else if(opcode_session.argument[0] == motor->GetInternalID() + 1)
                {
                    if(motor->config.ClearNVMConfig() == FuncRetCode::OK)
                    {
                        opcode_session.status = 2; // success
                        opcode_session.argument_echo[0] = motor->GetInternalID() + 1;
                    }
                    motor->GetConfig().set_node_id(node_id);
                }
                break;
            }
            case 3: // REBOOT
            {
                if(argument_len != 5) break;
                if(opcode_session.argument[0] != 0xAA ||
                    opcode_session.argument[1] != 0xBB ||
                    opcode_session.argument[2] != 0xCC ||
                    opcode_session.argument[3] != 0xDD) break;
                if(opcode_session.argument[4] == 0)
                {
                    opcode_session.status = 2;
                    HAL::SystemReboot();
                }
                else if(opcode_session.argument[4] == 1)
                {
                    if(HAL::Bootloader::HasBL())
                    {
                        opcode_session.status = 2;
                        BootloaderMsg msg{};
                        msg.server_node_id = node_id; // reuse server_node_id as node_id, to speed up discovery process
                        HAL::Bootloader::JumpToBL(msg);
                    }
                }
                break;
            }
            case 4: // TOGGLE_BEEP_IDENTIFY
            {
                if(argument_len > 0) break;
                if(motor->GetCurrentState() != MotorState::IDLE) break;
                opcode_session.status = 2;
                opcode_session.argument_echo_len = 1;
                opcode_session.argument_echo[0] = motor->ToggleBeepIdentify() == FuncRetCode::OK ? 1 : 0;
                break;
            }
            case 5: // GET_ERROR
            {
                if(argument_len > 0) break;
                opcode_session.status = 2;
                opcode_session.argument_echo_len = 9;
                const uint64_t error = motor->GetError();
                memcpy(opcode_session.argument_echo, &error, 8);
                opcode_session.argument_echo[8] = 0;
                break;
            }
            case 6: // CLEAR_ERROR_MASK
            {
                if(argument_len != 10) break;
                if(opcode_session.argument[8] || opcode_session.argument[9]) break;
                uint64_t mask = 0;
                memcpy(&mask, opcode_session.argument, 8);
                motor->ClearError(mask);
                const uint64_t error = motor->GetError();
                if((error & mask) == 0UL) opcode_session.status = 2;
                opcode_session.argument_echo_len = 9;
                memcpy(opcode_session.argument_echo, &error, 8);
                if(argument_len > 0) break;
                break;
            }
            case 7: // SET_MOTOR_STATE
            {
                if(argument_len != 1) break;
                if(opcode_session.argument[0] > (uint32_t)MotorState::OPEN_LOOP_VELOCITY_CONTROL) break;
                const MotorState req_state = (MotorState)opcode_session.argument[0];
                const uint8_t ret_state = (uint8_t)motor->state_machine.RequestState(req_state);
                if(ret_state == opcode_session.argument[0]) opcode_session.status = 2;
                opcode_session.argument_echo_len = 1;
                opcode_session.argument_echo[0] = ret_state;
                break;
            }
            case 8: // GET_NVM_FILE_INFO_BY_KEY
            {
                if(argument_len != 10) break;
                char key[11]{};
                bool terminated = false;
                bool valid = opcode_session.argument[0] != '\0';
                for(uint8_t i = 0; i < 10 && valid; i++)
                {
                    const uint8_t ch = opcode_session.argument[i];
                    if(terminated)
                    {
                        valid = ch == '\0';
                    }
                    else if(ch == '\0')
                    {
                        terminated = true;
                    }
                    else
                    {
                        valid = ch >= 0x20 && ch <= 0x7E;
                        key[i] = (char)ch;
                    }
                }
                if(!valid) break;

                const uint32_t size = BlobNVMStorage().GetKVSize(key);
                if(size == 0) break;
                opcode_session.status = 2;
                opcode_session.argument_echo_len = 17;
                // memset(opcode_session.argument_echo, 0, opcode_session.argument_echo_len);
                memcpy(opcode_session.argument_echo, opcode_session.argument, 10);
                opcode_session.argument_echo[10] = (uint8_t)(size & 0xFF);
                opcode_session.argument_echo[11] = (uint8_t)((size >> 8) & 0xFF);
                opcode_session.argument_echo[12] = (uint8_t)((size >> 16) & 0xFF);
                opcode_session.argument_echo[13] = (uint8_t)((size >> 24) & 0xFF);
                break;
            }
            case 9: // DELETE_NVM_FILE_BY_KEY
            {
                if(argument_len != 10) break;
                if(motor->GetCurrentState() != MotorState::IDLE) break;
                char key[11]{};
                bool terminated = false;
                bool valid = opcode_session.argument[0] != '\0';
                for(uint8_t i = 0; i < 10 && valid; i++)
                {
                    const uint8_t ch = opcode_session.argument[i];
                    if(terminated)
                    {
                        valid = ch == '\0';
                    }
                    else if(ch == '\0')
                    {
                        terminated = true;
                    }
                    else
                    {
                        valid = ch >= 0x20 && ch <= 0x7E;
                        key[i] = (char)ch;
                    }
                }
                if(!valid) break;

                if(BlobNVMStorage().ClearNVM(key) == FuncRetCode::OK)
                {
                    opcode_session.status = 2;
                }
                break;
            }
            case 10: // ERASE_NVM_USER_AREA
            {
                if(argument_len != 4) break;
                if(motor->GetCurrentState() != MotorState::IDLE) break;
                if(opcode_session.argument[0] != 0x11 ||
                    opcode_session.argument[1] != 0x22 ||
                    opcode_session.argument[2] != 0x33 ||
                    opcode_session.argument[3] != 0x44) break;
                if(BlobNVMStorage().ClearAllNVM() == FuncRetCode::OK)
                {
                    opcode_session.status = 2;
                }
                break;
            }
            default:
            {
                opcode_session.status = 0; // NOT_SUPPORTED
                break;
            }
        }
        SendOpcodeSession();
    }

    void CANFDProtocol::HandleParamGetSet(const uint8_t* payload, uint8_t len)
    {
        constexpr uint8_t COMMAND_INLINE_REQ = 0x00;
        constexpr uint8_t COMMAND_BEGIN_REQ = 0x01;
        constexpr uint8_t COMMAND_WRITE_REQ_CHUNK = 0x02;
        constexpr uint8_t COMMAND_EXEC_REQ = 0x03;
        constexpr uint8_t COMMAND_READ_RSP_CHUNK = 0x04;
        constexpr uint8_t COMMAND_ABORT = 0x05;

        constexpr uint8_t STATUS_ACK = 0x01;
        constexpr uint8_t STATUS_INLINE_RSP = 0x02;
        constexpr uint8_t STATUS_RSP_READY = 0x03;
        constexpr uint8_t STATUS_RSP_CHUNK = 0x04;
        constexpr uint8_t STATUS_DONE = 0x05;
        constexpr uint8_t STATUS_ERROR = 0x06;

        constexpr uint16_t ERROR_INVALID_COMMAND = 0x01;
        constexpr uint16_t ERROR_INVALID_SESSION = 0x02;
        constexpr uint16_t ERROR_BAD_OFFSET = 0x03;
        constexpr uint16_t ERROR_BAD_CRC = 0x04;
        constexpr uint16_t ERROR_PAYLOAD_TOO_LONG = 0x05;
        constexpr uint16_t ERROR_TYPE = 0x07;
        constexpr uint16_t ERROR_DOMAIN_NOT_FOUND = 0x0C;

        constexpr uint8_t OP_GET_COUNT = 0x00;
        constexpr uint8_t OP_GET_BY_INDEX = 0x01;
        constexpr uint8_t OP_GET_BY_NAME = 0x02;
        constexpr uint8_t OP_SET_BY_INDEX = 0x03;
        constexpr uint8_t OP_SET_BY_NAME = 0x04;
        constexpr uint8_t OP_EXEC_LOAD_DEFAULT = 0x07;

        constexpr uint8_t RESULT_OK = 0x00;
        constexpr uint8_t RESULT_NOT_FOUND = 0x01;
        constexpr uint8_t RESULT_TYPE_MISMATCH = 0x02;
        constexpr uint8_t RESULT_RANGE_REJECTED = 0x04;
        constexpr uint8_t RESULT_NAME_INDEX_MISMATCH = 0x06;
        constexpr uint8_t RESULT_NOT_ALLOWED = 0x07;

        constexpr uint8_t FIELD_CURRENT_VALUE = 1 << 0;
        constexpr uint8_t FIELD_NAME = 1 << 1;
        constexpr uint8_t FIELD_ACCESS_FLAGS = 1 << 5;
        constexpr uint8_t FIELD_PARAM_TYPE = 1 << 6;

        constexpr uint8_t ACCESS_READABLE = 1 << 0;
        constexpr uint8_t ACCESS_WRITABLE = 1 << 1;
        constexpr uint8_t ACCESS_PERSISTENT = 1 << 3;

        constexpr uint8_t TYPE_EMPTY = 0;
        constexpr uint8_t TYPE_BOOLEAN = 1;
        constexpr uint8_t TYPE_FLOAT32 = 2;
        constexpr uint8_t TYPE_UINT64 = 3;
        constexpr uint8_t TYPE_INT64 = 4;
        constexpr uint8_t TYPE_STRING = 5;
        constexpr uint8_t TYPE_BYTES = 6;

        constexpr uint8_t MAX_WRITE_CHUNK_LEN = 57;
        constexpr uint8_t MAX_READ_CHUNK_LEN = 54;
        if(len < 4) return;

        const uint8_t command = payload[0];
        const uint16_t session_id = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
        const uint8_t flags = payload[3];

        auto send_reply = [&](uint8_t status, uint16_t sid, uint16_t info,
                              const uint8_t* data = nullptr, uint16_t data_len = 0)
        {
            if(data_len > 59) return;
            buffer[0] = status;
            buffer[1] = (uint8_t)(sid & 0xFF);
            buffer[2] = (uint8_t)(sid >> 8);
            buffer[3] = (uint8_t)(info & 0xFF);
            buffer[4] = (uint8_t)(info >> 8);
            if(data && data_len) memcpy(buffer + 5, data, data_len);
            TransmitPacket(4, (uint8_t)(5 + data_len), buffer);
        };

        auto send_error = [&](uint16_t sid, uint16_t error_code)
        {
            send_reply(STATUS_ERROR, sid, error_code);
        };

        if(command > COMMAND_ABORT)
        {
            send_error(session_id, ERROR_INVALID_COMMAND);
            return;
        }
        if(session_id == 0)
        {
            send_error(session_id, ERROR_INVALID_SESSION);
            return;
        }
        if((flags & 0xFE) != 0)
        {
            send_error(session_id, ERROR_INVALID_COMMAND);
            return;
        }

        if(param_session.state == 2 && param_session.response_complete &&
           param_session.session_id != session_id &&
           (command == COMMAND_INLINE_REQ || command == COMMAND_BEGIN_REQ))
        {
            ResetParamSession();
        }
        if(param_session.state != 0 && param_session.session_id != session_id &&
           command != COMMAND_ABORT)
        {
            send_error(session_id, ERROR_INVALID_SESSION);
            return;
        }

        auto min_u16 = [](uint16_t a, uint16_t b) -> uint16_t
        {
            return a < b ? a : b;
        };

        auto append_bytes = [](uint8_t* dst, uint16_t& dst_len, uint16_t capacity,
                               const void* src, uint16_t src_len) -> bool
        {
            if(dst_len > capacity || src_len > capacity - dst_len) return false;
            if(src && src_len) memcpy(dst + dst_len, src, src_len);
            dst_len += src_len;
            return true;
        };

        auto append_u8 = [&](uint8_t* dst, uint16_t& dst_len, uint16_t capacity,
                             uint8_t value) -> bool
        {
            return append_bytes(dst, dst_len, capacity, &value, 1);
        };

        auto typed_value_valid = [&](uint8_t type, uint8_t value_len) -> bool
        {
            switch(type)
            {
                case TYPE_EMPTY: return value_len == 0;
                case TYPE_BOOLEAN: return value_len == 1;
                case TYPE_FLOAT32: return value_len == 4;
                case TYPE_UINT64:
                case TYPE_INT64: return value_len == 8;
                case TYPE_STRING:
                case TYPE_BYTES: return value_len <= 31;
                default: return false;
            }
        };

        auto get_request_logical_len = [&](const uint8_t* request, uint16_t available_len,
                                           bool allow_dlc_padding, uint16_t& logical_len,
                                           uint16_t& transport_error) -> bool
        {
            transport_error = 0;
            logical_len = 0;
            if(available_len < 5)
            {
                transport_error = ERROR_PAYLOAD_TOO_LONG;
                return false;
            }

            const uint8_t op = request[0];
            const uint8_t value_type_len = request[4];
            const uint8_t value_type = value_type_len & 0x07;
            const uint8_t value_len = value_type_len >> 3;
            if(!typed_value_valid(value_type, value_len))
            {
                transport_error = ERROR_TYPE;
                return false;
            }

            logical_len = (uint16_t)(5 + value_len);
            if(logical_len > available_len)
            {
                transport_error = ERROR_PAYLOAD_TOO_LONG;
                return false;
            }
            if(op == OP_GET_BY_NAME || op == OP_SET_BY_NAME)
            {
                if(logical_len >= available_len)
                {
                    transport_error = ERROR_PAYLOAD_TOO_LONG;
                    return false;
                }
                const uint8_t name_len = request[logical_len++];
                if(name_len > 31 || name_len > available_len - logical_len)
                {
                    transport_error = ERROR_PAYLOAD_TOO_LONG;
                    return false;
                }
                logical_len += name_len;
            }
            if(!allow_dlc_padding && logical_len != available_len)
            {
                transport_error = ERROR_PAYLOAD_TOO_LONG;
                return false;
            }
            if(allow_dlc_padding)
            {
                for(uint16_t i = logical_len; i < available_len; i++)
                {
                    if(request[i] != 0)
                    {
                        transport_error = ERROR_PAYLOAD_TOO_LONG;
                        return false;
                    }
                }
            }
            return true;
        };

        auto param_type_from_reflect = [](Reflection::ProtoFieldType type) -> uint8_t
        {
            switch(type)
            {
                case Reflection::ProtoFieldType::DOUBLE:
                case Reflection::ProtoFieldType::FLOAT: return TYPE_FLOAT32;
                case Reflection::ProtoFieldType::INT32:
                case Reflection::ProtoFieldType::INT64: return TYPE_INT64;
                case Reflection::ProtoFieldType::UINT32:
                case Reflection::ProtoFieldType::UINT64: return TYPE_UINT64;
                case Reflection::ProtoFieldType::BOOL: return TYPE_BOOLEAN;
                case Reflection::ProtoFieldType::STRING: return TYPE_STRING;
                default: return TYPE_EMPTY;
            }
        };

        struct ParamTarget
        {
            MemberInfo info{};
            uint8_t* ptr = nullptr;
            uint8_t index = 0;
            char name[32]{};
            uint8_t name_len = 0;
            uint8_t value_capacity = 0;
        };

        auto copy_param_name = [](ParamTarget& target, const char* name)
        {
            auto name_len = strnlen(name, sizeof(target.name) - 1);
            if(name_len > 0 && name[name_len - 1] == '_') name_len--;
            memcpy(target.name, name, name_len);
            target.name[name_len] = '\0';
            target.name_len = (uint8_t)name_len;
        };

        auto get_domain = [&](uint8_t domain_id, const ReflectMap*& reflect,
                              uint8_t*& start_ptr) -> bool
        {
            const auto motor = GetMotor<FOCMotor>();
            switch(domain_id)
            {
                case 0:
                    reflect = &BoardConfig().GetConfig().GetReflectMap();
                    start_ptr = (uint8_t*)&BoardConfig().GetConfig();
                    return true;
                case 1:
                    reflect = &motor->GetConfig().GetReflectMap();
                    start_ptr = (uint8_t*)&motor->GetConfig();
                    return true;
                default:
                    reflect = nullptr;
                    start_ptr = nullptr;
                    return false;
            }
        };

        auto get_domain_count = [&](uint8_t domain_id, uint16_t& count) -> bool
        {
            const ReflectMap* reflect = nullptr;
            uint8_t* start_ptr = nullptr;
            if(!get_domain(domain_id, reflect, start_ptr)) return false;
            count = (uint16_t)reflect->size();
            if(domain_id == 1) count += 1;
            return true;
        };

        auto find_param_by_index = [&](uint8_t domain_id, uint8_t index,
                                       ParamTarget& target) -> bool
        {
            const auto motor = GetMotor<FOCMotor>();
            const ReflectMap* reflect = nullptr;
            uint8_t* start_ptr = nullptr;
            if(!get_domain(domain_id, reflect, start_ptr)) return false;

            if(domain_id == 1 && index == 0)
            {
                target.info.first = Reflection::ProtoFieldType::STRING;
                target.info.second = 0;
                target.ptr = (uint8_t*)motor->GetConfig()._d.node_name;
                target.index = 0;
                target.value_capacity = sizeof(motor->GetConfig()._d.node_name);
                copy_param_name(target, "node_name");
                return true;
            }

            const uint16_t map_index = domain_id == 1 ? (uint16_t)index - 1 : index;
            uint16_t iter_index = 0;
            for(const auto& [name, info] : *reflect)
            {
                if(iter_index == map_index)
                {
                    target.info = info;
                    target.ptr = start_ptr + info.second;
                    target.index = index;
                    target.value_capacity = (uint8_t)Reflection::GetFieldSize(info.first);
                    copy_param_name(target, name);
                    return true;
                }
                iter_index++;
            }
            return false;
        };

        auto find_param_by_name = [&](uint8_t domain_id, uint8_t index_hint,
                                      const uint8_t* name, uint8_t name_len,
                                      bool index_hint_valid, ParamTarget& target,
                                      uint8_t& result_code) -> bool
        {
            result_code = RESULT_NOT_FOUND;
            if(name_len == 0 || name_len > 31) return false;
            for(uint8_t i = 0; i < name_len; i++)
            {
                if(name[i] < 0x20 || name[i] > 0x7E) return false;
            }

            char name_buffer[33]{};
            memcpy(name_buffer, name, name_len);
            uint8_t local_len = name_len;

            if(domain_id == 1 && strcmp(name_buffer, "node_name") == 0)
            {
                if(!find_param_by_index(domain_id, 0, target)) return false;
                if(index_hint_valid && index_hint != target.index)
                {
                    result_code = RESULT_NAME_INDEX_MISMATCH;
                    return false;
                }
                result_code = RESULT_OK;
                return true;
            }

            const ReflectMap* reflect = nullptr;
            uint8_t* start_ptr = nullptr;
            if(!get_domain(domain_id, reflect, start_ptr)) return false;
            auto it = reflect->find(name_buffer);
            if(it == reflect->end() && local_len < sizeof(name_buffer) - 1 &&
               name_buffer[local_len - 1] != '_')
            {
                name_buffer[local_len++] = '_';
                name_buffer[local_len] = '\0';
                it = reflect->find(name_buffer);
            }
            if(it == reflect->end()) return false;

            uint16_t actual_index = domain_id == 1 ? 1 : 0;
            for(const auto& [map_name, info] : *reflect)
            {
                (void)info;
                if(strcmp(map_name, it->first) == 0) break;
                actual_index++;
            }
            if(actual_index > 255) return false;

            target.info = it->second;
            target.ptr = start_ptr + it->second.second;
            target.index = (uint8_t)actual_index;
            target.value_capacity = (uint8_t)Reflection::GetFieldSize(it->second.first);
            copy_param_name(target, it->first);
            if(index_hint_valid && index_hint != target.index)
            {
                result_code = RESULT_NAME_INDEX_MISMATCH;
                return false;
            }
            result_code = RESULT_OK;
            return true;
        };

        auto append_empty_value = [&](uint8_t* dst, uint16_t& dst_len,
                                      uint16_t capacity) -> bool
        {
            return append_u8(dst, dst_len, capacity, TYPE_EMPTY);
        };

        auto append_current_value = [&](uint8_t* dst, uint16_t& dst_len,
                                        uint16_t capacity,
                                        const ParamTarget& target) -> bool
        {
            switch(target.info.first)
            {
                case Reflection::ProtoFieldType::DOUBLE:
                {
                    double value = 0.0;
                    memcpy(&value, target.ptr, sizeof(value));
                    const float out = (float)value;
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((sizeof(out) << 3) | TYPE_FLOAT32)) &&
                           append_bytes(dst, dst_len, capacity, &out, sizeof(out));
                }
                case Reflection::ProtoFieldType::FLOAT:
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((sizeof(float) << 3) | TYPE_FLOAT32)) &&
                           append_bytes(dst, dst_len, capacity, target.ptr, sizeof(float));
                case Reflection::ProtoFieldType::INT32:
                {
                    int32_t value = 0;
                    memcpy(&value, target.ptr, sizeof(value));
                    const int64_t out = value;
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((sizeof(out) << 3) | TYPE_INT64)) &&
                           append_bytes(dst, dst_len, capacity, &out, sizeof(out));
                }
                case Reflection::ProtoFieldType::INT64:
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((sizeof(int64_t) << 3) | TYPE_INT64)) &&
                           append_bytes(dst, dst_len, capacity, target.ptr, sizeof(int64_t));
                case Reflection::ProtoFieldType::UINT32:
                {
                    uint32_t value = 0;
                    memcpy(&value, target.ptr, sizeof(value));
                    const uint64_t out = value;
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((sizeof(out) << 3) | TYPE_UINT64)) &&
                           append_bytes(dst, dst_len, capacity, &out, sizeof(out));
                }
                case Reflection::ProtoFieldType::UINT64:
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((sizeof(uint64_t) << 3) | TYPE_UINT64)) &&
                           append_bytes(dst, dst_len, capacity, target.ptr, sizeof(uint64_t));
                case Reflection::ProtoFieldType::BOOL:
                {
                    const uint8_t value = *target.ptr ? 1 : 0;
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((1 << 3) | TYPE_BOOLEAN)) &&
                           append_u8(dst, dst_len, capacity, value);
                }
                case Reflection::ProtoFieldType::STRING:
                {
                    const uint8_t string_len = (uint8_t)min_u16(
                            (uint16_t)strnlen((char*)target.ptr, target.value_capacity), 31);
                    return append_u8(dst, dst_len, capacity,
                                     (uint8_t)((string_len << 3) | TYPE_STRING)) &&
                           append_bytes(dst, dst_len, capacity, target.ptr, string_len);
                }
                default: return append_empty_value(dst, dst_len, capacity);
            }
        };

        auto write_value = [&](const ParamTarget& target, uint8_t value_type,
                               uint8_t value_len, const uint8_t* value_ptr) -> uint8_t
        {
            switch(target.info.first)
            {
                case Reflection::ProtoFieldType::DOUBLE:
                {
                    if(value_type != TYPE_FLOAT32 || value_len != sizeof(float))
                        return RESULT_TYPE_MISMATCH;
                    float in = 0.0f;
                    memcpy(&in, value_ptr, sizeof(in));
                    const double out = in;
                    memcpy(target.ptr, &out, sizeof(out));
                    return RESULT_OK;
                }
                case Reflection::ProtoFieldType::FLOAT:
                    if(value_type != TYPE_FLOAT32 || value_len != sizeof(float))
                        return RESULT_TYPE_MISMATCH;
                    memcpy(target.ptr, value_ptr, sizeof(float));
                    return RESULT_OK;
                case Reflection::ProtoFieldType::INT32:
                {
                    if(value_type != TYPE_INT64 || value_len != sizeof(int64_t))
                        return RESULT_TYPE_MISMATCH;
                    int64_t in = 0;
                    memcpy(&in, value_ptr, sizeof(in));
                    if(in < (-2147483647LL - 1) || in > 2147483647LL)
                        return RESULT_RANGE_REJECTED;
                    const int32_t out = (int32_t)in;
                    memcpy(target.ptr, &out, sizeof(out));
                    return RESULT_OK;
                }
                case Reflection::ProtoFieldType::INT64:
                    if(value_type != TYPE_INT64 || value_len != sizeof(int64_t))
                        return RESULT_TYPE_MISMATCH;
                    memcpy(target.ptr, value_ptr, sizeof(int64_t));
                    return RESULT_OK;
                case Reflection::ProtoFieldType::UINT32:
                {
                    if(value_type != TYPE_UINT64 || value_len != sizeof(uint64_t))
                        return RESULT_TYPE_MISMATCH;
                    uint64_t in = 0;
                    memcpy(&in, value_ptr, sizeof(in));
                    if(in > 0xFFFFFFFFULL) return RESULT_RANGE_REJECTED;
                    const uint32_t out = (uint32_t)in;
                    memcpy(target.ptr, &out, sizeof(out));
                    return RESULT_OK;
                }
                case Reflection::ProtoFieldType::UINT64:
                    if(value_type != TYPE_UINT64 || value_len != sizeof(uint64_t))
                        return RESULT_TYPE_MISMATCH;
                    memcpy(target.ptr, value_ptr, sizeof(uint64_t));
                    return RESULT_OK;
                case Reflection::ProtoFieldType::BOOL:
                {
                    if(value_type != TYPE_BOOLEAN || value_len != 1)
                        return RESULT_TYPE_MISMATCH;
                    const uint8_t out = value_ptr[0] ? 1 : 0;
                    memcpy(target.ptr, &out, sizeof(out));
                    return RESULT_OK;
                }
                case Reflection::ProtoFieldType::STRING:
                    if(value_type != TYPE_STRING || target.value_capacity == 0 ||
                       value_len >= target.value_capacity)
                        return RESULT_TYPE_MISMATCH;
                    for(uint8_t i = 0; i < value_len; i++)
                    {
                        if(value_ptr[i] < 0x20 || value_ptr[i] > 0x7E)
                            return RESULT_TYPE_MISMATCH;
                    }
                    memcpy(target.ptr, value_ptr, value_len);
                    target.ptr[value_len] = '\0';
                    return RESULT_OK;
                default: return RESULT_TYPE_MISMATCH;
            }
        };

        auto append_param_response = [&](uint8_t* response, uint16_t& response_len,
                                         uint8_t op, uint8_t result,
                                         uint8_t domain_id, uint8_t index,
                                         uint8_t requested_mask,
                                         const ParamTarget* target) -> bool
        {
            const uint16_t capacity = sizeof(param_session.response_buffer);
            const uint16_t present_pos = 4;
            uint8_t present_mask = 0;
            if(!append_u8(response, response_len, capacity, op) ||
               !append_u8(response, response_len, capacity, result) ||
               !append_u8(response, response_len, capacity, domain_id) ||
               !append_u8(response, response_len, capacity, index) ||
               !append_u8(response, response_len, capacity, 0)) return false;

            if(target && (requested_mask & FIELD_ACCESS_FLAGS))
            {
                const uint8_t access = ACCESS_READABLE | ACCESS_WRITABLE | ACCESS_PERSISTENT;
                if(!append_u8(response, response_len, capacity, access)) return false;
                present_mask |= FIELD_ACCESS_FLAGS;
            }
            if(target && (requested_mask & FIELD_PARAM_TYPE))
            {
                if(!append_u8(response, response_len, capacity,
                              param_type_from_reflect(target->info.first))) return false;
                present_mask |= FIELD_PARAM_TYPE;
            }
            if(requested_mask & FIELD_CURRENT_VALUE)
            {
                if(target)
                {
                    if(!append_current_value(response, response_len, capacity, *target))
                        return false;
                }
                else if(!append_empty_value(response, response_len, capacity)) return false;
                present_mask |= FIELD_CURRENT_VALUE;
            }
            if(target && (requested_mask & FIELD_NAME))
            {
                if(!append_u8(response, response_len, capacity, target->name_len) ||
                   !append_bytes(response, response_len, capacity,
                                 target->name, target->name_len)) return false;
                present_mask |= FIELD_NAME;
            }
            response[present_pos] = present_mask;
            return true;
        };

        auto execute_request = [&](const uint8_t* request, uint16_t available_len,
                                   bool allow_dlc_padding, uint8_t request_flags,
                                   uint8_t* response, uint16_t& response_len,
                                   uint16_t& logical_len,
                                   uint16_t& transport_error) -> bool
        {
            response_len = 0;
            if(!get_request_logical_len(request, available_len, allow_dlc_padding,
                                        logical_len, transport_error)) return false;

            const uint8_t op = request[0];
            const uint8_t domain_id = request[1];
            const uint8_t index = request[2];
            uint8_t field_mask = request[3];
            const uint8_t value_type_len = request[4];
            const uint8_t value_type = value_type_len & 0x07;
            const uint8_t value_len = value_type_len >> 3;
            const uint8_t* value_ptr = request + 5;
            uint16_t offset = 5 + value_len;
            uint8_t name_len = 0;
            const uint8_t* name_ptr = nullptr;

            if(op > OP_EXEC_LOAD_DEFAULT || op == 0x05 || op == 0x06 ||
               (field_mask & 0x80))
            {
                transport_error = ERROR_INVALID_COMMAND;
                return false;
            }
            if(op == OP_GET_BY_NAME || op == OP_SET_BY_NAME)
            {
                name_len = request[offset++];
                name_ptr = request + offset;
            }
            if(op != OP_SET_BY_INDEX && op != OP_SET_BY_NAME &&
               (value_type != TYPE_EMPTY || value_len != 0))
            {
                transport_error = ERROR_TYPE;
                return false;
            }

            uint16_t domain_count = 0;
            if(!get_domain_count(domain_id, domain_count))
            {
                transport_error = ERROR_DOMAIN_NOT_FOUND;
                return false;
            }

            if(op == OP_GET_COUNT)
            {
                const uint64_t count = domain_count;
                field_mask = FIELD_CURRENT_VALUE;
                return append_u8(response, response_len, sizeof(param_session.response_buffer), op) &&
                       append_u8(response, response_len, sizeof(param_session.response_buffer), RESULT_OK) &&
                       append_u8(response, response_len, sizeof(param_session.response_buffer), domain_id) &&
                       append_u8(response, response_len, sizeof(param_session.response_buffer), 0) &&
                       append_u8(response, response_len, sizeof(param_session.response_buffer), field_mask) &&
                       append_u8(response, response_len, sizeof(param_session.response_buffer),
                                 (uint8_t)((sizeof(count) << 3) | TYPE_UINT64)) &&
                       append_bytes(response, response_len, sizeof(param_session.response_buffer),
                                    &count, sizeof(count));
            }

            if(op == OP_EXEC_LOAD_DEFAULT)
            {
                const auto motor = GetMotor<FOCMotor>();
                uint8_t result = RESULT_OK;
                if(motor->GetCurrentState() != MotorState::IDLE)
                {
                    result = RESULT_NOT_ALLOWED;
                }
                else
                {
                    if(domain_id == 0) BoardConfig().GetConfig().clear();
                    else motor->ResetDefaultConfig();
                }
                return append_param_response(response, response_len, op, result,
                                             domain_id, index, 0, nullptr);
            }

            ParamTarget target{};
            uint8_t result = RESULT_OK;
            bool found = false;
            const bool index_hint_valid = (request_flags & 0x01) != 0;
            if(op == OP_GET_BY_INDEX || op == OP_SET_BY_INDEX)
            {
                found = find_param_by_index(domain_id, index, target);
                if(!found) result = RESULT_NOT_FOUND;
            }
            else if(op == OP_GET_BY_NAME || op == OP_SET_BY_NAME)
            {
                found = find_param_by_name(domain_id, index, name_ptr, name_len,
                                           index_hint_valid, target, result);
            }
            else
            {
                transport_error = ERROR_INVALID_COMMAND;
                return false;
            }

            if(found && (op == OP_SET_BY_INDEX || op == OP_SET_BY_NAME))
            {
                result = write_value(target, value_type, value_len, value_ptr);
                field_mask |= FIELD_CURRENT_VALUE;
            }
            if((op == OP_GET_BY_INDEX || op == OP_GET_BY_NAME) &&
               !(field_mask & (FIELD_CURRENT_VALUE | FIELD_NAME |
                               FIELD_ACCESS_FLAGS | FIELD_PARAM_TYPE)))
            {
                field_mask |= FIELD_CURRENT_VALUE;
            }

            const ParamTarget* response_target = found ? &target : nullptr;
            const uint8_t response_index = found ? target.index : index;
            if(!append_param_response(response, response_len, op, result, domain_id,
                                      response_index, field_mask, response_target))
            {
                transport_error = ERROR_PAYLOAD_TOO_LONG;
                return false;
            }
            return true;
        };

        auto send_cached_response = [&](uint16_t sid)
        {
            param_session.state = 2;
            if((uint16_t)(5 + param_session.response_len) <= 64)
            {
                send_reply(STATUS_INLINE_RSP, sid, param_session.response_len,
                           param_session.response_buffer, param_session.response_len);
                param_session.response_complete = 1;
            }
            else
            {
                send_reply(STATUS_RSP_READY, sid, param_session.response_len);
                param_session.response_complete = 0;
            }
            param_session.activity_seq++;
        };

        if(command == COMMAND_ABORT)
        {
            if(len != 4)
            {
                send_error(session_id, ERROR_INVALID_COMMAND);
                return;
            }
            if(param_session.state != 0 && param_session.session_id != session_id)
            {
                send_error(session_id, ERROR_INVALID_SESSION);
                return;
            }
            ResetParamSession();
            send_reply(STATUS_DONE, session_id, 0);
            return;
        }

        switch(command)
        {
            case COMMAND_INLINE_REQ:
            {
                uint16_t logical_len = 0;
                uint16_t transport_error = 0;
                if(param_session.state == 2 && param_session.session_id == session_id)
                {
                    if(!get_request_logical_len(payload + 4, len - 4, true,
                                                logical_len, transport_error) ||
                       flags != param_session.flags || logical_len != param_session.request_len ||
                       memcmp(payload + 4, param_session.request_buffer, logical_len) != 0)
                    {
                        send_error(session_id, ERROR_INVALID_SESSION);
                        break;
                    }
                    send_cached_response(session_id);
                    break;
                }
                if(param_session.state != 0)
                {
                    send_error(session_id, ERROR_INVALID_SESSION);
                    break;
                }

                param_session.session_id = session_id;
                param_session.flags = flags;
                if(!execute_request(payload + 4, len - 4, true, flags,
                                    param_session.response_buffer,
                                    param_session.response_len, logical_len,
                                    transport_error))
                {
                    ResetParamSession();
                    send_error(session_id, transport_error ? transport_error : ERROR_INVALID_COMMAND);
                    break;
                }
                if(logical_len > sizeof(param_session.request_buffer))
                {
                    ResetParamSession();
                    send_error(session_id, ERROR_PAYLOAD_TOO_LONG);
                    break;
                }
                memcpy(param_session.request_buffer, payload + 4, logical_len);
                param_session.request_len = logical_len;
                param_session.request_total_len = logical_len;
                param_session.request_crc16 = get_crc16(param_session.request_buffer, logical_len);
                send_cached_response(session_id);
                break;
            }
            case COMMAND_BEGIN_REQ:
            {
                if(len != 8)
                {
                    send_error(session_id, ERROR_INVALID_COMMAND);
                    break;
                }
                const uint16_t total_len = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
                const uint16_t crc16 = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
                if(total_len < 5 || total_len > sizeof(param_session.request_buffer))
                {
                    send_error(session_id, ERROR_PAYLOAD_TOO_LONG);
                    break;
                }
                if(param_session.state == 1 && param_session.session_id == session_id)
                {
                    if(param_session.request_total_len == total_len &&
                       param_session.request_crc16 == crc16 && param_session.flags == flags)
                    {
                        param_session.activity_seq++;
                        send_reply(STATUS_ACK, session_id, param_session.request_len);
                    }
                    else send_error(session_id, ERROR_INVALID_SESSION);
                    break;
                }
                if(param_session.state != 0)
                {
                    send_error(session_id, ERROR_INVALID_SESSION);
                    break;
                }
                param_session.state = 1;
                param_session.session_id = session_id;
                param_session.flags = flags;
                param_session.request_total_len = total_len;
                param_session.request_len = 0;
                param_session.request_crc16 = crc16;
                param_session.response_len = 0;
                param_session.response_complete = 0;
                param_session.activity_seq++;
                send_reply(STATUS_ACK, session_id, 0);
                break;
            }
            case COMMAND_WRITE_REQ_CHUNK:
            {
                if(param_session.state != 1 || param_session.session_id != session_id ||
                   param_session.flags != flags)
                {
                    send_error(session_id, ERROR_INVALID_SESSION);
                    break;
                }
                if(len < 7)
                {
                    send_error(session_id, ERROR_INVALID_COMMAND);
                    break;
                }
                const uint16_t offset = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
                const uint8_t chunk_len = payload[6];
                if(chunk_len > MAX_WRITE_CHUNK_LEN || (uint16_t)(7 + chunk_len) > len)
                {
                    send_error(session_id, ERROR_PAYLOAD_TOO_LONG);
                    break;
                }
                if(offset == param_session.request_len)
                {
                    if(chunk_len > param_session.request_total_len - param_session.request_len)
                    {
                        send_error(session_id, ERROR_PAYLOAD_TOO_LONG);
                        break;
                    }
                    memcpy(param_session.request_buffer + param_session.request_len,
                           payload + 7, chunk_len);
                    param_session.request_len += chunk_len;
                }
                else if(offset < param_session.request_len)
                {
                    if(chunk_len > param_session.request_len - offset ||
                       memcmp(param_session.request_buffer + offset, payload + 7, chunk_len) != 0)
                    {
                        send_error(session_id, ERROR_INVALID_SESSION);
                        break;
                    }
                }
                else
                {
                    send_error(session_id, ERROR_BAD_OFFSET);
                    break;
                }
                param_session.activity_seq++;
                send_reply(STATUS_ACK, session_id, param_session.request_len);
                break;
            }
            case COMMAND_EXEC_REQ:
            {
                if(len != 4)
                {
                    send_error(session_id, ERROR_INVALID_COMMAND);
                    break;
                }
                if(param_session.state == 2 && param_session.session_id == session_id)
                {
                    if(param_session.flags != flags)
                        send_error(session_id, ERROR_INVALID_SESSION);
                    else send_cached_response(session_id);
                    break;
                }
                if(param_session.state != 1 || param_session.session_id != session_id ||
                   param_session.flags != flags)
                {
                    send_error(session_id, ERROR_INVALID_SESSION);
                    break;
                }
                if(param_session.request_len != param_session.request_total_len)
                {
                    send_error(session_id, ERROR_BAD_OFFSET);
                    break;
                }
                if(get_crc16(param_session.request_buffer, param_session.request_len) !=
                   param_session.request_crc16)
                {
                    send_error(session_id, ERROR_BAD_CRC);
                    break;
                }

                uint16_t logical_len = 0;
                uint16_t transport_error = 0;
                if(!execute_request(param_session.request_buffer, param_session.request_len,
                                    false, flags, param_session.response_buffer,
                                    param_session.response_len, logical_len,
                                    transport_error))
                {
                    ResetParamSession();
                    send_error(session_id, transport_error ? transport_error : ERROR_INVALID_COMMAND);
                    break;
                }
                send_cached_response(session_id);
                break;
            }
            case COMMAND_READ_RSP_CHUNK:
            {
                if(param_session.state != 2 || param_session.session_id != session_id ||
                   param_session.flags != flags)
                {
                    send_error(session_id, ERROR_INVALID_SESSION);
                    break;
                }
                if(len != 7)
                {
                    send_error(session_id, ERROR_INVALID_COMMAND);
                    break;
                }
                const uint16_t offset = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
                const uint8_t max_chunk_len = payload[6];
                if(offset >= param_session.response_len || max_chunk_len == 0)
                {
                    send_error(session_id, ERROR_BAD_OFFSET);
                    break;
                }
                uint16_t chunk_len = param_session.response_len - offset;
                chunk_len = min_u16(chunk_len, max_chunk_len);
                chunk_len = min_u16(chunk_len, MAX_READ_CHUNK_LEN);

                buffer[0] = STATUS_RSP_CHUNK;
                buffer[1] = (uint8_t)(session_id & 0xFF);
                buffer[2] = (uint8_t)(session_id >> 8);
                buffer[3] = 0;
                buffer[4] = 0;
                buffer[5] = (uint8_t)(param_session.response_len & 0xFF);
                buffer[6] = (uint8_t)(param_session.response_len >> 8);
                buffer[7] = (uint8_t)(offset & 0xFF);
                buffer[8] = (uint8_t)(offset >> 8);
                buffer[9] = (uint8_t)chunk_len;
                memcpy(buffer + 10, param_session.response_buffer + offset, chunk_len);
                TransmitPacket(4, (uint8_t)(10 + chunk_len), buffer);
                if(offset + chunk_len >= param_session.response_len)
                    param_session.response_complete = 1;
                param_session.activity_seq++;
                break;
            }
            default:
                send_error(session_id, ERROR_INVALID_COMMAND);
                break;
        }
    }

    void CANFDProtocol::HandlePosControl(const uint8_t* payload, uint8_t len)
    {
        if(len != 16) return;
        // Send Feedback first
        SendRTFeedback();
        const uint16_t _seq = payload[14] | (uint16_t)payload[15] << 8;
        if(_seq == pos_command_seq) return;
        if(ParseMotionHeader((payload[0])))
        {
            control_struct.command = control_struct.POSITION;
            control_struct.is_relative = payload[1] & 0x01;
            if(control_struct.is_relative) control_struct.is_relative_curr_based = (payload[1] & 0x02) >> 1;
            control_struct.is_trajectory = (payload[1] & 0x04) >> 2;
            if(control_struct.is_trajectory) control_struct.is_trajectory_s_curve = (payload[1] & 0x08) >> 3;
            float position = 0.0f;
            memcpy(&position, &payload[2], 4);
            const float16 velocity_ff = float16::from_bits((uint16_t)payload[6] | (uint16_t)payload[7] << 8);
            const float16 velocity_limit = float16::from_bits((uint16_t)payload[8] | (uint16_t)payload[9] << 8);
            const float16 torque_ff = float16::from_bits((uint16_t)payload[10] | (uint16_t)payload[11] << 8);
            const float16 torque_limit = float16::from_bits((uint16_t)payload[12] | (uint16_t)payload[13] << 8);
            control_struct.motion.torque.value = torque_ff.to_float();
            control_struct.motion.torque.limit = torque_limit.to_float();
            control_struct.motion.speed.value = velocity_ff.to_float();
            control_struct.motion.speed.limit = velocity_limit.to_float();
            control_struct.motion.pos.value = position;
            pos_command_seq = _seq;
            ProcessControlStruct(false);
        }
    }

    void CANFDProtocol::HandleVelTorqueControl(const uint8_t* payload, uint8_t len)
    {
        if(len != 12) return;
        // Send Feedback first
        SendRTFeedback();
        if(ParseMotionHeader(payload[0]))
        {
            if(payload[1] == 0)
            {
                control_struct.command = control_struct.TORQUE;
                float torque_ff;
                memcpy(&torque_ff, &payload[6], 4);
                control_struct.motion.torque.value = torque_ff;
            }
            else if(payload[1] == 1)
            {
                control_struct.command = control_struct.VELOCITY;
                float temp;
                memcpy(&temp, &payload[2], 4);
                control_struct.motion.speed.value = temp;
                memcpy(&temp, &payload[6], 4);
                control_struct.motion.torque.value = temp;
                const float16 torque_limit = float16::from_bits((uint16_t)payload[10] | (uint16_t)payload[11] << 8);
                control_struct.motion.torque.limit = torque_limit.to_float();
            }
            else return;
            ProcessControlStruct(false);
        }
    }

    void CANFDProtocol::HandleFileDownload(const uint8_t* payload, uint8_t len)
    {
        constexpr uint8_t COMMAND_BEGIN = 0;
        constexpr uint8_t COMMAND_DATA = 1;
        constexpr uint8_t COMMAND_FIN = 2;
        constexpr uint8_t COMMAND_ABORT = 3;

        constexpr uint8_t STATUS_ACK = 1;
        constexpr uint8_t STATUS_ERROR = 2;
        constexpr uint8_t STATUS_VERIFY_OK = 3;

        constexpr uint32_t ERROR_SESSION_MISMATCH = 0x05;
        constexpr uint32_t ERROR_TARGET_UNSUPPORTED = 0x06;
        constexpr uint32_t ERROR_INVALID_ARGUMENT = 0x07;
        constexpr uint32_t ERROR_NONCONTIGUOUS_OFFSET = 0x08;

        auto send_reply = [&](const uint8_t command, const uint16_t session_id,
                              const uint8_t status, const uint32_t info)
        {
            uint8_t reply[8]{};
            reply[0] = command;
            reply[1] = (uint8_t)(session_id & 0xFF);
            reply[2] = (uint8_t)(session_id >> 8);
            reply[3] = status;
            reply[4] = (uint8_t)(info & 0xFF);
            reply[5] = (uint8_t)(info >> 8);
            reply[6] = (uint8_t)(info >> 16);
            reply[7] = (uint8_t)(info >> 24);
            TransmitPacket(10, sizeof(reply), reply);
        };
        auto reset_session = [&]()
        {
            file_download = {};
            vssc = 0;
        };
        auto update_progress = [&]()
        {
            if(file_download.file_size == 0)
            {
                vssc = 0;
                return;
            }
            const uint64_t progress =
                (uint64_t)file_download.current_file_offset * 100ULL /
                file_download.file_size;
            vssc = (uint8_t)MIN(progress, 100ULL);
        };

        // The public header is mandatory even for an error reply, because the
        // sender needs COMMAND_ECHO and SESSION_ID to match the transaction.
        if(payload == nullptr || len < 3) return;
        const uint8_t command = payload[0];
        const uint16_t session_id = (uint16_t)payload[1] |
                                    ((uint16_t)payload[2] << 8);
        if(session_id == 0 || command > COMMAND_ABORT)
        {
            send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
            return;
        }
        if(GetMotor<FOCMotor>()->GetCurrentState() != MotorState::IDLE)
        {
            send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
            return;
        }
        switch(command)
        {
            case COMMAND_BEGIN:
            {
                if(len != 8)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                    break;
                }
                const uint8_t target = payload[3];
                const uint32_t file_size = (uint32_t)payload[4] |
                                           ((uint32_t)payload[5] << 8) |
                                           ((uint32_t)payload[6] << 16) |
                                           ((uint32_t)payload[7] << 24);
                if(target > 1)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_TARGET_UNSUPPORTED);
                    break;
                }
                if(file_size == 0)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                    break;
                }

                file_download.state = file_download.RECEIVING;
                file_download.target = target;
                file_download.session_id = session_id;
                file_download.file_size = file_size;

                // if(file_download.state != file_download.IDLE &&
                //     file_download.session_id == session_id)
                {
                    if(file_download.target != target ||
                       file_download.file_size != file_size)
                    {
                        send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                        break;
                    }

                    if(HAL::Bootloader::HasBL())
                    {
                        BootloaderMsg message{};
                        message.server_node_id = node_id;
                        HAL::Bootloader::JumpToBL(message);
                    }

                    // JumpToBL normally reboots and does not return. Treat a
                    // returned call as a failed handoff instead of acknowledging
                    // a file that this application cannot store.
                    reset_session();
                    send_reply(command, session_id, STATUS_ERROR, ERROR_TARGET_UNSUPPORTED);
                    // send_reply(command, session_id, STATUS_ACK, 0);
                    break;
                }

                // file_download.state = file_download.RECEIVING;
                // send_reply(command, session_id, 0, 0);

                // APP test mode: receive each DATA frame and acknowledge its
                // offset, but do not erase or program flash.
                // reset_session();
                // file_download.state = file_download.RECEIVING;
                // file_download.target = target;
                // file_download.session_id = session_id;
                // file_download.file_size = file_size;
                // send_reply(command, session_id, STATUS_ACK, 0);
                break;
            }
            case COMMAND_DATA:
            {
                if(file_download.state == file_download.IDLE ||
                   file_download.session_id != session_id)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_SESSION_MISMATCH);
                    break;
                }
                if(file_download.state != file_download.RECEIVING || len < 9)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                    break;
                }
                const uint32_t file_offset = (uint32_t)payload[3] |
                                             ((uint32_t)payload[4] << 8) |
                                             ((uint32_t)payload[5] << 16) |
                                             ((uint32_t)payload[6] << 24);
                const uint8_t chunk_len = payload[7];
                const uint8_t logical_len = (uint8_t)(8 + chunk_len);
                if(chunk_len == 0 || chunk_len > 56 ||
                   (uint8_t)can->GetDLCFromLength(logical_len) != len ||
                   file_offset > file_download.file_size ||
                   chunk_len > file_download.file_size - file_offset)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                    break;
                }
                for(uint8_t index = logical_len; index < len; ++index)
                {
                    if(payload[index] != 0)
                    {
                        send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                        return;
                    }
                }

                if(file_offset == file_download.current_file_offset)
                {
                    // Test-only sink: retain one completed chunk solely to
                    // identify a retry whose ACK was lost.
                    file_download.last_data_offset = file_offset;
                    file_download.last_data_len = chunk_len;
                    memcpy(file_download.last_data, payload + 8, chunk_len);
                    file_download.last_data_valid = 1;
                    file_download.current_file_offset += chunk_len;
                    update_progress();
                    send_reply(command, session_id, STATUS_ACK,
                               file_download.current_file_offset);
                    break;
                }
                if(file_offset < file_download.current_file_offset &&
                   file_download.last_data_valid &&
                   file_offset == file_download.last_data_offset &&
                   chunk_len == file_download.last_data_len &&
                   memcmp(payload + 8, file_download.last_data, chunk_len) == 0)
                {
                    send_reply(command, session_id, STATUS_ACK,
                               file_download.current_file_offset);
                    break;
                }
                send_reply(command, session_id, STATUS_ERROR,
                           file_offset > file_download.current_file_offset
                               ? ERROR_NONCONTIGUOUS_OFFSET
                               : ERROR_INVALID_ARGUMENT);
                break;
            }
            case COMMAND_FIN:
            {
                if(file_download.state == file_download.IDLE ||
                   file_download.session_id != session_id)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_SESSION_MISMATCH);
                    break;
                }
                if(len != 12 || payload[11] != 0)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                    break;
                }
                uint64_t expected_crc64 = 0;
                memcpy(&expected_crc64, payload + 3, sizeof(expected_crc64));
                if(file_download.state == file_download.COMPLETE)
                {
                    send_reply(command, session_id,
                               expected_crc64 == file_download.expected_crc64
                                   ? STATUS_VERIFY_OK : STATUS_ERROR,
                               expected_crc64 == file_download.expected_crc64
                                   ? 0 : ERROR_INVALID_ARGUMENT);
                    break;
                }
                if(file_download.state != file_download.RECEIVING ||
                   file_download.current_file_offset != file_download.file_size)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                    break;
                }
                // APP test mode deliberately skips flash flush and CRC64
                // calculation. The expected CRC is retained only so repeated
                // FIN frames are deterministic.
                file_download.expected_crc64 = expected_crc64;
                file_download.state = file_download.COMPLETE;
                vssc = 100;
                send_reply(command, session_id, STATUS_VERIFY_OK, 0);
                break;
            }
            case COMMAND_ABORT:
            {
                if(len != 3)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_INVALID_ARGUMENT);
                    return;
                }
                if(file_download.state != file_download.IDLE &&
                   file_download.session_id != session_id)
                {
                    send_reply(command, session_id, STATUS_ERROR, ERROR_SESSION_MISMATCH);
                    return;
                }
                // ABORT is idempotent. An idle application has committed zero
                // bytes, so it can still acknowledge a repeated abort.
                const uint32_t current_file_offset = file_download.current_file_offset;
                reset_session();
                send_reply(command, session_id, STATUS_ACK, current_file_offset);
                break;
            }
            default: break;
        }
    }

    void CANFDProtocol::ResetParamSession()
    {
        param_session.state = 0;
        param_session.session_id = 0;
        param_session.flags = 0;
        param_session.request_total_len = 0;
        param_session.request_len = 0;
        param_session.request_crc16 = 0;
        param_session.response_len = 0;
        param_session.response_complete = 0;
    }

    bool CANFDProtocol::ParseMotionHeader(const uint8_t header)
    {
        control_struct.Reset();
        control_struct.motion.ref = ((header & 0x01) == 1) ? Motion::Ref::OUTPUT : Motion::Ref::BASE;
        control_struct.motion.torque.unit = (((header & 0x02) >> 1) == 1) ? Motion::TorqueUnit::NM : Motion::TorqueUnit::AMP;
        const uint8_t speed_unit = (header & 0x1C) >> 2;
        switch(speed_unit)
        {
            case 0:
            {
                control_struct.motion.speed.unit = Motion::SpeedUnit::RADS;
                break;
            }
            case 1:
            {
                control_struct.motion.speed.unit = Motion::SpeedUnit::DEGS;
                break;
            }
            case 2:
            {
                control_struct.motion.speed.unit = Motion::SpeedUnit::REVS;
                break;
            }
            case 3:
            {
                control_struct.motion.speed.unit = Motion::SpeedUnit::RPM;
                break;
            }
            case 4:
            {
                control_struct.motion.speed.unit = Motion::SpeedUnit::HZ;
                break;
            }
            default:
            {
                control_struct.Reset();
                return false;
            }
        }
        const uint8_t pos_unit = (header & 0x60) >> 5;
        switch(pos_unit)
        {
            case 0:
            {
                control_struct.motion.pos.unit = Motion::PosUnit::RAD;
                break;
            }
            case 1:
            {
                control_struct.motion.pos.unit = Motion::PosUnit::DEG;
                break;
            }
            case 2:
            {
                control_struct.motion.pos.unit = Motion::PosUnit::REV;
                break;
            }
            default:
            {
                control_struct.Reset();
                return false;
            }
        }
        control_struct.is_latched = ((header & 0x80) >> 7);
        return true;
    }

    void CANFDProtocol::ProcessControlStruct(const bool latch_arrived)
    {
        if(control_struct.is_latched && !latch_arrived) return;
        const auto motor = GetMotor<FOCMotor>();
        motor->UpdateWatchdog();
        auto mode = MotorControlMode::CTRL_MODE_CURRENT;
        switch(control_struct.command)
        {
            case 0: break;
            case 1:
            {
                mode = MotorControlMode::CTRL_MODE_VELOCITY;
                break;
            }
            case 2:
            {
                mode = MotorControlMode::CTRL_MODE_POSITION;
                break;
            }
            case 3:
            case 4:
            {
                mode = MotorControlMode::CTRL_MODE_HYBRID;
                break;
            }
            default: return;
        }
        Motion target_copy = control_struct.motion;
        if(control_struct.is_relative)
        {
            // modify target_copy
            if(control_struct.is_relative_curr_based)
            {
                const auto curr_motion = motor->GetCurrentMotionStruct(target_copy);
                target_copy.pos.value += curr_motion.pos.value;
            }
            else
            {
                const auto last_target_motion = motor->GetTargetMotionStruct(target_copy);
                target_copy.pos.value += last_target_motion.pos.value;
            }
        }
        motor->SetControlMode(mode);
        if(mode == MotorControlMode::CTRL_MODE_POSITION && control_struct.is_trajectory)
        {
            motor->SetTrajectoryTargetMotion(target_copy, control_struct.is_trajectory_s_curve);
        }
        else motor->SetTargetMotion(target_copy);
        control_struct.Reset();
    }

    void CANFDProtocol::SendOpcodeSession()
    {
        if(opcode_session.argument_len > 62 || opcode_session.argument_echo_len > 61) return;
        buffer[0] = opcode_session.session_id;
        buffer[1] = opcode_session.opcode;
        buffer[2] = opcode_session.status;
        memcpy(buffer + 3, &opcode_session.argument_echo[0], opcode_session.argument_echo_len);
        TransmitPacket(3, 3 + opcode_session.argument_echo_len, buffer);
    }

    void CANFDProtocol::SendNodeStatus()
    {
        const auto motor = GetMotor<FOCMotor>();
        uint8_t payload[12];
        const uint32_t uptime_sec = HAL::GetUptimeSeconds();
        memcpy(&payload[0], &uptime_sec, 4);
        const uint64_t error = motor->GetError();
        payload[4] = count_bits(error); // HEALTH
        const bool is_initialization_mode = motor->CheckError(MotorError::MOTOR_PHASE_RESISTANCE_OUT_OF_RANGE) ||
                motor->CheckError(MotorError::MOTOR_PHASE_INDUCTANCE_OUT_OF_RANGE) ||
                motor->CheckError(MotorError::MOTOR_POLE_PAIR_NUMBER_OUT_OF_RANGE) ||
                motor->CheckError(MotorError::MOTOR_CURR_SENSE_CALIBRATION_TIMEOUT) ||
                motor->CheckError(MotorError::STARTUP_SEQ_REQUIREMENTS_UNMET) ||
                motor->CheckError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING) ||
                motor->CheckError(MotorError::PRIMARY_SENSOR_INIT_FAILED) ||
                motor->CheckError(MotorError::PRIMARY_SENSOR_CALIBRATION_FAILED) ||
                motor->CheckError(MotorError::AUXILIARY_SENSOR_INIT_FAILED) ||
                motor->CheckError(MotorError::AUXILIARY_SENSOR_CALIBRATION_FAILED) ||
                motor->CheckError(MotorError::CONFIG_BOARD_CONFIG_INVALID) ||
                motor->CheckError(MotorError::CONFIG_CURR_SENSE_CONFIG_INVALID) ||
                motor->CheckError(MotorError::CONFIG_BUS_SENSE_CONFIG_INVALID) ||
                motor->CheckError(MotorError::CURR_SENSE_COMPONENT_MISSING) ||
                motor->CheckError(MotorError::CURR_SENSE_INIT_FAILED) ||
                motor->CheckError(MotorError::BUS_SENSE_COMPONENT_MISSING) ||
                motor->CheckError(MotorError::BUS_SENSE_INIT_FAILED) ||
                motor->CheckError(MotorError::BUS_SENSE_DEV_ID_MISMATCH) ||
                motor->CheckError(MotorError::DRIVER_COMPONENT_MISSING) ||
                motor->CheckError(MotorError::DRIVER_INIT_FAILED) ||
                motor->CheckError(MotorError::DRIVER_COMMUNICATION_ERROR) ||
                motor->CheckError(MotorError::DRIVER_DEV_ID_MISMATCH);
        payload[5] = is_initialization_mode ? 1 : 0; // MODE
        payload[6] = (uint8_t)motor->GetCurrentState(); // SUB_MODE
        payload[7] = vssc;
        const uint32_t uuid = HAL::GetSerialNumber();
        memcpy(&payload[8], &uuid, 4);
        TransmitPacket(1, 12, payload);
    }

    void CANFDProtocol::SendMiscFeedback()
    {
        const auto motor = GetMotor<FOCMotor>();
        constexpr float divVOLT_PER_LSB = 1.0f / (0.1f);
        constexpr float divAMPERE_PER_LSB = 1.0f / (0.1f);
        const auto dc_bus_voltage = (uint16_t)MIN(motor->GetBusSense()->voltage * divVOLT_PER_LSB, 8191);
        const auto dc_bus_current = (int16_t)_constrain(motor->GetBusSense()->current * divAMPERE_PER_LSB, -4096, 4095);
        const auto core_temp_celsius = (int16_t)_constrain(motor->GetCoreTempSense() ? motor->GetCoreTempSense()->temp_celsius : 0, -256, 255);
        const auto mosfet_temp_celsius = (int16_t)_constrain(motor->GetMosfetTempSense() ? motor->GetMosfetTempSense()->temp_celsius : 0, -256, 255);
        const auto motor_temp_celsius = (int16_t)_constrain(motor->GetMotorTempSense() ? motor->GetMotorTempSense()->temp_celsius : 0, -256, 255);
        const auto response_avg_us = (uint8_t)MIN((uint16_t)task_time_avg.response_avg.GetCurrent(), 63); // uint6
        const auto response_max_us = (uint8_t)MIN(response_timer.max_elapsed_time_us, 63); // uint6
        const auto rt_time_avg_us = (uint8_t)MIN((uint16_t)task_time_avg.rt_task_avg.GetCurrent(), 63); // uint6
        const auto rt_time_max_us = (uint8_t)MIN(motor->task_times.rt_main_task.max_elapsed_time_us, 63); // uint6
        const auto mid_avg_us = (uint8_t)MIN(task_time_avg.mid_task_avg.GetCurrent(), 63); // uint6
        const auto mid_max_us = (uint8_t)MIN(motor->task_times.mid_interval_task.max_elapsed_time_us, 63); // uint6
        uint8_t payload[12];
        payload[0] = dc_bus_voltage & 0xFF;
        payload[1] = ((dc_bus_voltage & 0x1F00) >> 8) | ((dc_bus_current & 0x07) << 5);
        payload[2] = (dc_bus_current & 0x07F8) >> 3;
        payload[3] = ((dc_bus_current & 0x1800) >> 11) | ((core_temp_celsius & 0x3F) << 2);
        payload[4] = ((core_temp_celsius & 0x01C0) >> 6) | ((mosfet_temp_celsius & 0x1F) << 3);
        payload[5] = ((mosfet_temp_celsius & 0x01E0) >> 5) | ((motor_temp_celsius & 0x0F) << 4);
        payload[6] = ((motor_temp_celsius & 0x01F0) >> 4) | ((response_avg_us & 0x07) << 5);
        payload[7] = ((response_avg_us & 0x38) >> 3) | ((response_max_us & 0x1F) << 3);
        payload[8] = ((response_max_us & 0x20) >> 5) | ((rt_time_avg_us & 0x3F) << 1) | ((rt_time_max_us & 0x01) << 7);
        payload[9] = ((rt_time_max_us & 0x3E) >> 1) | ((mid_avg_us & 0x07) << 5);
        payload[10] = ((mid_avg_us & 0x38) >> 3) | ((mid_max_us & 0x1F) << 3);
        payload[11] = ((mid_max_us & 0x20) >> 5) | ((cpu_usage_pct & 0x7F) << 1);
        TransmitPacket(14, 12, payload);
    }

    void CANFDProtocol::SendRTFeedback()
    {
        const auto motor = GetMotor<FOCMotor>();
        motor->UpdateWatchdog();
        const Motion current = motor->GetCurrentMotionStruct(Motion::Ref::OUTPUT,
        Motion::TorqueUnit::NM,
        Motion::SpeedUnit::RADS,
        Motion::PosUnit::RAD);
        const auto single_round_u16 = (uint16_t)(normalize_rad(current.pos.value) * divPI2 * 65535.0f);
        uint8_t payload[16];
        payload[0] = ((uint8_t)motor->GetCurrentState() & 0xF) |
                        ((uint8_t)motor->GetControlMode() & 0x3) << 4 |
                        (((uint8_t)(motor->GetError() > 0))) << 6 |
                        (((uint8_t)motor->IsArmed())) << 7;
        payload[1] = single_round_u16 & 0xFF;
        payload[2] = single_round_u16 >> 8;
        memcpy(&payload[3], &current.pos.value, 4); // OUTPUT_MULTI_ROUND
        memcpy(&payload[7], &current.speed.value, 4); // OUTPUT_VELOCITY_RAD_S
        memcpy(&payload[11], &current.torque.value, 4); // OUTPUT_TORQUE_NM
        payload[15] = rt_feedback_seq;
        rt_feedback_seq++;
        TransmitPacket(15, 16, payload);
    }

    void CANFDProtocol::PrepareNodeInfo()
    {
        const char* node_name = GetMotor<FOCMotor>()->GetConfig().node_name();
        const auto node_name_len = strnlen(node_name, 13);
        node_info_packet.data_len = 18 + node_name_len;
        uint32_t temp = HAL::GetSerialNumber();
        memcpy(&node_info_packet.data[0], &temp, 4); // UUID
        node_info_packet.data[4] = get_sw_ver_major(); // SW_VER_MAJOR
        temp = get_sw_ver_vcs();
        memcpy(&node_info_packet.data[5], &temp, 4); // SW_VER_MINOR
        const uint64_t crc = HAL::GetFirmwareCRC64();
        memcpy(&node_info_packet.data[9], &crc, 8); // SW_CRC64
        node_info_packet.data[17] = node_name_len;  // NODE_NAME_LEN
        memcpy(&node_info_packet.data[18], node_name, node_name_len); // NODE_NAME
    }

    void CANFDProtocol::TransmitPacket(uint8_t frame_id, const uint8_t payload_len, uint8_t* payload) const
    {
        if(payload_len > 64) return; // invalid payload len
        // Step #1: construct CAN ID (11-bits)
        frame_id = frame_id & 0xF;
        const uint16_t id = (FIDToClass(frame_id) << 9) | (1 << 8) | (node_id << 4) | frame_id;
        // Step #2: determine CANFD payload len
        DataType::Comm::CANFDMessage msg{};
        msg.dlc = can->GetDLCFromLength(payload_len);
        // Step #3: set 11-bit id
        msg.is_ext = false;
        msg.cob_id[0] = (id & 0xFF);
        msg.cob_id[1] = (id >> 8) & 0xFF;
        // Step #4: set payload
        memcpy(msg.data, payload, payload_len);
        // Step #5: send msg
        can->TransmitMessage(msg);
    }

    void CANFDProtocol::SetNodeID(uint8_t new_id)
    {
        if(new_id == 0 || new_id > BROADCAST_ID) new_id = BROADCAST_ID; // id invalid
        const auto motor = GetMotor<FOCMotor>();
        // Set HW filter: DIRECTION == 0, NODE_ID = node_id
        // HW filter id: motor->GetInternalID() * 2 & motor->GetInternalID() * 2 + 1
        // mask: 00111110000b (11-bit standard id) -> 0x1F0
        //   id: 000|id|0000b -> (new_id & 0xF) << 4
        // with FRAME_ID = 0: mask: 11111111111b -> 0x7FF, id: 10011110000b -> 0x4F0 (Dynamic Node Allocation)
        can->SetHWFilter(motor->GetInternalID() * 2, (new_id & 0xF) << 4, 0x1F0, false);
        can->SetHWFilter(motor->GetInternalID() * 2 + 1, 0x4F0, 0x7FF, false);
        node_id = new_id;
        motor->GetConfig().set_node_id(new_id);
    }

    bool CANFDProtocol::OnRxEvent(const DataType::Comm::CANFDMessage& msg)
    {
        task_time_avg.response_avg.GetOutput(response_timer.elapsed_time_us);
        MEASURE_TIME(response_timer)
        {
            if(msg.is_ext || msg.cob_id[2] > 0x00 || msg.cob_id[3] > 0x00) return false; // ID invalid
            const uint16_t id = msg.cob_id[0] | ((uint16_t)msg.cob_id[1] << 8);
            if(id > 0x7FF) return false;
            const uint8_t class_id = (id & 0x600) >> 9;
            const uint8_t direction = (id & 0x100) >> 8;
            const uint8_t target_node_id = (id & 0xF0) >> 4;
            const uint8_t frame_id = (id & 0xF);
            const uint8_t payload_len = msg.dlc;
            // Step #1: validate direction
            if(direction) return false;
            // Step #2: validate class_id vs frame_id
            if(FIDToClass(frame_id) != class_id) return false;
            // Step #3: validate target_node_id
            if(target_node_id != node_id && (target_node_id == BROADCAST_ID && frame_id == 0x00)) return false;
            switch(frame_id)
            {
                case 0: // Dynamic Node Allocation
                {
                    HandleNodeAllocation(msg.data, payload_len);
                    return true;
                }
                case 1: // Node Status Upload (self-upload typically)
                {
                    SendNodeStatus();
                    return true;
                }
                case 2: // Get Node Info
                {
                    HandleGetNodeInfo(msg.data, payload_len);
                    return true;
                }
                case 3: // Execute Opcode
                {
                    HandleExecuteOpcode(msg.data, payload_len);
                    return true;
                }
                case 4: // Param Get/Set
                {
                    HandleParamGetSet(msg.data, payload_len);
                    return true;
                }
                case 7: // Position Control
                {
                    HandlePosControl(msg.data, payload_len);
                    return true;
                }
                case 8: // Velocity / torque control
                {
                    HandleVelTorqueControl(msg.data, payload_len);
                    return true;
                }
                case 10: // File Download
                {
                    HandleFileDownload(msg.data, payload_len);
                    return true;
                }
                case 13: // Sync
                {
                    ProcessControlStruct(true);
                    return true;
                }
                case 14: // Misc Feedback polling
                {
                    SendMiscFeedback();
                    return true;
                }
                case 15: // RT Feedback polling
                {
                    SendRTFeedback();
                    return true;
                }
                default: break;
            }
        }
        return false; // frame not successfully handled by this instance
    }
}
