#include "rs485_protocol.hpp"

#include "foc_motor.hpp"

#define HOST_ID (0)
#define BROADCAST_ID (15)

constexpr TickType_t PARAM_SESSION_TIMEOUT_TICKS = 500;
constexpr TickType_t MISC_FEEDBACK_SEND_INTERVAL_TICKS = 20;

namespace iFOC::Protocol
{
RS485Protocol::RS485Protocol(HAL::UARTHSBase* u) : uart(u)
{

}

RS485Protocol::~RS485Protocol()
{
    const auto motor = GetMotor<FOCMotor>();
    motor->RemoveTaskByName("RS485");
}

void RS485Protocol::Init()
{
    const auto motor = GetMotor<FOCMotor>();
    const auto now_id = motor->GetConfig().node_id();
    if(now_id >= 1 && now_id < BROADCAST_ID) node_id = now_id;
    else node_id = BROADCAST_ID;
    BoardConfig().GetConfig().GetReflectMap(); // generate reflect map first, to avoid generate in interrupt
    motor->GetConfig().GetReflectMap();
    FillNodeDetailedInfo();
    FillMiscFeedback();
    control_frame_last_misc_send_tick = xTaskGetTickCount();
    motor->AppendTask(&worker);
    uart->RegisterIdleCallback(std::bind(&RS485Protocol::IdleCallback, this, std::placeholders::_1));
}

void RS485Protocol::SetScopeData(const uint8_t ch, const float data)
{
    if(ch >= scope_data.size()) return;
    scope_data[ch] = data;
    if(ch + 1 > scope_channel_cnt) scope_channel_cnt = ch + 1;
}

void RS485Protocol::IdleCallback(HAL::UARTHSBase* uart)
{
    MEASURE_TIME(response_timer)
    {
        // Consume all in FIFO at once
        uint8_t byte = 0;
        while(uart->rx_fifo.used())
        {
            uint8_t new_byte;
            uart->rx_fifo.get(&new_byte, 1);
            if(new_byte == 0x55 && byte == 0xAA) // Found header from Host
            {
                byte = 0;
                if(uart->rx_fifo.used() >= 4) // there's enough for target_info, data_len & crc16
                {
                    uint8_t info_and_len[2];
                    uart->rx_fifo.get(info_and_len, 2);
                    // now we have at least data_len + 2 bytes if correct
                    if(uart->rx_fifo.used() >= info_and_len[1] + 2)
                    {
                        // check ID first
                        rx_packet.info.byte = info_and_len[0];
                        if(rx_packet.info.bit.id == node_id || rx_packet.info.bit.id == BROADCAST_ID)
                        {
                            // ID matched, extracting payload (peek first)
                            rx_packet.data_len = info_and_len[1];
                            uart->rx_fifo.peek(rx_packet.data, info_and_len[1] + 2);
                            // now rx_packet is completed, with CRC16 stored in rx_packet.data
                            // check CRC16 (Little-Endian)
                            const uint16_t crc16_get = rx_packet.data[info_and_len[1]] | (rx_packet.data[info_and_len[1] + 1] << 8);
                            if(get_crc16((uint8_t*)&rx_packet, info_and_len[1] + 2) == crc16_get)
                            {
                                // CRC matched, wipe_n then process
                                uart->rx_fifo.wipe_n(info_and_len[1] + 2);
                                ProcessRxPacket();
                            }
                            else
                            {
                                // CRC mismatch

                            }
                        }
                        else
                        {
                            // ID mismatch, wipe_n
                            rx_packet.info.byte = 0;
                            rx_packet.data_len = 0;
                            uart->rx_fifo.wipe_n(info_and_len[1] + 2);
                        }
                    }
                    else uart->rx_fifo.flush();
                }
                else uart->rx_fifo.flush();
            }
            else byte = new_byte;
        }
    }
}

void RS485Protocol::ProcessRxPacket()
{
    // rx_packet
    switch(rx_packet.info.bit.frame_id)
    {
        case 0: // FRAME_ID = 0, New Node Discovery
        {
            HandleNewNodeDiscovery();
            break;
        }
        case 1: // FRAME_ID = 1, Node Status Polling
        {
            if(node_id != BROADCAST_ID &&
                rx_packet.data_len == 0) SendNodeStatus(1);
            break;
        }
        case 2: // FRAME_ID = 2, Node Info Polling
        {
            if(node_id != BROADCAST_ID &&
                rx_packet.data_len == 0) SendNodeDetailedInfo();
            break;
        }
        case 3: // FRAME_ID = 3, Execute Opcode
        {
            if(node_id != BROADCAST_ID) HandleExecuteOpcode();
            break;
        }
        case 4: // FRAME_ID = 4, Parameter Get/Set
        {
            if(node_id != BROADCAST_ID) HandleParamGetSet();
            break;
        }
        case 7: // FRAME_ID = 7, Position Control
        {
            if(node_id != BROADCAST_ID) HandlePosControl();
            break;
        }
        case 8: // FRAME_ID = 8, Velocity / Torque Control
        {
            if(node_id != BROADCAST_ID) HandleVelTorqueControl();
            break;
        }
        case 10: // FRAME_ID = 10, File Download
        {
            if(node_id != BROADCAST_ID) HandleFileDownload();
            break;
        }
        case 12: // FRAME_ID = 12, Serial Scope
        {
            if(node_id != BROADCAST_ID) HandleSerialScope();
            break;
        }
        case 13: // FRAME_ID = 13, Sync Broadcast Frame (BROADCAST_ID)
        {
            if(node_id != BROADCAST_ID &&
                rx_packet.info.bit.id == BROADCAST_ID &&
                rx_packet.data_len == 0)
            {
                // Latch arrived
                ProcessControlStruct(true);
            }
            break;
        }
        case 14: // FRAME_ID = 14, Misc Feedback
        {
            if(node_id != BROADCAST_ID) SendMiscFeedback();
            break;
        }
        case 15: // FRAME_ID = 15, RT Feedback
        {
            if(node_id != BROADCAST_ID) SendRTFeedback();
            break;
        }
        default: break;
    }
}

void RS485Protocol::HandleNewNodeDiscovery()
{
    if(rx_packet.data_len == 0) return;
    const auto motor = GetMotor<FOCMotor>();
    const uint8_t command = rx_packet.data[0];
    if(command == 0)
    {
        if(rx_packet.data_len != 5 || node_id != BROADCAST_ID) return;
        const uint16_t modulus = rx_packet.data[1] | ((uint16_t)rx_packet.data[2] << 8);
        const uint16_t bucket_index = rx_packet.data[3] | ((uint16_t)rx_packet.data[4] << 8);
        if(modulus == 0 || bucket_index >= modulus) return;
        const uint32_t uuid = HAL::GetSerialNumber();
        if(uuid % modulus == bucket_index) SendNodeStatus(0);
    }
    else if(command == 1)
    {
        if(rx_packet.data_len != 6) return;
        uint32_t uuid_get = rx_packet.data[1] | ((uint32_t)rx_packet.data[2] << 8) | ((uint32_t)rx_packet.data[3] << 16) | ((uint32_t)rx_packet.data[4] << 24);
        if(uuid_get == HAL::GetSerialNumber())
        {
            uint8_t new_node_id = rx_packet.data[5];
            if(new_node_id >= 1 && new_node_id <= BROADCAST_ID && new_node_id != node_id)
            {
                node_id = new_node_id;
                motor->GetConfig().set_node_id(node_id);
                SendNodeStatus(0);
            }
        }
    }
}

void RS485Protocol::HandleExecuteOpcode()
{
    if(rx_packet.data_len != 9) return;
    const uint8_t opcode = rx_packet.data[0];
    uint64_t argument = 0;
    memcpy(&argument, &rx_packet.data[1], 8);
    tx_packet.info.bit.id = node_id;
    tx_packet.info.bit.frame_id = 3;
    tx_packet.data_len = 9;
    memset(tx_packet.data, 0, 9);
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    switch(opcode)
    {
        case 1: // SAVE_PARAM
        {
            if(argument > 2UL)
            {
                WriteTxPacket();
                break;
            }
            tx_packet.data[0] = 0x02;
            tx_packet.data[1] = (uint8_t)argument;
            WriteTxPacket();
            if(argument == 0UL)
            {
                BoardConfig().SaveNVMConfig();
            }
            else
            {
                motor->config.SaveNVMConfig();
            }
            break;
        }
        case 2: // ERASE_PARAM
        {
            if(argument > 2UL)
            {
                WriteTxPacket();
                break;
            }
            tx_packet.data[0] = 0x02;
            tx_packet.data[1] = (uint8_t)argument;
            WriteTxPacket();
            if(argument == 0UL)
            {
                BoardConfig().ClearNVMConfig();
            }
            else
            {
                motor->config.ClearNVMConfig();
                motor->GetConfig().set_node_id(node_id);
            }
            break;
        }
        case 3: // RESTART_NODE, no return
        {
            if(argument > 2UL)
            {
                WriteTxPacket();
                break;
            }
            if(argument == 0UL) HAL::SystemReboot();
            else if(!HAL::Bootloader::HasBL())
            {
                WriteTxPacket();
                break;
            }
            BootloaderMsg msg{};
            msg.server_node_id = node_id; // reuse server_node_id as node_id, to speed up discovery process
            HAL::Bootloader::JumpToBL(msg);
            break;
        }
        case 4: // TOGGLE_BEEP_IDENTIFY
        {
            if(motor->GetCurrentState() != MotorState::IDLE)
            {
                tx_packet.data[0] = 0x01;
                WriteTxPacket();
                break;
            }
            tx_packet.data[0] = 0x02;
            tx_packet.data[1] = motor->ToggleBeepIdentify() == FuncRetCode::OK ? 1 : 0;
            WriteTxPacket();
            break;
        }
        case 5: // GET_ERROR
        {
            tx_packet.data[0] = 0x02;
            const uint64_t error = motor->GetError();
            memcpy(&tx_packet.data[1], &error, 8);
            WriteTxPacket();
            break;
        }
        case 6: // CLEAR_ERROR_MASK
        {
            motor->ClearError(argument);
            const uint64_t error = motor->GetError();
            if((error & argument) > 0UL) tx_packet.data[0] = 0x01;
            else tx_packet.data[0] = 0x02;
            memcpy(&tx_packet.data[1], &error, 8);
            WriteTxPacket();
            break;
        }
        case 7: // SET_MOTOR_STATE
        {
            if(argument > (uint32_t)MotorState::OPEN_LOOP_VELOCITY_CONTROL)
            {
                WriteTxPacket();
                break;
            }
            const MotorState req_state = (MotorState)argument;
            const uint8_t ret_state = (uint8_t)motor->state_machine.RequestState(req_state);
            if(ret_state == (uint8_t)argument) tx_packet.data[0] = 0x02;
            else tx_packet.data[0] = 0x01;
            tx_packet.data[1] = ret_state;
            WriteTxPacket();
            break;
        }
        case 10: // ERASE_NVM_USER_AREA
        {
            if((argument & 0xFF) != 0x11 ||
                ((argument >> 8) & 0xFF) != 0x22 ||
                ((argument >> 16) & 0xFF) != 0x33 ||
                ((argument >> 24) & 0xFF) != 0x44)
            {
                WriteTxPacket();
                break;
            }
            if(motor->GetCurrentState() != MotorState::IDLE)
            {
                WriteTxPacket();
                break;
            }
            if(BlobNVMStorage().ClearAllNVM() == FuncRetCode::OK) tx_packet.data[0] = 0x02;
            else tx_packet.data[0] = 0x01;
            WriteTxPacket();
            break;
        }
        default:
        {
            WriteTxPacket();
            break;
        }
    }
}

void RS485Protocol::HandleParamGetSet()
{
    if(rx_packet.data_len < 4) return;
    const uint8_t command = rx_packet.data[0];
    const uint8_t session_id = rx_packet.data[1];
    const uint8_t flags = rx_packet.data[2];
    const uint8_t max_response_len = rx_packet.data[3];

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

    constexpr uint8_t ERROR_INVALID_COMMAND = 0x01;
    constexpr uint8_t ERROR_INVALID_SESSION = 0x02;
    constexpr uint8_t ERROR_BAD_OFFSET = 0x03;
    constexpr uint8_t ERROR_BAD_CRC = 0x04;
    constexpr uint8_t ERROR_PAYLOAD_TOO_LONG = 0x05;
    constexpr uint8_t ERROR_DOMAIN_NOT_FOUND = 0x0C;

    constexpr uint8_t OP_GET_COUNT = 0x00;
    constexpr uint8_t OP_GET_BY_INDEX = 0x01;
    constexpr uint8_t OP_GET_BY_NAME = 0x02;
    constexpr uint8_t OP_SET_BY_INDEX = 0x03;
    constexpr uint8_t OP_SET_BY_NAME = 0x04;
    constexpr uint8_t OP_EXEC_SAVE = 0x05;
    constexpr uint8_t OP_EXEC_ERASE = 0x06;
    constexpr uint8_t OP_EXEC_LOAD_DEFAULT = 0x07;

    constexpr uint8_t RESULT_OK = 0x00;
    constexpr uint8_t RESULT_NOT_FOUND = 0x01;
    constexpr uint8_t RESULT_TYPE_MISMATCH = 0x02;
    constexpr uint8_t RESULT_RANGE_REJECTED = 0x04;
    constexpr uint8_t RESULT_NAME_INDEX_MISMATCH = 0x06;
    constexpr uint8_t RESULT_NOT_ALLOWED = 0x07;
    constexpr uint8_t RESULT_NVM_ERROR = 0x08;

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

    const TickType_t now = xTaskGetTickCount();

    auto reset_session = [&]()
    {
        get_set_struct.current_state = 0;
        get_set_struct.session_id = 0;
        get_set_struct.flags = 0;
        get_set_struct.request_total_len = 0;
        get_set_struct.request_len = 0;
        get_set_struct.request_crc16 = 0;
        get_set_struct.response_len = 0;
        get_set_struct.response_complete = 0;
    };

    if(get_set_struct.current_state != 0 &&
        now - get_set_struct.last_update_tick > PARAM_SESSION_TIMEOUT_TICKS)
    {
        reset_session();
    }

    auto send_reply = [&](uint8_t status, uint8_t sid, uint16_t info, uint8_t info2,
                          const uint8_t* payload = nullptr, uint16_t payload_len = 0)
    {
        if((uint16_t)(5 + payload_len) > (uint16_t)sizeof(tx_packet.data)) return;
        tx_packet.info.bit.id = node_id;
        tx_packet.info.bit.frame_id = 4;
        tx_packet.data_len = 5 + payload_len;
        tx_packet.data[0] = status;
        tx_packet.data[1] = sid;
        tx_packet.data[2] = (uint8_t)(info & 0xFF);
        tx_packet.data[3] = (uint8_t)(info >> 8);
        tx_packet.data[4] = info2;
        if(payload && payload_len) memcpy(&tx_packet.data[5], payload, payload_len);
        WriteTxPacket();
    };

    auto send_error = [&](uint8_t sid, uint8_t error_code)
    {
        send_reply(STATUS_ERROR, sid, error_code, 0);
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

    if(get_set_struct.current_state == 2 && get_set_struct.response_complete &&
        get_set_struct.session_id != session_id &&
        (command == COMMAND_INLINE_REQ || command == COMMAND_BEGIN_REQ))
    {
        reset_session();
    }

    if(get_set_struct.current_state != 0 &&
        get_set_struct.session_id != session_id &&
        command != COMMAND_ABORT)
    {
        send_error(session_id, ERROR_INVALID_SESSION);
        return;
    }

    auto min_u16 = [](uint16_t a, uint16_t b) -> uint16_t
    {
        return a < b ? a : b;
    };

    auto append_bytes = [](uint8_t* dst, uint16_t& len, uint16_t capacity,
                           const void* src, uint16_t src_len) -> bool
    {
        if(len > capacity || src_len > capacity - len) return false;
        if(src && src_len) memcpy(dst + len, src, src_len);
        len += src_len;
        return true;
    };

    auto append_u8 = [&](uint8_t* dst, uint16_t& len, uint16_t capacity, uint8_t value) -> bool
    {
        return append_bytes(dst, len, capacity, &value, 1);
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
    };

    auto copy_param_name = [](ParamTarget& target, const char* name)
    {
        auto len = strnlen(name, sizeof(target.name) - 1);
        if(len > 0 && name[len - 1] == '_') len--;
        memcpy(target.name, name, len);
        target.name[len] = '\0';
        target.name_len = (uint8_t)len;
    };

    auto get_domain = [&](uint8_t domain_id, const ReflectMap*& reflect, uint8_t*& start_ptr) -> bool
    {
        const auto motor = GetMotor<FOCMotor>();
        switch(domain_id)
        {
            case 0:
            {
                reflect = &BoardConfig().GetConfig().GetReflectMap();
                start_ptr = (uint8_t*)&BoardConfig().GetConfig();
                return true;
            }
            case 1:
            {
                reflect = &motor->GetConfig().GetReflectMap();
                start_ptr = (uint8_t*)&motor->GetConfig();
                return true;
            }
            default:
            {
                reflect = nullptr;
                start_ptr = nullptr;
                return false;
            }
        }
    };

    auto get_domain_count = [&](uint8_t domain_id, uint16_t& count) -> bool
    {
        const ReflectMap* reflect = nullptr;
        uint8_t* start_ptr = nullptr;
        if(!get_domain(domain_id, reflect, start_ptr)) return false;
        count = (uint16_t)reflect->size();
        if(domain_id == 1) count += 1; // node_name is stored in motor config but is not in the reflect map.
        return true;
    };

    auto find_param_by_index = [&](uint8_t domain_id, uint8_t index, ParamTarget& target) -> bool
    {
        const auto motor = GetMotor<FOCMotor>();
        const ReflectMap* reflect = nullptr;
        uint8_t* start_ptr = nullptr;
        if(!get_domain(domain_id, reflect, start_ptr)) return false;

        if(domain_id == 1 && index == 0)
        {
            target.info.first = Reflection::ProtoFieldType::STRING;
            target.info.second = sizeof(motor->GetConfig()._d.node_name);
            target.ptr = (uint8_t*)motor->GetConfig()._d.node_name;
            target.index = 0;
            copy_param_name(target, "node_name");
            return true;
        }

        uint16_t iter_index = 0;
        const uint16_t map_index = domain_id == 1 ? (uint16_t)index - 1 : index;
        for(const auto& [name, info] : *reflect)
        {
            if(iter_index == map_index)
            {
                target.info = info;
                target.ptr = start_ptr + info.second;
                target.index = index;
                copy_param_name(target, name);
                return true;
            }
            iter_index++;
        }
        return false;
    };

    auto find_param_by_name = [&](uint8_t domain_id, uint8_t index_hint, const uint8_t* name,
                                  uint8_t name_len, bool index_hint_valid, ParamTarget& target,
                                  uint8_t& result_code) -> bool
    {
        result_code = RESULT_NOT_FOUND;
        if(name_len == 0 || name_len > 31) return false;

        char buffer[40]{};
        memcpy(buffer, name, name_len);
        buffer[name_len] = '\0';
        uint8_t local_len = name_len;

        if(domain_id == 1 && strcmp(buffer, "node_name") == 0)
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

        auto it = reflect->find(buffer);
        if(it == reflect->end() && local_len > 0 && local_len < sizeof(buffer) - 1 && buffer[local_len - 1] != '_')
        {
            buffer[local_len++] = '_';
            buffer[local_len] = '\0';
            it = reflect->find(buffer);
        }
        if(it == reflect->end()) return false;

        uint16_t actual_index = domain_id == 1 ? 1 : 0;
        for(const auto& [map_name, info] : *reflect)
        {
            if(strcmp(map_name, it->first) == 0) break;
            actual_index++;
        }
        if(actual_index > 255) return false;

        target.info = it->second;
        target.ptr = start_ptr + it->second.second;
        target.index = (uint8_t)actual_index;
        copy_param_name(target, it->first);

        if(index_hint_valid && index_hint != target.index)
        {
            result_code = RESULT_NAME_INDEX_MISMATCH;
            return false;
        }
        result_code = RESULT_OK;
        return true;
    };

    auto append_empty_value = [&](uint8_t* dst, uint16_t& len, uint16_t capacity) -> bool
    {
        return append_u8(dst, len, capacity, TYPE_EMPTY);
    };

    auto append_current_value = [&](uint8_t* dst, uint16_t& len, uint16_t capacity,
                                    const ParamTarget& target) -> bool
    {
        switch(target.info.first)
        {
            case Reflection::ProtoFieldType::DOUBLE:
            {
                double value = 0.0;
                memcpy(&value, target.ptr, sizeof(double));
                const float out = (float)value;
                const uint8_t header = (uint8_t)((sizeof(float) << 3) | TYPE_FLOAT32);
                return append_u8(dst, len, capacity, header) &&
                       append_bytes(dst, len, capacity, &out, sizeof(out));
            }
            case Reflection::ProtoFieldType::FLOAT:
            {
                const uint8_t header = (uint8_t)((sizeof(float) << 3) | TYPE_FLOAT32);
                return append_u8(dst, len, capacity, header) &&
                       append_bytes(dst, len, capacity, target.ptr, sizeof(float));
            }
            case Reflection::ProtoFieldType::INT32:
            {
                int32_t value = 0;
                memcpy(&value, target.ptr, sizeof(value));
                const int64_t out = value;
                const uint8_t header = (uint8_t)((sizeof(out) << 3) | TYPE_INT64);
                return append_u8(dst, len, capacity, header) &&
                       append_bytes(dst, len, capacity, &out, sizeof(out));
            }
            case Reflection::ProtoFieldType::INT64:
            {
                const uint8_t header = (uint8_t)((sizeof(int64_t) << 3) | TYPE_INT64);
                return append_u8(dst, len, capacity, header) &&
                       append_bytes(dst, len, capacity, target.ptr, sizeof(int64_t));
            }
            case Reflection::ProtoFieldType::UINT32:
            {
                uint32_t value = 0;
                memcpy(&value, target.ptr, sizeof(value));
                const uint64_t out = value;
                const uint8_t header = (uint8_t)((sizeof(out) << 3) | TYPE_UINT64);
                return append_u8(dst, len, capacity, header) &&
                       append_bytes(dst, len, capacity, &out, sizeof(out));
            }
            case Reflection::ProtoFieldType::UINT64:
            {
                const uint8_t header = (uint8_t)((sizeof(uint64_t) << 3) | TYPE_UINT64);
                return append_u8(dst, len, capacity, header) &&
                       append_bytes(dst, len, capacity, target.ptr, sizeof(uint64_t));
            }
            case Reflection::ProtoFieldType::BOOL:
            {
                const uint8_t value = *target.ptr ? 1 : 0;
                const uint8_t header = (uint8_t)((1 << 3) | TYPE_BOOLEAN);
                return append_u8(dst, len, capacity, header) &&
                       append_u8(dst, len, capacity, value);
            }
            case Reflection::ProtoFieldType::STRING:
            {
                const uint8_t str_len = (uint8_t)min_u16((uint16_t)strnlen((char*)target.ptr, target.info.second), 31);
                const uint8_t header = (uint8_t)((str_len << 3) | TYPE_STRING);
                return append_u8(dst, len, capacity, header) &&
                       append_bytes(dst, len, capacity, target.ptr, str_len);
            }
            default: break;
        }
        return append_empty_value(dst, len, capacity);
    };

    auto write_value = [&](const ParamTarget& target, uint8_t value_type, uint8_t value_len,
                           const uint8_t* value_ptr) -> uint8_t
    {
        switch(target.info.first)
        {
            case Reflection::ProtoFieldType::DOUBLE:
            {
                if(value_type != TYPE_FLOAT32 || value_len != sizeof(float)) return RESULT_TYPE_MISMATCH;
                float in = 0.0f;
                memcpy(&in, value_ptr, sizeof(in));
                const double out = (double)in;
                memcpy(target.ptr, &out, sizeof(out));
                return RESULT_OK;
            }
            case Reflection::ProtoFieldType::FLOAT:
            {
                if(value_type != TYPE_FLOAT32 || value_len != sizeof(float)) return RESULT_TYPE_MISMATCH;
                memcpy(target.ptr, value_ptr, sizeof(float));
                return RESULT_OK;
            }
            case Reflection::ProtoFieldType::INT32:
            {
                if(value_type != TYPE_INT64 || value_len != sizeof(int64_t)) return RESULT_TYPE_MISMATCH;
                int64_t in = 0;
                memcpy(&in, value_ptr, sizeof(in));
                if(in < (-2147483647LL - 1) || in > 2147483647LL) return RESULT_RANGE_REJECTED;
                const int32_t out = (int32_t)in;
                memcpy(target.ptr, &out, sizeof(out));
                return RESULT_OK;
            }
            case Reflection::ProtoFieldType::INT64:
            {
                if(value_type != TYPE_INT64 || value_len != sizeof(int64_t)) return RESULT_TYPE_MISMATCH;
                memcpy(target.ptr, value_ptr, sizeof(int64_t));
                return RESULT_OK;
            }
            case Reflection::ProtoFieldType::UINT32:
            {
                if(value_type != TYPE_UINT64 || value_len != sizeof(uint64_t)) return RESULT_TYPE_MISMATCH;
                uint64_t in = 0;
                memcpy(&in, value_ptr, sizeof(in));
                if(in > 0xFFFFFFFFULL) return RESULT_RANGE_REJECTED;
                const uint32_t out = (uint32_t)in;
                memcpy(target.ptr, &out, sizeof(out));
                return RESULT_OK;
            }
            case Reflection::ProtoFieldType::UINT64:
            {
                if(value_type != TYPE_UINT64 || value_len != sizeof(uint64_t)) return RESULT_TYPE_MISMATCH;
                memcpy(target.ptr, value_ptr, sizeof(uint64_t));
                return RESULT_OK;
            }
            case Reflection::ProtoFieldType::BOOL:
            {
                if(value_type != TYPE_BOOLEAN || value_len != 1) return RESULT_TYPE_MISMATCH;
                const uint8_t out = value_ptr[0] ? 1 : 0;
                memcpy(target.ptr, &out, sizeof(out));
                return RESULT_OK;
            }
            case Reflection::ProtoFieldType::STRING:
            {
                if(value_type != TYPE_STRING || value_len >= target.info.second) return RESULT_TYPE_MISMATCH;
                memcpy(target.ptr, value_ptr, value_len);
                target.ptr[value_len] = '\0';
                return RESULT_OK;
            }
            default: break;
        }
        return RESULT_TYPE_MISMATCH;
    };

    auto append_param_response = [&](uint8_t* rsp, uint16_t& rsp_len, uint8_t op, uint8_t result,
                                     uint8_t domain_id, uint8_t index, uint8_t requested_mask,
                                     const ParamTarget* target) -> bool
    {
        uint8_t present_mask = 0;
        const uint16_t capacity = sizeof(get_set_struct.response_buffer);
        const uint16_t present_pos = 4;

        if(!append_u8(rsp, rsp_len, capacity, op)) return false;
        if(!append_u8(rsp, rsp_len, capacity, result)) return false;
        if(!append_u8(rsp, rsp_len, capacity, domain_id)) return false;
        if(!append_u8(rsp, rsp_len, capacity, index)) return false;
        if(!append_u8(rsp, rsp_len, capacity, 0)) return false;

        if(target && (requested_mask & FIELD_ACCESS_FLAGS))
        {
            const uint8_t access = ACCESS_READABLE | ACCESS_WRITABLE | ACCESS_PERSISTENT;
            if(!append_u8(rsp, rsp_len, capacity, access)) return false;
            present_mask |= FIELD_ACCESS_FLAGS;
        }
        if(target && (requested_mask & FIELD_PARAM_TYPE))
        {
            if(!append_u8(rsp, rsp_len, capacity, param_type_from_reflect(target->info.first))) return false;
            present_mask |= FIELD_PARAM_TYPE;
        }
        if(requested_mask & FIELD_CURRENT_VALUE)
        {
            if(target) {
                if(!append_current_value(rsp, rsp_len, capacity, *target)) return false;
            }
            else {
                if(!append_empty_value(rsp, rsp_len, capacity)) return false;
            }
            present_mask |= FIELD_CURRENT_VALUE;
        }
        if(target && (requested_mask & FIELD_NAME))
        {
            if(!append_u8(rsp, rsp_len, capacity, target->name_len)) return false;
            if(!append_bytes(rsp, rsp_len, capacity, target->name, target->name_len)) return false;
            present_mask |= FIELD_NAME;
        }

        rsp[present_pos] = present_mask;
        return true;
    };

    auto execute_request = [&](const uint8_t* request, uint16_t request_len, uint8_t request_flags,
                               uint8_t* response, uint16_t& response_len, uint8_t& transport_error) -> bool
    {
        transport_error = 0;
        response_len = 0;
        if(request_len < 5)
        {
            transport_error = ERROR_PAYLOAD_TOO_LONG;
            return false;
        }

        const uint8_t op = request[0];
        const uint8_t domain_id = request[1];
        const uint8_t index = request[2];
        uint8_t field_mask = request[3];
        const uint8_t value_type_len = request[4];
        const uint8_t value_type = value_type_len & 0x07;
        const uint8_t value_len = value_type_len >> 3;
        if(value_len > 31 || 5 + value_len > request_len)
        {
            transport_error = ERROR_PAYLOAD_TOO_LONG;
            return false;
        }

        const uint8_t* value_ptr = request + 5;
        uint16_t offset = 5 + value_len;
        uint8_t name_len = 0;
        const uint8_t* name_ptr = nullptr;
        if(op == OP_GET_BY_NAME || op == OP_SET_BY_NAME)
        {
            if(offset >= request_len)
            {
                transport_error = ERROR_PAYLOAD_TOO_LONG;
                return false;
            }
            name_len = request[offset++];
            if(name_len > 31 || offset + name_len > request_len)
            {
                transport_error = ERROR_PAYLOAD_TOO_LONG;
                return false;
            }
            name_ptr = request + offset;
        }

        uint16_t domain_count = 0;
        if(!get_domain_count(domain_id, domain_count))
        {
            transport_error = ERROR_DOMAIN_NOT_FOUND;
            return false;
        }

        if(op == OP_GET_COUNT)
        {
            field_mask = FIELD_CURRENT_VALUE;
            uint64_t count64 = domain_count;
            if(!append_u8(response, response_len, sizeof(get_set_struct.response_buffer), op) ||
                !append_u8(response, response_len, sizeof(get_set_struct.response_buffer), RESULT_OK) ||
                !append_u8(response, response_len, sizeof(get_set_struct.response_buffer), domain_id) ||
                !append_u8(response, response_len, sizeof(get_set_struct.response_buffer), 0) ||
                !append_u8(response, response_len, sizeof(get_set_struct.response_buffer), field_mask) ||
                !append_u8(response, response_len, sizeof(get_set_struct.response_buffer), (uint8_t)((sizeof(count64) << 3) | TYPE_UINT64)) ||
                !append_bytes(response, response_len, sizeof(get_set_struct.response_buffer), &count64, sizeof(count64)))
            {
                transport_error = ERROR_PAYLOAD_TOO_LONG;
                return false;
            }
            return true;
        }

        if(op == OP_EXEC_SAVE || op == OP_EXEC_ERASE || op == OP_EXEC_LOAD_DEFAULT)
        {
            const auto motor = GetMotor<FOCMotor>();
            uint8_t result = RESULT_OK;
            if(motor->GetCurrentState() != MotorState::IDLE)
            {
                result = RESULT_NOT_ALLOWED;
            }
            else if(op == OP_EXEC_SAVE)
            {
                FuncRetCode ret = FuncRetCode::OK;
                if(domain_id == 0) ret = BoardConfig().SaveNVMConfig();
                else ret = motor->config.SaveNVMConfig();
                if(ret != FuncRetCode::OK) result = RESULT_NVM_ERROR;
            }
            else if(op == OP_EXEC_ERASE)
            {
                FuncRetCode ret = FuncRetCode::OK;
                if(domain_id == 0)
                {
                    ret = BoardConfig().ClearNVMConfig();
                    BoardConfig().GetConfig().clear();
                }
                else
                {
                    ret = motor->config.ClearNVMConfig();
                    motor->ResetDefaultConfig();
                }
                if(ret != FuncRetCode::OK) result = RESULT_NVM_ERROR;
            }
            else
            {
                if(domain_id == 0) BoardConfig().GetConfig().clear();
                else motor->ResetDefaultConfig();
            }

            return append_param_response(response, response_len, op, result, domain_id, index, 0, nullptr);
        }

        ParamTarget target{};
        uint8_t result = RESULT_OK;
        bool found = false;
        const bool index_hint_valid = (request_flags & 0x01) != 0;

        switch(op)
        {
            case OP_GET_BY_INDEX:
            case OP_SET_BY_INDEX:
            {
                found = find_param_by_index(domain_id, index, target);
                if(!found) result = RESULT_NOT_FOUND;
                break;
            }
            case OP_GET_BY_NAME:
            case OP_SET_BY_NAME:
            {
                found = find_param_by_name(domain_id, index, name_ptr, name_len, index_hint_valid, target, result);
                break;
            }
            default:
            {
                transport_error = ERROR_INVALID_COMMAND;
                return false;
            }
        }

        if(found && (op == OP_SET_BY_INDEX || op == OP_SET_BY_NAME))
        {
            result = write_value(target, value_type, value_len, value_ptr);
            field_mask |= FIELD_CURRENT_VALUE;
        }

        const ParamTarget* response_target = found ? &target : nullptr;
        const uint8_t response_index = found ? target.index : index;
        if((op == OP_GET_BY_INDEX || op == OP_GET_BY_NAME) && !(field_mask & (FIELD_CURRENT_VALUE | FIELD_NAME | FIELD_ACCESS_FLAGS | FIELD_PARAM_TYPE)))
        {
            field_mask |= FIELD_CURRENT_VALUE;
        }
        if(!append_param_response(response, response_len, op, result, domain_id, response_index,
                                  field_mask, response_target))
        {
            transport_error = ERROR_PAYLOAD_TOO_LONG;
            return false;
        }
        return true;
    };

    auto send_cached_response = [&](uint8_t sid, uint8_t max_len, bool keep_session)
    {
        if(5 + get_set_struct.response_len <= max_len)
        {
            send_reply(STATUS_INLINE_RSP, sid, get_set_struct.response_len, 0,
                       get_set_struct.response_buffer, get_set_struct.response_len);
            if(keep_session) get_set_struct.response_complete = 1;
            if(!keep_session) reset_session();
        }
        else
        {
            get_set_struct.current_state = 2;
            get_set_struct.response_complete = 0;
            send_reply(STATUS_RSP_READY, sid, get_set_struct.response_len & 0xFFFF,
                       (uint8_t)(get_set_struct.response_len >> 16));
        }
        get_set_struct.last_update_tick = now;
    };

    if(command == COMMAND_ABORT)
    {
        if(get_set_struct.current_state != 0 && get_set_struct.session_id != session_id)
        {
            send_error(session_id, ERROR_INVALID_SESSION);
            return;
        }
        reset_session();
        send_reply(STATUS_DONE, session_id, 0, 0);
        return;
    }

    switch(command)
    {
        case COMMAND_INLINE_REQ:
        {
            uint8_t transport_error = 0;
            get_set_struct.session_id = session_id;
            get_set_struct.flags = flags;
            if(!execute_request(rx_packet.data + 4, rx_packet.data_len - 4, flags,
                                get_set_struct.response_buffer, get_set_struct.response_len,
                                transport_error))
            {
                reset_session();
                send_error(session_id, transport_error ? transport_error : ERROR_INVALID_COMMAND);
                break;
            }
            get_set_struct.last_update_tick = now;
            send_cached_response(session_id, max_response_len, false);
            break;
        }
        case COMMAND_BEGIN_REQ:
        {
            if(rx_packet.data_len != 8)
            {
                send_error(session_id, ERROR_INVALID_COMMAND);
                break;
            }
            const uint16_t total_len = rx_packet.data[4] | ((uint16_t)rx_packet.data[5] << 8);
            const uint16_t crc16 = rx_packet.data[6] | ((uint16_t)rx_packet.data[7] << 8);
            if(total_len < 5 || total_len > sizeof(get_set_struct.request_buffer))
            {
                send_error(session_id, ERROR_PAYLOAD_TOO_LONG);
                break;
            }
            if(get_set_struct.current_state == 1 && get_set_struct.session_id == session_id)
            {
                if(get_set_struct.request_total_len == total_len && get_set_struct.request_crc16 == crc16)
                {
                    get_set_struct.last_update_tick = now;
                    send_reply(STATUS_ACK, session_id, 0, 0);
                }
                else send_error(session_id, ERROR_INVALID_SESSION);
                break;
            }
            if(get_set_struct.current_state != 0)
            {
                send_error(session_id, ERROR_INVALID_SESSION);
                break;
            }
            get_set_struct.current_state = 1;
            get_set_struct.session_id = session_id;
            get_set_struct.flags = flags;
            get_set_struct.request_total_len = total_len;
            get_set_struct.request_len = 0;
            get_set_struct.request_crc16 = crc16;
            get_set_struct.response_len = 0;
            get_set_struct.response_complete = 0;
            get_set_struct.last_update_tick = now;
            send_reply(STATUS_ACK, session_id, 0, 0);
            break;
        }
        case COMMAND_WRITE_REQ_CHUNK:
        {
            if(get_set_struct.current_state != 1 || get_set_struct.session_id != session_id)
            {
                send_error(session_id, ERROR_INVALID_SESSION);
                break;
            }
            if(rx_packet.data_len < 6)
            {
                send_error(session_id, ERROR_INVALID_COMMAND);
                break;
            }
            const uint16_t offset = rx_packet.data[4] | ((uint16_t)rx_packet.data[5] << 8);
            const uint16_t chunk_len = rx_packet.data_len - 6;
            if(offset == get_set_struct.request_len)
            {
                if(chunk_len > get_set_struct.request_total_len - get_set_struct.request_len)
                {
                    send_error(session_id, ERROR_PAYLOAD_TOO_LONG);
                    break;
                }
                memcpy(get_set_struct.request_buffer + get_set_struct.request_len, rx_packet.data + 6, chunk_len);
                get_set_struct.request_len += chunk_len;
            }
            else if(offset > get_set_struct.request_len)
            {
                send_error(session_id, ERROR_BAD_OFFSET);
                break;
            }
            get_set_struct.last_update_tick = now;
            send_reply(STATUS_ACK, session_id, get_set_struct.request_len, 0);
            break;
        }
        case COMMAND_EXEC_REQ:
        {
            if(get_set_struct.current_state == 2 && get_set_struct.session_id == session_id)
            {
                send_cached_response(session_id, max_response_len, true);
                break;
            }
            if(get_set_struct.current_state != 1 || get_set_struct.session_id != session_id)
            {
                send_error(session_id, ERROR_INVALID_SESSION);
                break;
            }
            if(rx_packet.data_len != 4)
            {
                send_error(session_id, ERROR_INVALID_COMMAND);
                break;
            }
            if(get_set_struct.request_len != get_set_struct.request_total_len)
            {
                send_error(session_id, ERROR_BAD_OFFSET);
                break;
            }
            if(get_crc16(get_set_struct.request_buffer, get_set_struct.request_len) != get_set_struct.request_crc16)
            {
                send_error(session_id, ERROR_BAD_CRC);
                break;
            }

            uint8_t transport_error = 0;
            if(!execute_request(get_set_struct.request_buffer, get_set_struct.request_len, flags,
                                get_set_struct.response_buffer, get_set_struct.response_len,
                                transport_error))
            {
                reset_session();
                send_error(session_id, transport_error ? transport_error : ERROR_INVALID_COMMAND);
                break;
            }
            get_set_struct.current_state = 2;
            get_set_struct.flags = flags;
            get_set_struct.response_complete = 0;
            get_set_struct.last_update_tick = now;
            send_cached_response(session_id, max_response_len, true);
            break;
        }
        case COMMAND_READ_RSP_CHUNK:
        {
            if(get_set_struct.current_state != 2 || get_set_struct.session_id != session_id)
            {
                send_error(session_id, ERROR_INVALID_SESSION);
                break;
            }
            if(rx_packet.data_len != 7)
            {
                send_error(session_id, ERROR_INVALID_COMMAND);
                break;
            }
            const uint16_t offset = rx_packet.data[4] | ((uint16_t)rx_packet.data[5] << 8);
            const uint8_t max_len = rx_packet.data[6];
            if(offset >= get_set_struct.response_len || max_len == 0 || max_response_len <= 7)
            {
                send_error(session_id, ERROR_BAD_OFFSET);
                break;
            }
            uint16_t chunk_len = get_set_struct.response_len - offset;
            chunk_len = min_u16(chunk_len, max_len);
            chunk_len = min_u16(chunk_len, max_response_len - 7);
            if((uint16_t)(7 + chunk_len) > (uint16_t)sizeof(tx_packet.data))
            {
                send_error(session_id, ERROR_PAYLOAD_TOO_LONG);
                break;
            }

            tx_packet.info.bit.id = node_id;
            tx_packet.info.bit.frame_id = 4;
            tx_packet.data_len = 7 + chunk_len;
            tx_packet.data[0] = STATUS_RSP_CHUNK;
            tx_packet.data[1] = session_id;
            tx_packet.data[2] = (uint8_t)(offset & 0xFF);
            tx_packet.data[3] = (uint8_t)(offset >> 8);
            tx_packet.data[4] = 0;
            tx_packet.data[5] = (uint8_t)(get_set_struct.response_len & 0xFF);
            tx_packet.data[6] = (uint8_t)(get_set_struct.response_len >> 8);
            memcpy(&tx_packet.data[7], get_set_struct.response_buffer + offset, chunk_len);
            WriteTxPacket();

            if(offset + chunk_len >= get_set_struct.response_len) get_set_struct.response_complete = 1;
            get_set_struct.last_update_tick = now;
            break;
        }
        default:
        {
            send_error(session_id, ERROR_INVALID_COMMAND);
            break;
        }
    }
}

void RS485Protocol::HandlePosControl()
{
    if(rx_packet.data_len != 14) return;
    // Send Feedback first, to ensure real-time capability
    if(xTaskGetTickCount() - control_frame_last_misc_send_tick >= MISC_FEEDBACK_SEND_INTERVAL_TICKS)
    {
        SendMiscFeedback();
        control_frame_last_misc_send_tick = xTaskGetTickCount();
    }
    else SendRTFeedback();
    if(ParseMotionHeader(rx_packet.data[0]))
    {
        control_struct.command = control_struct.POSITION;
        control_struct.is_relative = rx_packet.data[1] & 0x01;
        if(control_struct.is_relative) control_struct.is_relative_curr_based = (rx_packet.data[1] & 0x02) >> 1;
        control_struct.is_trajectory = (rx_packet.data[1] & 0x04) >> 2;
        if(control_struct.is_trajectory) control_struct.is_trajectory_s_curve = (rx_packet.data[1] & 0x08) >> 3;
        float position = 0.0f;
        memcpy(&position, &rx_packet.data[2], 4);
        const float16 velocity_ff = float16::from_bits((uint16_t)rx_packet.data[6] | (uint16_t)rx_packet.data[7] << 8);
        const float16 velocity_limit = float16::from_bits((uint16_t)rx_packet.data[8] | (uint16_t)rx_packet.data[9] << 8);
        const float16 torque_ff = float16::from_bits((uint16_t)rx_packet.data[10] | (uint16_t)rx_packet.data[11] << 8);
        const float16 torque_limit = float16::from_bits((uint16_t)rx_packet.data[12] | (uint16_t)rx_packet.data[13] << 8);
        control_struct.motion.torque.value = torque_ff.to_float();
        control_struct.motion.torque.limit = torque_limit.to_float();
        control_struct.motion.speed.value = velocity_ff.to_float();
        control_struct.motion.speed.limit = velocity_limit.to_float();
        control_struct.motion.pos.value = position;
        ProcessControlStruct(false);
    }
}

void RS485Protocol::HandleVelTorqueControl()
{
    if(rx_packet.data_len != 10) return;
    // Send Feedback first, to ensure real-time capability
    if(xTaskGetTickCount() - control_frame_last_misc_send_tick >= MISC_FEEDBACK_SEND_INTERVAL_TICKS)
    {
        SendMiscFeedback();
        control_frame_last_misc_send_tick = xTaskGetTickCount();
    }
    else SendRTFeedback();
    // Parse rx packet
    if(ParseMotionHeader(rx_packet.data[0]))
    {
        if(rx_packet.data[1] == 0)
        {
            control_struct.command = control_struct.TORQUE;
            const float16 torque_ff = float16::from_bits((uint16_t)rx_packet.data[6] | (uint16_t)rx_packet.data[7] << 8);
            control_struct.motion.torque.value = torque_ff.to_float();
        }
        else if(rx_packet.data[1] == 1)
        {
            control_struct.command = control_struct.VELOCITY;
            float velocity = 0.0f;
            memcpy(&velocity, &rx_packet.data[2], 4);
            const float16 torque_ff = float16::from_bits((uint16_t)rx_packet.data[6] | (uint16_t)rx_packet.data[7] << 8);
            const float16 torque_limit = float16::from_bits((uint16_t)rx_packet.data[8] | (uint16_t)rx_packet.data[9] << 8);
            control_struct.motion.torque.limit = torque_limit.to_float();
            control_struct.motion.torque.value = torque_ff.to_float();
            control_struct.motion.speed.value = velocity;
        }
        else return;
        ProcessControlStruct(false);
    }
}

void RS485Protocol::HandleFileDownload()
{
    if(rx_packet.data_len < 3) return;
    const uint8_t command = rx_packet.data[0];
    const uint16_t session_id = rx_packet.data[1] | ((uint32_t)rx_packet.data[2] << 8);
    if(command > 3) return;
    // For safety, file downloading is not allowed except IDLE state.
    if(GetMotor<FOCMotor>()->GetCurrentState() != MotorState::IDLE)
    {
        SendFileDownloadReply(2, 0x07);
        return;
    }
    if(command != 0 && file_dl_struct.current_state > 0 && file_dl_struct.session_id != session_id) // session id not matched (error 0x05)
    {
        SendFileDownloadReply(2, 0x05);
        return;
    }
    switch(command)
    {
        case 0: // BEGIN
        {
            if(rx_packet.data_len != 8) break;
            // just override file_dl_struct
            if(file_dl_struct.current_state != 1)
            {
                file_dl_struct.current_state = 1;
                file_dl_struct.target = rx_packet.data[3];
                file_dl_struct.session_id = session_id;
                file_dl_struct.file_size = rx_packet.data[4] | (uint32_t)rx_packet.data[5] << 8 | (uint32_t)rx_packet.data[6] << 16 | (uint32_t)rx_packet.data[7] << 24;
                file_dl_struct.current_file_offset = 0;
                // Send BUSY at first time
                SendFileDownloadReply(0x00, 0x00); // BUSY
            }
            else
            {
                // here we should go to bootloader
                BootloaderMsg msg{};
                msg.server_node_id = node_id; // reuse server_node_id as node_id, to speed up discovery process
                HAL::Bootloader::JumpToBL(msg);
                // no jump? bootloader not presented, send error
                SendFileDownloadReply(0x02, 0x06); // ERROR, TARGET NOT SUPPORTED
            }

            // send reply
            // SendFileDownloadReply(0x01, 0x00); // ACK?
            break;
        }
        case 1: // DATA
        {
            if(rx_packet.data_len < 7) break;
            const uint32_t file_offset = rx_packet.data[3] | ((uint32_t)rx_packet.data[4] << 8) | ((uint32_t)rx_packet.data[5] << 16) | ((uint32_t)rx_packet.data[6] << 24);
            const uint16_t current_packet_size = rx_packet.data_len - 7;

            if(file_offset > file_dl_struct.file_size || current_packet_size > file_dl_struct.file_size - file_offset)
            {
                SendFileDownloadReply(2, 0x07);
                break;
            }

            // 正常顺序包：写入并推进 offset
            if(file_offset == file_dl_struct.current_file_offset)
            {
                file_dl_struct.current_state = 2; // DATA

                // TODO: flash: write(file_offset, rx_packet.data + 7, current_packet_size)
                // memcpy(file_dl_struct.data_buffer, rx_packet.data + 7, current_packet_size);

                file_dl_struct.current_file_offset += current_packet_size;
            }
            else if(file_offset < file_dl_struct.current_file_offset)
            {
                // 重复包/迟到包：不要重复写，只回复当前 next offset
            }
            else
            {
                // file_offset > current_file_offset，说明中间丢包或乱序，要求主机回到当前 offset
                // 这里不要写入
            }

            SendFileDownloadReply(1, file_dl_struct.current_file_offset);
            break;
        }
        case 2: // FIN
        {
            if(rx_packet.data_len != 11) break;
            uint64_t expected_crc64 = 0;
            memcpy(&expected_crc64, rx_packet.data + 3, 8);
            file_dl_struct.current_state = 3;
            // verify... (WIP)

            // reply
            tx_packet.info.bit.id = node_id;
            tx_packet.info.bit.frame_id = 10;
            tx_packet.data_len = 5;
            tx_packet.data[0] = 3; // VERIFY_OK
            tx_packet.data[1] = 0x00;
            tx_packet.data[2] = 0x00;
            tx_packet.data[3] = 0x00;
            tx_packet.data[4] = 0x00;
            WriteTxPacket();
            file_dl_struct.current_file_offset = 0;
            file_dl_struct.current_state = 0;
            file_dl_struct.session_id = 0;
            file_dl_struct.target = 0;
            file_dl_struct.file_size = 0;
            break;
        }
        case 3: // ABORT
        {
            if(rx_packet.data_len != 3) break;
            // reply
            SendFileDownloadReply(1, file_dl_struct.current_file_offset);
            file_dl_struct.current_file_offset = 0;
            file_dl_struct.current_state = 0;
            file_dl_struct.session_id = 0;
            file_dl_struct.target = 0;
            file_dl_struct.file_size = 0;
            break;
        }
        default: break;
    }
}

void RS485Protocol::HandleSerialScope()
{
    if(rx_packet.data_len == 0) return;
    const uint8_t command = rx_packet.data[0];
    if(command == 0)
    {
        if(rx_packet.data_len != 1) return;
        tx_packet.info.bit.id = node_id;
        tx_packet.info.bit.frame_id = 12;
        tx_packet.data_len = 2;
        tx_packet.data[0] = 0;
        tx_packet.data[1] = scope_channel_cnt;
        WriteTxPacket();
    }
    else if(command == 1) // Request continuous channel
    {
        if(rx_packet.data_len != 3) return;
        const uint8_t start_index = rx_packet.data[1];
        const uint8_t request_count = rx_packet.data[2];
        if(request_count == 0 || request_count >= 63 || start_index >= scope_data.max_size()) return;
        if(start_index + request_count - 1 >= scope_data.max_size()) return;
        tx_packet.info.bit.id = node_id;
        tx_packet.info.bit.frame_id = 12;
        tx_packet.data_len = 2 + 4 * request_count;
        tx_packet.data[0] = 1;
        tx_packet.data[1] = MIN(request_count, scope_channel_cnt);
        memcpy(&tx_packet.data[2], scope_data.data() + start_index, request_count * 4);
        WriteTxPacket();
    }
}

void RS485Protocol::ProcessControlStruct(const bool latch_arrived)
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

bool RS485Protocol::ParseMotionHeader(const uint8_t header)
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

void RS485Protocol::SendFileDownloadReply(uint8_t status, uint32_t info)
{
    tx_packet.info.bit.id = node_id;
    tx_packet.info.bit.frame_id = 10;
    tx_packet.data_len = 5;
    tx_packet.data[0] = status;
    memcpy(&tx_packet.data[1], &info, 4);
    WriteTxPacket();
}

void RS485Protocol::SendNodeStatus(uint8_t frame_id)
{
    const auto motor = GetMotor<FOCMotor>();
    tx_packet.info.bit.id = node_id;
    tx_packet.info.bit.frame_id = frame_id;
    tx_packet.data_len = 12;
    const uint32_t uptime_sec = HAL::GetUptimeSeconds();
    memcpy(&tx_packet.data[0], &uptime_sec, 4);
    tx_packet.data[4] = count_bits(motor->GetError()); // HEALTH
    tx_packet.data[5] = 0; // MODE
    tx_packet.data[6] = (uint8_t)motor->GetCurrentState(); // SUB_MODE
    tx_packet.data[7] = vssc; // VSSC
    const uint32_t uuid = HAL::GetSerialNumber();
    memcpy(&tx_packet.data[8], &uuid, 4);
    WriteTxPacket();
}

void RS485Protocol::FillNodeDetailedInfo()
{
    // Performance issue
    node_info_packet.info.bit.id = node_id;
    node_info_packet.info.bit.frame_id = 2;
    const char* node_name = GetMotor<FOCMotor>()->GetConfig().node_name();
    const auto node_name_len = strnlen(node_name, 13);
    node_info_packet.data_len = 17 + node_name_len;
    uint32_t temp = HAL::GetSerialNumber();
    memcpy(&node_info_packet.data[0], &temp, 4); // UUID
    node_info_packet.data[4] = get_sw_ver_major(); // SW_VER_MAJOR
    temp = get_sw_ver_vcs();
    memcpy(&node_info_packet.data[5], &temp, 4); // SW_VER_MINOR
    const uint64_t crc = HAL::GetFirmwareCRC64();
    memcpy(&node_info_packet.data[9], &crc, 8); // SW_CRC64
    memcpy(&node_info_packet.data[17], node_name, node_name_len); // NODE_NAME
}

void RS485Protocol::SendNodeDetailedInfo()
{
    node_info_packet.header_1 = 0xAA;
    node_info_packet.header_2 = 0x54;
    node_info_packet.info.bit.id = node_id;
    node_info_packet.info.bit.frame_id = 2;
    const uint16_t crc16 = get_crc16(&node_info_packet.info.byte, node_info_packet.data_len + 2);
    uart->WriteBytes(&node_info_packet.header_1, node_info_packet.data_len + 4);
    uart->WriteBytes((const uint8_t*)&crc16, 2);
    uart->StartTransmit();
}

void RS485Protocol::FillMiscFeedback()
{
    const auto motor = GetMotor<FOCMotor>();
    constexpr float divVOLT_PER_LSB = 1.0f / (0.2f);
    constexpr float divAMPERE_PER_LSB = 1.0f / (0.2f);
    misc_fb_packet.info.bit.id = node_id;
    misc_fb_packet.info.bit.frame_id = 14;
    misc_fb_packet.data_len = 10;
    const auto dc_bus_voltage = (uint16_t)MIN(motor->GetBusSense()->voltage * divVOLT_PER_LSB, 4095);
    const auto dc_bus_current = (int16_t)_constrain(motor->GetBusSense()->current * divAMPERE_PER_LSB, -2048, 2047);
    const auto core_temp_celsius = (int16_t)_constrain(motor->GetCoreTempSense() ? motor->GetCoreTempSense()->temp_celsius : 0, -255, 254);
    const auto mosfet_temp_celsius = (int16_t)_constrain(motor->GetMosfetTempSense() ? motor->GetMosfetTempSense()->temp_celsius : 0, -255, 254);
    const auto motor_temp_celsius = (int16_t)_constrain(motor->GetMotorTempSense() ? motor->GetMotorTempSense()->temp_celsius : 0, -255, 254);
    const auto protocol_response_avg_us = (uint8_t)MIN(response_timer.elapsed_time_us, 31); // uint5
    const auto protocol_response_max_us = (uint8_t)MIN(response_timer.max_elapsed_time_us, 63); // uint6
    const auto rt_task_time_avg_us = (uint8_t)MIN(motor->task_times.rt_main_task.elapsed_time_us, 63); // uint6
    const auto rt_task_time_max_us = (uint8_t)MIN(motor->task_times.rt_main_task.max_elapsed_time_us, 63); // uint6
    const auto mid_task_time_max_us = (uint8_t)MIN(motor->task_times.mid_interval_task.max_elapsed_time_us, 63); // uint6

    misc_fb_packet.data[0] = dc_bus_voltage & 0xFF;
    misc_fb_packet.data[1] = (dc_bus_voltage & 0xF00) >> 8 | (dc_bus_current & 0xF) << 4;
    misc_fb_packet.data[2] = (dc_bus_current & 0xFF0) >> 4;
    misc_fb_packet.data[3] = (core_temp_celsius & 0xFF);
    misc_fb_packet.data[4] = (core_temp_celsius & 0x100) >> 8 | (mosfet_temp_celsius & 0x7F) << 1;
    misc_fb_packet.data[5] = (mosfet_temp_celsius & 0x180) >> 7 | (motor_temp_celsius & 0x3F) << 2;
    misc_fb_packet.data[6] = (motor_temp_celsius & 0x1C0) >> 6 | (protocol_response_avg_us & 0x1F) << 3;
    misc_fb_packet.data[7] = (protocol_response_max_us & 0x3F) | (rt_task_time_avg_us & 0x03) << 6;
    misc_fb_packet.data[8] = (rt_task_time_avg_us & 0x3C) >> 2 | (rt_task_time_max_us & 0x0F) << 4;
    misc_fb_packet.data[9] = (rt_task_time_max_us & 0x30) >> 4 | (mid_task_time_max_us & 0x3F) << 2;
}

void RS485Protocol::SendMiscFeedback()
{
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();

    misc_fb_packet.header_1 = 0xAA;
    misc_fb_packet.header_2 = 0x54;
    const uint16_t crc16 = get_crc16(&misc_fb_packet.info.byte, misc_fb_packet.data_len + 2);
    uart->WriteBytes(&misc_fb_packet.header_1, misc_fb_packet.data_len + 4);
    uart->WriteBytes((const uint8_t*)&crc16, 2);
    uart->StartTransmit();

    FillMiscFeedback();
}

void RS485Protocol::SendRTFeedback()
{
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    const Motion current = motor->GetCurrentMotionStruct(Motion::Ref::OUTPUT,
        Motion::TorqueUnit::NM,
        Motion::SpeedUnit::RADS,
        Motion::PosUnit::RAD);
    const auto single_round_u16 = (uint16_t)(normalize_rad(current.pos.value) * divPI2 * 65535.0f);
    tx_packet.info.bit.id = node_id;
    tx_packet.info.bit.frame_id = 15;
    tx_packet.data_len = 15;
    tx_packet.data[0] = ((uint8_t)motor->GetCurrentState() & 0xF) |
                        ((uint8_t)motor->GetControlMode() & 0x3) << 4 |
                        (((uint8_t)(motor->GetError() > 0))) << 6 |
                        (((uint8_t)motor->IsArmed())) << 7;
    memcpy(&tx_packet.data[1], &single_round_u16, 2); // OUTPUT_SINGLE_ROUND
    memcpy(&tx_packet.data[3], &current.pos.value, 4); // OUTPUT_MULTI_ROUND
    memcpy(&tx_packet.data[7], &current.speed.value, 4); // OUTPUT_VELOCITY_RAD_S
    memcpy(&tx_packet.data[11], &current.torque.value, 4); // OUTPUT_TORQUE_NM
    WriteTxPacket();
}

void RS485Protocol::WriteTxPacket()
{
    tx_packet.header_1 = 0xAA;
    tx_packet.header_2 = 0x54;
    const uint16_t crc16 = get_crc16(&tx_packet.info.byte, tx_packet.data_len + 2);
    uart->WriteBytes(&tx_packet.header_1, tx_packet.data_len + 4);
    uart->WriteBytes((const uint8_t*)&crc16, 2);
    uart->StartTransmit();
}

RS485Protocol::WorkerTask::WorkerTask(RS485Protocol* p) : Task("RS485"), parent(p)
{
    RegisterTask(TaskType::MID_TASK);
}

void RS485Protocol::WorkerTask::UpdateMid(float Ts)
{
    parent->uart->UpdateRxFIFO();
    // if(parent->uart->rx_fifo.move_to(parent->uart->tx_fifo, parent->uart->GetRxLen()) > 0)
    // {
    //     parent->uart->StartTransmit();
    // }
}
}
