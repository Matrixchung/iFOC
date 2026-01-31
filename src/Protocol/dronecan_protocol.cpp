#include "dronecan_protocol.hpp"
#include "foc_motor.hpp"

using namespace DroneCAN;

// DSDL definitions import
// DroneCAN definitions below
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/GetNodeInfo.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/GetTransportStats.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/RestartNode.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/param/GetSet.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/param/ExecuteOpcode.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/dynamic_node_id/Allocation.h"

#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/CompactFeedback.h"

#ifndef IFOC_NODE_NAME
#define IFOC_NODE_NAME ("com.ifoc.driver")
#endif

// 31(dec) -> 0x31(hex), only <= 2 digits supported
uint8_t dec_to_hex_byte(uint8_t dec)
{
    if(dec > 99) return 0;
    const uint8_t tens = dec / 10;
    const uint8_t units = dec % 10;
    return (tens << 4) | units;
}

namespace iFOC::Protocol
{
DroneCANProtocol::DroneCANProtocol(HAL::CANBase* base) : polling_task(this), can(base)
{
    canard_memory_pool = pvPortMalloc(CANARD_MEMORY_POOL_SIZE);
    isr_msg_queue = xQueueCreate(ISR_MSG_QUEUE_SIZE, sizeof(DataType::Comm::CANMessage));
}

DroneCANProtocol::~DroneCANProtocol()
{
    polling_task.Stop();
    vQueueDelete(isr_msg_queue);
    vPortFree(canard_memory_pool);
}

void DroneCANProtocol::Init()
{
    const auto motor = GetMotor<FOCMotor>();
    if(!canard_memory_pool) return; // pvPortMalloc() failed
    canardInit(&canard,
               canard_memory_pool,
               CANARD_MEMORY_POOL_SIZE,
               std::bind(&DroneCANProtocol::ProcessTransfer, this, std::placeholders::_1, std::placeholders::_2),
               std::bind(&DroneCANProtocol::ShouldAccept, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
               nullptr);
    auto target_node_id = motor->GetConfig().node_id();
    if(target_node_id > CANARD_MAX_NODE_ID)
    {
        target_node_id = CANARD_BROADCAST_NODE_ID;
        motor->GetConfig().set_node_id(target_node_id);
    }
    canard.node_id = target_node_id;
    can->SetHWFilter(motor->GetInternalID(), 0, 0, true, false); // accept ext only
    polling_task.Start();
    can->RegisterRxHandler(std::bind(&DroneCANProtocol::OnRxEvent, this, std::placeholders::_1));
}

DroneCANProtocol::PollingTask::PollingTask(DroneCANProtocol* p) : Task("DroneCAN"), parent(p)
{
    RegisterTask(TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 4;
    config.stack_depth = 1024;
}

void DroneCANProtocol::PollingTask::UpdateNormal()
{
    const auto motor = parent->GetMotor<FOCMotor>();
    const auto new_node_id = motor->GetConfig().node_id();
    if(new_node_id > CANARD_MAX_NODE_ID)
    {
        auto fallback_node_id = parent->canard.node_id;
        if(fallback_node_id > CANARD_MAX_NODE_ID)
        {
            fallback_node_id = CANARD_BROADCAST_NODE_ID;
            parent->canard.node_id = fallback_node_id;
        }
        motor->GetConfig().set_node_id(fallback_node_id);
    }
    else if(new_node_id != parent->canard.node_id)
    {
        parent->canard.node_id = new_node_id;
    }
    // #1: Response received transfer first
    DataType::Comm::CANMessage message{};
    if(xQueueReceive(parent->isr_msg_queue, &message, 1) == pdTRUE)
    {
        CanardCANFrame frame{};
        frame.id = message.cob_id;
        memcpy(frame.data, message.data, message.len);
        frame.data_len = message.len;
        frame.iface_id = 0;
        const auto ret = canardHandleRxFrame(&parent->canard, &frame, xTaskGetTickCount() * 1000);
        if(ret == CANARD_OK)
        {
            parent->rx_frame_received++;
        }
        else if(ret != -CANARD_ERROR_RX_INCOMPATIBLE_PACKET && ret != -CANARD_ERROR_RX_WRONG_ADDRESS && ret != -CANARD_ERROR_RX_NOT_WANTED)
        {
            parent->rx_frame_error++;
        }
    }
    // #2: Periodically tasks here
    const bool anonymous = parent->canard.node_id == CANARD_BROADCAST_NODE_ID || parent->canard.node_id > CANARD_MAX_NODE_ID;
    if(!anonymous)
    {
        auto interval_ms = motor->GetConfig().can_heartbeat_interval_ms();
        if(interval_ms > 0)
        {
            interval_ms = _constrain(interval_ms,
                                     UAVCAN_PROTOCOL_NODESTATUS_MIN_BROADCASTING_PERIOD_MS,
                                     UAVCAN_PROTOCOL_NODESTATUS_MAX_BROADCASTING_PERIOD_MS);
            if((xTaskGetTickCount() - last_send_tick.heartbeat) >= interval_ms)
            {
                last_send_tick.heartbeat = xTaskGetTickCount();
                canardCleanupStaleTransfers(&parent->canard, xTaskGetTickCount() * 1000);
                parent->SendNodeStatus();
            }
        }
        interval_ms = motor->GetConfig().can_feedback_interval_ms();
        if(interval_ms > 0)
        {
            interval_ms = _constrain(interval_ms,
                                     IFOC_COMPACTFEEDBACK_MIN_BROADCASTING_PERIOD_MS,
                                     IFOC_COMPACTFEEDBACK_MAX_BROADCASTING_PERIOD_MS);
            if((xTaskGetTickCount() - last_send_tick.feedback) >= interval_ms)
            {
                last_send_tick.feedback = xTaskGetTickCount();
                parent->SendFOCCompactFeedback();
            }
        }
    }
    else // waiting for Dynamic Node-ID Allocation (DNA)
    {
        if(xTaskGetTickCount() > parent->dna.next_dna_request_tick)
        {
            parent->RequestDNAAllocation();
        }
    }
    // #3: Generate Tx packets
    // Continuously write till FuncRetCode::BUFFER_FULL, to get maximum throughput
    DataType::Comm::CANMessage tx_message{};
    tx_message.is_ext = true;
    tx_message.is_rtr = false;
    for(const CanardCANFrame* tx_frame = nullptr; (tx_frame = canardPeekTxQueue(&parent->canard)) != nullptr; ) // multi frame approach
    // if(const CanardCANFrame* tx_frame = canardPeekTxQueue(&parent->canard); tx_frame) // single frame approach
    {
        // DataType::Comm::CANMessage tx_message
        // {
        //     .cob_id = tx_frame->id,
        //     .is_ext = true,
        //     .is_rtr = false,
        //     .len = (uint8_t)_constrain(tx_frame->data_len, 0, sizeof(DataType::Comm::CANMessage::data))
        // };
        tx_message.cob_id = tx_frame->id;
        tx_message.len = _constrain(tx_frame->data_len, 0, sizeof(DataType::Comm::CANMessage::data));
        memcpy(tx_message.data, tx_frame->data, tx_message.len);
        const auto ret = parent->can->TransmitMessage(tx_message);
        if(ret == FuncRetCode::OK)
        {
            canardPopTxQueue(&parent->canard);
            parent->tx_frame_sent++;
            continue;
        }
        break;
        // if(const auto ret = parent->can->TransmitMessage(tx_message); ret != FuncRetCode::REMOTE_TIMEOUT) // timeout, retry
        // {
        //     canardPopTxQueue(&parent->canard);
        //     if(ret != FuncRetCode::OK) parent->tx_frame_failed++;
        //     else parent->tx_frame_sent++;
        // }
    }
}

void DroneCANProtocol::ProcessTransfer(DroneCAN::CanardInstance* ins, DroneCAN::CanardRxTransfer* transfer)
{
    switch(transfer->transfer_type)
    {
        case CanardTransferTypeRequest:
        {
            switch(transfer->data_type_id)
            {
                case UAVCAN_PROTOCOL_GETNODEINFO_ID:
                {
                    SendGetInfoResponse(transfer);
                    break;
                }
                case UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID:
                {
                    SendGetTransportStatsResponse(transfer);
                    break;
                }
                case UAVCAN_PROTOCOL_RESTARTNODE_ID:
                {
                    SendRestartNodeResponse(transfer);
                    break;
                }
                case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
                {
                    SendParamGetSetResponse(transfer);
                    break;
                }
                case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
                {
                    SendExecuteOpcodeResponse(transfer);
                    break;
                }
                default: break;
            }
            break;
        }
        case CanardTransferTypeResponse:
        {
            break;
        }
        case CanardTransferTypeBroadcast:
        {
            switch(transfer->data_type_id)
            {
                case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
                {
                    OnDNAAllocation(transfer);
                    break;
                }
                default: break;
            }
            break;
        }
        default: break;
    }
}

bool DroneCANProtocol::ShouldAccept(const DroneCAN::CanardInstance* ins, uint64_t* out,
    uint16_t data_type_id, DroneCAN::CanardTransferType transfer_type, uint8_t source_node_id)
{
    switch(transfer_type)
    {
        case CanardTransferTypeRequest:
        {
            switch(data_type_id)
            {
                case UAVCAN_PROTOCOL_GETNODEINFO_ID:
                {
                    *out = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
                    return true;
                }
                case UAVCAN_PROTOCOL_GETTRANSPORTSTATS_ID:
                {
                    *out = UAVCAN_PROTOCOL_GETTRANSPORTSTATS_REQUEST_SIGNATURE;
                    return true;
                }
                case UAVCAN_PROTOCOL_RESTARTNODE_ID:
                {
                    *out = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE;
                    return true;
                }
                case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
                {
                    *out = UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_SIGNATURE;
                    return true;
                }
                case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
                {
                    *out = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_SIGNATURE;
                    return true;
                }
                default: break;
            }
            break;
        }
        case CanardTransferTypeResponse:
        {
            break;
        }
        case CanardTransferTypeBroadcast:
        {
            switch(data_type_id)
            {
                case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
                {
                    *out = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
                    return true;
                }
                default: break;
            }
            break;
        }
        default: break;
    }
    return false;
}

uavcan_protocol_NodeStatus DroneCANProtocol::BuildNodeStatus()
{
    const auto motor = GetMotor<FOCMotor>();
    const auto error = motor->GetError();
    const auto error_count = count_bits(error);
    uavcan_protocol_NodeStatus status
    {
        .uptime_sec = HAL::GetUptimeSeconds(),
        .health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
        .mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
        .sub_mode = 0,
        .vendor_specific_status_code = error_count
    };
    if(error_count == 1) status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING;
    else if(error_count > 1) status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
    return status;
}

void DroneCANProtocol::SendNodeStatus()
{
    auto status = BuildNodeStatus();
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    const uint32_t len = uavcan_protocol_NodeStatus_encode(&status, buffer);

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &next_transfer_id.uavcan_protocol_nodestatus,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

void DroneCANProtocol::SendFOCCompactFeedback()
{
    const auto motor = GetMotor<FOCMotor>();
    const auto error = motor->GetError();
    Motion current{};
    motor->GetCurrentMotion(current,
        Motion::Ref::OUTPUT,
        Motion::TorqueUnit::AMP,
        Motion::SpeedUnit::RPM,
        Motion::PosUnit::RAD);
    const uint16_t single_round_u16 = (uint16_t)(normalize_rad(current.pos.value) * divPI2 * 65535.0f);
    const float velocity_full_range = IFOC_COMPACTFEEDBACK_RPM_AMP_RANGE_RATIO * motor->GetConfig().max_output_speed_rpm();
    const int16_t velocity_rpm_i16 = (int16_t)(current.speed.value / velocity_full_range * 32767.0f);
    const float current_full_range = IFOC_COMPACTFEEDBACK_RPM_AMP_RANGE_RATIO * motor->GetConfig().max_current();
    const int16_t current_amp_i16 = (int16_t)(current.torque.value / current_full_range * 32767.0f);
    ifoc_CompactFeedback feedback
    {
        .state = (uint8_t)motor->GetCurrentState(),
        .control_mode = (uint8_t)motor->GetControlMode(),
        .has_error = error > 0,
        .is_armed = motor->IsArmed(),
        .output_single_round = single_round_u16,
        .output_velocity_rpm = velocity_rpm_i16,
        .phase_q_current_amp = current_amp_i16,
    };
    uint8_t buffer[IFOC_COMPACTFEEDBACK_MAX_SIZE];
    const uint16_t len = ifoc_CompactFeedback_encode(&feedback, buffer);

    canardBroadcast(&canard,
                    IFOC_COMPACTFEEDBACK_SIGNATURE,
                    IFOC_COMPACTFEEDBACK_ID,
                    &next_transfer_id.ifoc_compact_feedback,
                    CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buffer,
                    len);
}

void DroneCANProtocol::SendGetInfoResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_GetNodeInfoResponse response
    {
        .status = BuildNodeStatus(),
        .software_version =
        {
            .major = (uint8_t)YEAR(),
            .minor = (uint8_t)MONTH(),
            .optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT |
                                    UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC,
            .vcs_commit = 0,
            .image_crc = HAL::GetFirmwareCRC64(),
        },
        .hardware_version =
        {
            .major = 1,
            .minor = 0,
            .unique_id = {},
            .certificate_of_authenticity = {}
        },
        .name = {}
    };
    const auto serial_number = HAL::GetSerialNumber();
    memcpy(response.hardware_version.unique_id, &serial_number, sizeof(serial_number));
    response.name.len = strlen(IFOC_NODE_NAME);
    memcpy(response.name.data, IFOC_NODE_NAME, response.name.len);
    // construct compile time: MM DD hh mm, to vcs_commit (in Hexadecimal)
    response.software_version.vcs_commit = (uint32_t)dec_to_hex_byte(MONTH()) << 24 |
                                           (uint32_t)dec_to_hex_byte(DAY()) << 16 |
                                           (uint32_t)dec_to_hex_byte(HOUR()) << 8 |
                                           (uint32_t)dec_to_hex_byte(MINUTE()) << 0;

    auto* buffer = (uint8_t*)pvPortMalloc(UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE);
    if(!buffer) return;
    const uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&response, buffer);

    canardReleaseRxTransferPayload(&canard, transfer);
    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);

    vPortFree(buffer);
}

void DroneCANProtocol::SendGetTransportStatsResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_GetTransportStatsResponse response
    {
        .transfers_tx = tx_frame_sent,
        .transfers_rx = rx_frame_received,
        .transfer_errors = rx_frame_error,
        .can_iface_stats = {.len = 1, .data = {}}
    };
    response.can_iface_stats.data[0] =
    {
        .frames_tx = tx_frame_sent,
        .frames_rx = rx_frame_received,
        .errors = rx_frame_error,
    };
    auto* buffer = (uint8_t*)pvPortMalloc(UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_MAX_SIZE);
    if(!buffer) return;
    const uint32_t len = uavcan_protocol_GetTransportStatsResponse_encode(&response, buffer);

    canardReleaseRxTransferPayload(&canard, transfer);
    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_SIGNATURE,
                           UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);

    vPortFree(buffer);
}

void DroneCANProtocol::SendParamGetSetResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_param_GetSetRequest request{};
    if(uavcan_protocol_param_GetSetRequest_decode(transfer, &request)) return;
    uavcan_protocol_param_GetSetResponse response{};
    // #1: WRITE & READ, #2: READ ONLY
    // We can iterate through either name or index.
    // We will first check for request.name.
    MemberInfo info{};
    uint8_t* target_ptr = nullptr;
    char target_name[sizeof(uavcan_protocol_param_GetSetRequest::name.data) + 2 + 6]{};
    if(request.name.len > 0)
    {
        char buffer[sizeof(uavcan_protocol_param_GetSetRequest::name.data) + 2]{};
        if(request.name.len < sizeof(buffer))
        {
            memcpy(buffer, request.name.data, request.name.len);
            buffer[request.name.len] = '\0';
            auto original_len = strlen(buffer);
            if(original_len > 7)
            {
                const ReflectMap* reflect = nullptr;
                uint8_t* start_ptr = nullptr;
                if(strncmp(buffer, "board.", 6) == 0)
                {
                    reflect = &BoardConfig().GetConfig().GetReflectMap();
                    start_ptr = (uint8_t*)(&BoardConfig().GetConfig());
                }
                else if(strncmp(buffer, "motor.", 6) == 0)
                {
                    reflect = &(GetMotor<FOCMotor>()->GetConfig().GetReflectMap());
                    start_ptr = (uint8_t*)&(GetMotor<FOCMotor>()->GetConfig());
                }
                if(reflect && start_ptr)
                {
                    memcpy(target_name, buffer, original_len);
                    // trim target string
                    original_len -= 6;
                    memmove(buffer, buffer + 6, original_len + 1);

                    // add underscore
                    if(buffer[original_len - 1] != '_')
                    {
                        buffer[original_len] = '_';
                        original_len++;
                    }
                    buffer[original_len] = '\0';

                    if(const auto& it = reflect->find(buffer); it != reflect->end())
                    {
                        info = it->second;
                        target_ptr = start_ptr + info.second;
                    }
                }
            }
        }
    }
    else // Then we will search for index.
    {
        const auto index = request.index;
        const auto& board_map = BoardConfig().GetConfig().GetReflectMap();
        const auto& motor_map = GetMotor<FOCMotor>()->GetConfig().GetReflectMap();
        const uint16_t total_registers = board_map.size() + motor_map.size();
        // index: [0 - total_registers - 1]
        if(index < total_registers)
        {
            uint16_t iter_index = 0;
            if(index > board_map.size()) iter_index += board_map.size();
            else
            {
                for(const auto& [n, i] : board_map)
                {
                    if(iter_index == index)
                    {
                        snprintf(target_name, sizeof(target_name), "board.%s", n);
                        info = i;
                        target_ptr = (uint8_t*)(&BoardConfig().GetConfig()) + info.second;
                        break;
                    }
                    iter_index++;
                }
            }
            if(!target_ptr) // still can't find
            {
                for(const auto& [n, i] : motor_map)
                {
                    if(iter_index == index)
                    {
                        snprintf(target_name, sizeof(target_name), "motor.%s", n);
                        info = i;
                        target_ptr = (uint8_t*)&(GetMotor<FOCMotor>()->GetConfig()) + info.second;
                        break;
                    }
                    iter_index++;
                }
            }
        }
    }
    // here we got both info, target_ptr, and target_name.
    const auto target_name_len = strlen(target_name);
    if(target_ptr && target_name_len > 1)
    {
        // param exists, deciding WRITE + READ / READ ONLY.
        // WRITE
        switch(request.value.union_tag)
        {
            case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            {
                // int64_t, suitable for: INT32, INT64, UINT32, UINT64(possibly overflow)
                int64_t temp = request.value.integer_value;
                switch(info.first)
                {
                    case Reflection::ProtoFieldType::INT32:
                    {
                        *(int32_t*)target_ptr = (int32_t)temp;
                        break;
                    }
                    case Reflection::ProtoFieldType::INT64:
                    {
                        *(int64_t*)target_ptr = (int64_t)temp;
                        break;
                    }
                    case Reflection::ProtoFieldType::UINT32:
                    {
                        *(uint32_t*)target_ptr = (uint32_t)temp;
                        break;
                    }
                    case Reflection::ProtoFieldType::UINT64:
                    {
                        *(uint64_t*)target_ptr = (uint64_t)temp;
                        break;
                    }
                    default: break;
                }
                break;
            }
            case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            {
                // float, suitable for: FLOAT, DOUBLE
                float temp = request.value.real_value;
                switch(info.first)
                {
                    case Reflection::ProtoFieldType::FLOAT:
                    {
                        *(float*)target_ptr = (float)temp;
                        break;
                    }
                    // currently we don't want double floating point
                    // case Reflection::ProtoFieldType::DOUBLE:
                    // {
                    //     *(double*)target_ptr = (double)temp;
                    //     break;
                    // }
                    default: break;
                }
                break;
            }
            case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
            {
                // uint8_t, suitable for: BOOL
                uint8_t temp = request.value.boolean_value;
                if(info.first == Reflection::ProtoFieldType::BOOL) *(uint8_t*)target_ptr = (uint8_t)temp;
                break;
            }
            default: break;
        }
        // READ (response)
        switch(info.first)
        {
            case Reflection::ProtoFieldType::FLOAT:
            {
                response.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
                memcpy(&response.value.real_value, target_ptr, sizeof(float));
                break;
            }
            case Reflection::ProtoFieldType::INT32:
            case Reflection::ProtoFieldType::UINT32:
            {
                response.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                memcpy(&response.value.integer_value, target_ptr, sizeof(int32_t));
                break;
            }
            case Reflection::ProtoFieldType::INT64:
            case Reflection::ProtoFieldType::UINT64:
            {
                response.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                memcpy(&response.value.integer_value, target_ptr, sizeof(int64_t));
                break;
            }
            case Reflection::ProtoFieldType::BOOL:
            {
                response.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
                memcpy(&response.value.boolean_value, target_ptr, sizeof(uint8_t));
                break;
            }
            default:
            {
                return; // idk if this is correct
            }
        }
        response.name.len = target_name_len - 1; // we dont want underscore
        memcpy(response.name.data, target_name, response.name.len);
    }

    auto* buffer = (uint8_t*)pvPortMalloc(UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE);
    if(!buffer) return;
    const uint32_t len = uavcan_protocol_param_GetSetResponse_encode(&response, buffer);

    canardReleaseRxTransferPayload(&canard, transfer);
    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);

    vPortFree(buffer);
}

void DroneCANProtocol::SendRestartNodeResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_RestartNodeRequest request{};
    if(uavcan_protocol_RestartNodeRequest_decode(transfer, &request)) return;
    const bool is_restart_valid = request.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;

    if(is_restart_valid) HAL::SystemReboot(); // reboot immediately.

    uavcan_protocol_RestartNodeResponse response {.ok = is_restart_valid};

    auto* buffer = (uint8_t*)pvPortMalloc(UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE);
    if(!buffer) return;
    const uint32_t len = uavcan_protocol_RestartNodeResponse_encode(&response, buffer);

    canardReleaseRxTransferPayload(&canard, transfer);
    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_SIGNATURE,
                           UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);

    vPortFree(buffer);

    // if(is_restart_valid) // Send CAN frame immediately, then restart.
    // {
    //     DataType::Comm::CANMessage tx_message{};
    //     for(const CanardCANFrame* tx_frame = nullptr; (tx_frame = canardPeekTxQueue(&canard)) != nullptr; )
    //     {
    //         tx_message.cob_id = tx_frame->id;
    //         tx_message.len = _constrain(tx_frame->data_len, 0, sizeof(DataType::Comm::CANMessage::data));
    //         memcpy(tx_message.data, tx_frame->data, tx_message.len);
    //         auto ret = can->TransmitMessage(tx_message);
    //         if(ret == FuncRetCode::OK)
    //         {
    //             canardPopTxQueue(&canard);
    //             tx_frame_sent++;
    //             continue;
    //         }
    //         const auto start_tick = xTaskGetTickCount();
    //         bool exit = false;
    //         while(ret != FuncRetCode::OK)
    //         {
    //             ret = can->TransmitMessage(tx_message);
    //             if(xTaskGetTickCount() - start_tick >= 10)
    //             {
    //                 exit = true;
    //                 break;
    //             }
    //         }
    //         if(exit) break;
    //     }
    //     HAL::DelayMs(2);
    //     HAL::SystemReboot();
    // }
}

void DroneCANProtocol::SendExecuteOpcodeResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_param_ExecuteOpcodeRequest request{};
    if(uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &request)) return;
    uavcan_protocol_param_ExecuteOpcodeResponse response{};
    auto* buffer = (uint8_t*)pvPortMalloc(UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE);
    if(!buffer) return;
    const auto motor = GetMotor<FOCMotor>();
    switch(request.opcode)
    {
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE: // only erase motor config
        {
            if(motor->GetCurrentState() != MotorState::IDLE)
            {
                response.argument = 1; // state not in IDLE
                response.ok = false;   // erase
                break;
            }
            // auto ret = motor->config.ClearNVMConfig();
            motor->ResetDefaultConfig(); // whether success or not, we should always reset default config.
            // if(ret != FuncRetCode::OK)
            // {
            //     response.argument = 3; // motor config erase failed
            //     response.ok = false;
            //     break;
            // }
            response.argument = 0;
            response.ok = true;
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE: // save all config
        {
            if(motor->GetCurrentState() != MotorState::IDLE)
            {
                response.argument = 1; // state not in IDLE
                response.ok = false;   // save failed
                break;
            }
            // save board config first
            auto ret = BoardConfig().SaveNVMConfig();
            if(ret != FuncRetCode::OK)
            {
                response.argument = 2; // board config save failed
                response.ok = false;   // save failed
                break;
            }
            ret = motor->config.SaveNVMConfig();
            if(ret != FuncRetCode::OK)
            {
                response.argument = 3; // motor config save failed
                response.ok = false;
                break;
            }
            response.argument = 0;
            response.ok = true;
            break;
        }
        default: break;
    }

    const uint32_t len = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&response, buffer);

    canardReleaseRxTransferPayload(&canard, transfer);
    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           buffer,
                           len);
    vPortFree(buffer);
}

void DroneCANProtocol::RequestDNAAllocation()
{
    const auto current_tick = xTaskGetTickCount();
    srand(current_tick); // update random seed
    dna.next_dna_request_tick = current_tick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
                                (rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
#if CANARD_ENABLE_CANFD
    uint8_t allocation_request[CANARD_CANFD_FRAME_MAX_DATA_LEN - 1]{};
#else
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1]{};
#endif

    allocation_request[0] = (1 << 1U); // preferred_node_id = 1

    if(dna.id_allocation_uuid_offset == 0) allocation_request[0] |= 1; // first part of uuid

    uint8_t uuid_buffer[sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data)]{};
    const auto serial_number = HAL::GetSerialNumber();
    memcpy(uuid_buffer, &serial_number, sizeof(serial_number));

    uint8_t uid_size = (uint8_t)(sizeof(uuid_buffer) - dna.id_allocation_uuid_offset);

#if !CANARD_ENABLE_CANFD
    static constexpr uint8_t MAX_LEN_OF_UUID_IN_SINGLE_REQUEST = 6;
    if(uid_size > MAX_LEN_OF_UUID_IN_SINGLE_REQUEST) uid_size = MAX_LEN_OF_UUID_IN_SINGLE_REQUEST;
#endif

    memmove(&allocation_request[1], &uuid_buffer[dna.id_allocation_uuid_offset], uid_size);

    canardBroadcast(&canard,
    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
    &next_transfer_id.node_allocation,
    CANARD_TRANSFER_PRIORITY_LOW,
    &allocation_request[0],
    (uint16_t)(uid_size + 1));

    dna.id_allocation_uuid_offset = 0; // Preparing for timeout; if response is received, this value will be updated from the callback.
}

void DroneCANProtocol::OnDNAAllocation(DroneCAN::CanardRxTransfer* transfer)
{
    if(canard.node_id != CANARD_BROADCAST_NODE_ID) return; // already allocated

    // Rule C - updating the randomized time interval
    const auto current_tick = xTaskGetTickCount();
    srand(current_tick); // update random seed
    dna.next_dna_request_tick = current_tick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
                                (rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if(transfer->source_node_id == CANARD_BROADCAST_NODE_ID) // received another anonymous node's request, invalid
    {
        dna.id_allocation_uuid_offset = 0;
        return;
    }

    uavcan_protocol_dynamic_node_id_Allocation msg{};
    if(uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) return;

    uint8_t uuid_buffer[sizeof(msg.unique_id.data)]{};
    const auto serial_number = HAL::GetSerialNumber();
    memcpy(uuid_buffer, &serial_number, sizeof(serial_number));

    if(memcmp(msg.unique_id.data, uuid_buffer, sizeof(msg.unique_id.data)) != 0) // uuid mismatch
    {
        dna.id_allocation_uuid_offset = 0;
        return;
    }

    if(msg.unique_id.len < sizeof(msg.unique_id.data))
    {
        // The allocator has confirmed part of the UUID, switching to the next stage.
        dna.id_allocation_uuid_offset = msg.unique_id.len;
        dna.next_dna_request_tick -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
    }
    else
    {
        // allocation complete
        const auto motor = GetMotor<FOCMotor>();
        motor->GetConfig().set_node_id(msg.node_id);
        canard.node_id = msg.node_id;
    }
}

void DroneCANProtocol::OnRxEvent(const DataType::Comm::CANMessage& message)
{
    if(!message.is_ext || message.is_rtr) return;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(isr_msg_queue, &message, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
}
