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
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/file/BeginFirmwareUpdate.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/file/GetInfo.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/file/Delete.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/file/Read.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/file/Write.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/debug/LogMessage.h"

// iFOC custom definitions below
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/CompactFeedback.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/MiscFeedback.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/GetClearError.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/GetClearErrorIndex.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/GetOSStats.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/GetTaskStats.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/GetEncoders.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/GetCurrentMotion.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/GetTargetMotion.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetRefFrame.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetMotorState.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetControlMode.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetMITTarget.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetTrajTarget.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetPosTarget.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetVelTarget.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetTorqueTarget.h"
#include "../ThirdParty/libcanard-dronecan/dsdl/ifoc/SetDebugCmd.h"

#include <algorithm>

#ifndef IFOC_NODE_NAME
#define IFOC_NODE_NAME ("com.ifoc.driver")
#endif

#define LOG_MESSAGE_MAX_SIZE (90U)
#define DEBUG   (UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG)
#define INFO    (UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO)
#define WARNING (UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING)
#define ERROR   (UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR)

static constexpr char DRONECAN_NODE_NAME_DB_KEY_PREFIX[] = "nn";
static constexpr char DRONECAN_NODE_NAME[] = "node_name";

// Fixed a bug causing difference between dronecan_dsdlc.py compiled signature and PyDroneCAN compiled
// check the real signature using show_data_type_info.py
#define IFOC_GETOSSTATS_SIGNATURE_OVERRIDE (0x16e27da430c78667ULL)
#define IFOC_GETTASKSTATS_SIGNATURE_OVERRIDE (0xAC9B145FEDF47CA4ULL)
#define IFOC_GETCLEARERRORINDEX_SIGNATURE_OVERRIDE (0x62bb476f3e79fa2cULL)
#define IFOC_GETENCODERS_SIGNATURE_OVERRIDE (0x3057F54C149920A3ULL)
#define IFOC_GETCURRENTMOTION_SIGNATURE_OVERRIDE (0x50b27b206e23f7f2ULL)
#define IFOC_GETTARGETMOTION_SIGNATURE_OVERRIDE (0x6d9bc70fcf68b846ULL)
#define IFOC_SETREFFRAME_SIGNATURE_OVERRIDE (0x1d8a09f14b68e511ULL)
#define IFOC_SETMOTORSTATE_SIGNATURE_OVERRIDE (0x75b84757ca4943cbULL)
#define IFOC_SETCONTROLMODE_SIGNATURE_OVERRIDE (0xa7a844f501ba1097ULL)
#define IFOC_SETMITTARGET_SIGNATURE_OVERRIDE (0x2263b1bc574f3145ULL)
#define IFOC_SETTRAJTARGET_SIGNATURE_OVERRIDE (0x1b7941f7d9387e1aULL)
#define IFOC_SETPOSTARGET_SIGNATURE_OVERRIDE (0x8936246d79f9d46fULL)
#define IFOC_SETVELTARGET_SIGNATURE_OVERRIDE (0x2c78b99b3b1bd8e0ULL)
#define IFOC_SETTORQUETARGET_SIGNATURE_OVERRIDE (0x6b5d8a788d7c2b4fULL)
#define IFOC_SETDEBUGCMD_SIGNATURE_OVERRIDE (0x5aa8f22d46113408ULL)

namespace iFOC::Protocol
{
DroneCANProtocol::DroneCANProtocol(HAL::CANBase* base) : polling_task(this), can(base)
{
    canard_memory_pool = pvPortMalloc(CANARD_MEMORY_POOL_SIZE);
    // isr_msg_queue = xQueueCreate(ISR_MSG_QUEUE_SIZE, sizeof(DataType::Comm::CANMessage));
    isr_msg_fifo.init(ISR_MSG_QUEUE_SIZE);
    tx_msg_fifo.init(ISR_MSG_QUEUE_SIZE);
}

DroneCANProtocol::~DroneCANProtocol()
{
    const auto motor = GetMotor<FOCMotor>();
    motor->RemoveTaskByName("DroneCAN");
    // polling_task.Stop();
    // vQueueDelete(isr_msg_queue);
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
    // canard.node_id = target_node_id;
    SetNodeID(target_node_id);
    // read node name
    char key[sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX) + 1];
    memcpy(key, DRONECAN_NODE_NAME_DB_KEY_PREFIX, sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX) - 1);
    key[sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX) - 1] = motor->GetInternalID() + '0';
    key[sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX)] = '\0';
    uint16_t buffer_len = sizeof(node_name);
    if(BlobNVMStorage().ReadNVM(key, (uint8_t*)node_name, &buffer_len) != FuncRetCode::OK || buffer_len == 0 || buffer_len >= sizeof(node_name))
    {
        static_assert(sizeof(IFOC_NODE_NAME) < sizeof(node_name) - 1);
        const auto len = strlen(IFOC_NODE_NAME);
        memcpy(node_name, IFOC_NODE_NAME, len);
        node_name[len] = '\0';
    }
    else
    {
        node_name[buffer_len] = '\0';
    }
    BoardConfig().GetConfig().GetReflectMap(); // generate reflect map first, to avoid generate in interrupt
    motor->GetConfig().GetReflectMap();
    // polling_task.Start();
    motor->AppendTask(&polling_task);
    can->RegisterRxHandler(std::bind(&DroneCANProtocol::OnRxEvent, this, std::placeholders::_1));
}

DroneCANProtocol::PollingTask::PollingTask(DroneCANProtocol* p) : Task("DroneCAN"), parent(p)
{
    RegisterTask(TaskType::NORMAL_TASK, TaskType::MID_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 3;
    config.stack_depth = 2048;
}

void DroneCANProtocol::PollingTask::InitNormal()
{
    xLastWakeTick = xTaskGetTickCount();
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
            // parent->canard.node_id = fallback_node_id;
            parent->SetNodeID(fallback_node_id);
        }
        motor->GetConfig().set_node_id(fallback_node_id);
    }
    else if(new_node_id != parent->canard.node_id)
    {
        // parent->canard.node_id = new_node_id;
        parent->SetNodeID(new_node_id);
    }
    // #1: Response received transfer first
    DataType::Comm::CANMessage message{};
    if(parent->isr_msg_fifo.used())
    {
        // if(parent->isr_msg_fifo.get(&message, 1)) // get one single frame stored in fifo?
        while(parent->isr_msg_fifo.get(&message, 1)) // get all the frames stored in fifo?
        {
            CanardCANFrame frame;
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
    }
    // #2: Periodically tasks here
    const bool anonymous = parent->canard.node_id == CANARD_BROADCAST_NODE_ID || parent->canard.node_id > CANARD_MAX_NODE_ID;
    if(!anonymous)
    {
        // monitor here
        if(!monitor.is_init_send)
        {
            // initialize here
            monitor.last_motor_arm_state = motor->IsArmed();
            monitor.last_motor_state = motor->GetCurrentState();
            monitor.last_motor_mode = motor->GetControlMode();
            monitor.last_error = motor->GetError();
            parent->SendLogMessage(INFO, "INIT", 4);
            monitor.is_init_send = true;
        }
        else
        {
            const bool armed = motor->IsArmed();
            if(monitor.last_motor_arm_state != armed)
            {
                if(armed) parent->SendLogMessage(INFO, "ARMED", 5);
                else parent->SendLogMessage(INFO, "DISARM", 6);
                monitor.last_motor_arm_state = armed;
            }
            const uint64_t error = motor->GetError();
            if(monitor.last_error != error)
            {
                if(error > 0)
                {
                    uint8_t error_index = 0;
                    char buffer[LOG_MESSAGE_MAX_SIZE];
                    uint16_t offset = snprintf(buffer, sizeof(buffer), "ER");
                    uint64_t temp = error;
                    while(temp && offset < sizeof(buffer))
                    {
                        if(temp & 0x01)
                        {
                            const auto written = snprintf(buffer + offset, sizeof(buffer) - offset, "%u,", error_index);
                            if(written < 0) break;
                            if(written >= (int)(sizeof(buffer) - offset))
                            {
                                offset = sizeof(buffer) - 1;
                                break;
                            }
                            offset += written;
                        }
                        error_index++;
                        temp >>= 1ULL;
                    }
                    if(offset > 2) buffer[offset - 1] = '\0';
                    parent->SendLogMessage(ERROR, buffer, sizeof(buffer));
                }
                monitor.last_error = error;
            }
            const MotorState motor_state = motor->GetCurrentState();
            if(monitor.last_motor_state != motor_state)
            {
                char buffer[6];
                snprintf(buffer, sizeof(buffer), "ST%u>%u", (uint8_t)monitor.last_motor_state, (uint8_t)motor_state);
                parent->SendLogMessage(INFO, buffer, sizeof(buffer));
                monitor.last_motor_state = motor_state;
            }
            const MotorControlMode motor_mode = motor->GetControlMode();
            if(monitor.last_motor_mode != motor_mode)
            {
                char buffer[6];
                snprintf(buffer, sizeof(buffer), "MD%u>%u", (uint8_t)monitor.last_motor_mode, (uint8_t)motor_mode);
                parent->SendLogMessage(INFO, buffer, sizeof(buffer));
                monitor.last_motor_mode = motor_mode;
            }
        }
        auto interval_ms = motor->GetConfig().can_heartbeat_interval_ms();
        if(interval_ms == 0) interval_ms = UAVCAN_PROTOCOL_NODESTATUS_MAX_BROADCASTING_PERIOD_MS; // forced to send heartbeat
        // if(interval_ms > 0)
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
        interval_ms = motor->GetConfig().can_misc_fdbk_interval_ms();
        if(interval_ms > 0)
        {
            interval_ms = _constrain(interval_ms,
                                     IFOC_MISCFEEDBACK_MIN_BROADCASTING_PERIOD_MS,
                                     IFOC_MISCFEEDBACK_MAX_BROADCASTING_PERIOD_MS);
            if((xTaskGetTickCount() - last_send_tick.misc_feedback) >= interval_ms)
            {
                last_send_tick.misc_feedback = xTaskGetTickCount();
                parent->SendFOCMiscFeedback();
            }
        }
        // Cleanup file read buffer
        if(parent->file_read_buffer.last_read_tick > 0 && xTaskGetTickCount() - parent->file_read_buffer.last_read_tick >= parent->FILE_READ_TIMEOUT_MS)
        {
            if(parent->file_read_buffer.key)
            {
                vPortFree(parent->file_read_buffer.key);
                parent->file_read_buffer.key = nullptr;
            }
            parent->file_read_buffer.key_length = 0;
            if(parent->file_read_buffer.data)
            {
                vPortFree(parent->file_read_buffer.data);
                parent->file_read_buffer.data = nullptr;
            }
            parent->file_read_buffer.data_length = 0;
            parent->file_read_buffer.last_read_tick = 0;
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
    // DataType::Comm::CANMessage message{};
    message.is_ext = true;
    message.is_rtr = false;
    for(const CanardCANFrame* tx_frame = nullptr; (tx_frame = canardPeekTxQueue(&parent->canard)) != nullptr; ) // multi frame approach
        // if(const CanardCANFrame* tx_frame = canardPeekTxQueue(&parent->canard); tx_frame) // single frame approach
    {
        // message.cob_id = tx_frame->id;
        // message.len = _constrain(tx_frame->data_len, 0, sizeof(DataType::Comm::CANMessage::data));
        // memcpy(message.data, tx_frame->data, message.len);
        // const auto ret = parent->can->TransmitMessage(message);
        // if(ret == FuncRetCode::OK)
        // {
        //     canardPopTxQueue(&parent->canard);
        //     parent->tx_frame_sent++;
        //     continue;
        // }
        // break;
        if(parent->tx_msg_fifo.available()) // available
        {
            message.cob_id = tx_frame->id;
            message.len = _constrain(tx_frame->data_len, 0, sizeof(DataType::Comm::CANMessage::data));
            memcpy(message.data, tx_frame->data, message.len);
            parent->tx_msg_fifo.put(&message, 1);
            canardPopTxQueue(&parent->canard);
            continue;
        }
        break;
    }
    // sleep(1);
    vTaskDelayUntil(&xLastWakeTick, pdMS_TO_TICKS(1));
}

void DroneCANProtocol::PollingTask::UpdateMid(float Ts)
{
    if(parent->tx_msg_fifo.used())
    {
        DataType::Comm::CANMessage message;
        while(parent->tx_msg_fifo.peek(&message, 1))
        {
            const auto ret = parent->can->TransmitMessage(message);
            if(ret == FuncRetCode::OK)
            {
                parent->tx_frame_sent++;
                parent->tx_msg_fifo.wipe_n(1);
                continue;
            }
            break;
        }
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
                    SendGetNodeInfoResponse(transfer);
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
                case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
                {
                    SendFWUpdateResponse(transfer);
                    break;
                }
                case UAVCAN_PROTOCOL_FILE_GETINFO_ID:
                {
                    SendFileGetInfoResponse(transfer);
                    break;
                }
                case UAVCAN_PROTOCOL_FILE_DELETE_ID:
                {
                    SendFileDeleteResponse(transfer);
                    break;
                }
                case UAVCAN_PROTOCOL_FILE_READ_ID:
                {
                    SendFileReadResponse(transfer);
                    break;
                }
                case IFOC_GETCLEARERROR_ID:
                {
                    SendFOCGetClearErrorResponse(transfer);
                    break;
                }
                case IFOC_GETCLEARERRORINDEX_ID:
                {
                    SendFOCGetClearErrorIndexResponse(transfer);
                    break;
                }
                case IFOC_GETOSSTATS_ID:
                {
                    SendFOCGetOSStatsResponse(transfer);
                    break;
                }
                case IFOC_GETTASKSTATS_ID:
                {
                    SendFOCGetTaskStatsResponse(transfer);
                    break;
                }
                case IFOC_GETENCODERS_ID:
                {
                    SendFOCGetEncodersResponse(transfer);
                    break;
                }
                case IFOC_GETCURRENTMOTION_ID:
                {
                    SendFOCGetCurrentMotionResponse(transfer);
                    break;
                }
                case IFOC_GETTARGETMOTION_ID:
                {
                    SendFOCGetTargetMotionResponse(transfer);
                    break;
                }
                case IFOC_SETREFFRAME_ID:
                {
                    SendFOCSetRefFrameResponse(transfer);
                    break;
                }
                case IFOC_SETMOTORSTATE_ID:
                {
                    SendFOCSetMotorStateResponse(transfer);
                    break;
                }
                case IFOC_SETCONTROLMODE_ID:
                {
                    SendFOCSetControlModeResponse(transfer);
                    break;
                }
                case IFOC_SETMITTARGET_ID:
                {
                    SendFOCSetMITTargetResponse(transfer);
                    break;
                }
                case IFOC_SETTRAJTARGET_ID:
                {
                    SendFOCSetTrajTargetResponse(transfer);
                    break;
                }
                case IFOC_SETPOSTARGET_ID:
                {
                    SendFOCSetPosTargetResponse(transfer);
                    break;
                }
                case IFOC_SETVELTARGET_ID:
                {
                    SendFOCSetVelTargetResponse(transfer);
                    break;
                }
                case IFOC_SETTORQUETARGET_ID:
                {
                    SendFOCSetTorqueTargetResponse(transfer);
                    break;
                }
                case IFOC_SETDEBUGCMD_ID:
                {
                    SendFOCSetDebugCmdResponse(transfer);
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
                case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
                {
                    *out = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_REQUEST_SIGNATURE;
                    return true;
                }
                case UAVCAN_PROTOCOL_FILE_GETINFO_ID:
                {
                    *out = UAVCAN_PROTOCOL_FILE_GETINFO_REQUEST_SIGNATURE;
                    return true;
                }
                case UAVCAN_PROTOCOL_FILE_DELETE_ID:
                {
                    *out = UAVCAN_PROTOCOL_FILE_DELETE_REQUEST_SIGNATURE;
                    return true;
                }
                case UAVCAN_PROTOCOL_FILE_READ_ID:
                {
                    *out = UAVCAN_PROTOCOL_FILE_READ_REQUEST_SIGNATURE;
                    return true;
                }
                case IFOC_GETCLEARERROR_ID:
                {
                    *out = IFOC_GETCLEARERROR_REQUEST_SIGNATURE;
                    return true;
                }
                case IFOC_GETCLEARERRORINDEX_ID:
                {
                    *out = IFOC_GETCLEARERRORINDEX_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_GETOSSTATS_ID:
                {
                    // *out = IFOC_GETOSSTATS_SIGNATURE;
                    *out = IFOC_GETOSSTATS_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_GETTASKSTATS_ID:
                {
                    *out = IFOC_GETTASKSTATS_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_GETENCODERS_ID:
                {
                    *out = IFOC_GETENCODERS_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_GETCURRENTMOTION_ID:
                {
                    *out = IFOC_GETCURRENTMOTION_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_GETTARGETMOTION_ID:
                {
                    *out = IFOC_GETTARGETMOTION_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETREFFRAME_ID:
                {
                    *out = IFOC_SETREFFRAME_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETMOTORSTATE_ID:
                {
                    *out = IFOC_SETMOTORSTATE_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETCONTROLMODE_ID:
                {
                    *out = IFOC_SETCONTROLMODE_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETMITTARGET_ID:
                {
                    *out = IFOC_SETMITTARGET_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETTRAJTARGET_ID:
                {
                    *out = IFOC_SETTRAJTARGET_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETPOSTARGET_ID:
                {
                    *out = IFOC_SETPOSTARGET_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETVELTARGET_ID:
                {
                    *out = IFOC_SETVELTARGET_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETTORQUETARGET_ID:
                {
                    *out = IFOC_SETTORQUETARGET_SIGNATURE_OVERRIDE;
                    return true;
                }
                case IFOC_SETDEBUGCMD_ID:
                {
                    *out = IFOC_SETDEBUGCMD_SIGNATURE_OVERRIDE;
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

void DroneCANProtocol::_SendResponse(DroneCAN::CanardRxTransfer* transfer, const uint64_t signature, const uint8_t id,
    const void* payload, const uint16_t len)
{
    canardReleaseRxTransferPayload(&canard, transfer);
    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           signature,
                           id,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           payload,
                           len);
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
    const uint8_t len = uavcan_protocol_NodeStatus_encode(&status, buffer);

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &next_transfer_id.uavcan_protocol_nodestatus,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

void DroneCANProtocol::SendLogMessage(const uint8_t level, const char* text, const uint8_t max_buffer_len)
{
    uavcan_protocol_debug_LogMessage log_message
    {
        .level = {.value = level},
        .source = {},
        .text = {}
    };
    log_message.text.len = MIN(strnlen(text, max_buffer_len), LOG_MESSAGE_MAX_SIZE);
    memcpy(log_message.text.data, text, log_message.text.len);

    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    const uint16_t len = uavcan_protocol_debug_LogMessage_encode(&log_message, buffer);

    canardBroadcast(&canard,
        UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
        UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
        &next_transfer_id.uavcan_protocol_logmessage,
        log_message.level.value <= UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO ? CANARD_TRANSFER_PRIORITY_MEDIUM : CANARD_TRANSFER_PRIORITY_HIGH,
        buffer,
        len);
}

void DroneCANProtocol::SendFOCCompactFeedback()
{
    const auto motor = GetMotor<FOCMotor>();
    const auto error = motor->GetError();
    Motion current;
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

void DroneCANProtocol::SendFOCMiscFeedback()
{
    const auto motor = GetMotor<FOCMotor>();
    ifoc_MiscFeedback feedback
    {
        .dc_bus_voltage = (uint16_t)(motor->GetBusSense()->voltage / IFOC_MISCFEEDBACK_VOLT_PER_LSB),
        .dc_bus_current = (int16_t)(motor->GetBusSense()->current / IFOC_MISCFEEDBACK_AMPERE_PER_LSB),
        .core_temp_celsius = (int8_t)(motor->GetCoreTempSense() ? motor->GetCoreTempSense()->temp_celsius : 0),
        .mosfet_temp_celsius = (int8_t)(motor->GetMosfetTempSense() ? motor->GetMosfetTempSense()->temp_celsius : 0),
        .motor_temp_celsius = (int8_t)(motor->GetMotorTempSense() ? motor->GetMotorTempSense()->temp_celsius : 0),
        .rt_task_time_us = (uint8_t)motor->task_times.rt_main_task.elapsed_time_us,
    };
    uint8_t buffer[IFOC_MISCFEEDBACK_MAX_SIZE];
    const uint16_t len = ifoc_MiscFeedback_encode(&feedback, buffer);

    canardBroadcast(&canard,
                    IFOC_MISCFEEDBACK_SIGNATURE,
                    IFOC_MISCFEEDBACK_ID,
                    &next_transfer_id.ifoc_misc_feedback,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

void DroneCANProtocol::SendFOCGetClearErrorResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_GetClearErrorRequest request;
    if(ifoc_GetClearErrorRequest_decode(transfer, &request)) return;

    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    if(request.clear_mask) motor->ClearError(request.clear_mask);

    ifoc_GetClearErrorResponse response{};
    response.error = motor->GetError();

    uint8_t buffer[IFOC_GETCLEARERROR_RESPONSE_MAX_SIZE];
    const uint32_t len = ifoc_GetClearErrorResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_GETCLEARERROR_RESPONSE_SIGNATURE, IFOC_GETCLEARERROR_RESPONSE_ID, buffer, len);
}

void DroneCANProtocol::SendFOCGetClearErrorIndexResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_GetClearErrorIndexRequest request;
    if(ifoc_GetClearErrorIndexRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    for(auto i = 0; i < request.clear_index.len; i++)
    {
        const uint64_t temp_error = (1ULL << request.clear_index.data[i]);
        if(motor->GetError() & temp_error)
        {
            motor->ClearError(temp_error);
        }
    }
    ifoc_GetClearErrorIndexResponse response{};
    auto error = motor->GetError();
    uint8_t error_index = 0;
    while(error && response.error_index.len < sizeof(response.error_index.data))
    {
        if(error & 0x01)
        {
            response.error_index.data[response.error_index.len++] = error_index;
        }
        error_index++;
        error >>= 1LL;
    }
    uint8_t buffer[IFOC_GETCLEARERRORINDEX_RESPONSE_MAX_SIZE];
    const uint32_t len = ifoc_GetClearErrorIndexResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_GETCLEARERRORINDEX_SIGNATURE_OVERRIDE, IFOC_GETCLEARERRORINDEX_RESPONSE_ID, buffer, len);
}

void DroneCANProtocol::SendFOCGetOSStatsResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_GetOSStatsResponse response
    {
        .mem_used = (configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize()),
        .mem_total = configTOTAL_HEAP_SIZE,
        .nvm_used = BoardConfig().GetNVMUsedSize(),
        .nvm_total = BoardConfig().GetNVMTotalSize(),
        .app_version =
     {
            .major = get_sw_ver_major(),
            .minor = get_sw_ver_minor(),
            .optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT |
                                    UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC,
            .vcs_commit = get_sw_ver_vcs(),
            .image_crc = HAL::GetFirmwareCRC64(),
        },
        .bootloader_version = {},
        .tasks = {}
    };
    if(HAL::Bootloader::HasBL())
    {
        uint8_t major, minor;
        uint32_t vcs;
        HAL::Bootloader::GetBLVersion(major, minor, vcs);
        response.bootloader_version =
        {
            .major = major,
            .minor = minor,
            .optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT,
            .vcs_commit = vcs,
            .image_crc = 0
        };
    }
#if configUSE_TRACE_FACILITY == 1
    UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    uint32_t ulTotalRunTime = 0;
    TaskStatus_t *pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    if(pxTaskStatusArray)
    {
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
        ulTotalRunTime /= 100UL;
        if(ulTotalRunTime > 0)
        {
            Vector<TaskStatus_t> task_vector(pxTaskStatusArray, pxTaskStatusArray + uxArraySize);
            std::sort(task_vector.begin(), task_vector.end(), [](const TaskStatus_t &a, const TaskStatus_t &b) -> bool { return a.uxCurrentPriority > b.uxCurrentPriority; } );
            response.tasks.len = _constrain(uxArraySize, 0, sizeof(response.tasks.data));
            uint32_t ulStatsAsPct = 0;
            for(uint8_t i = 0; i < response.tasks.len; i++)
            {
                ulStatsAsPct = task_vector[i].ulRunTimeCounter / ulTotalRunTime;
                response.tasks.data[i].task_state = (uint8_t)task_vector[i].eCurrentState;
                response.tasks.data[i].priority = (uint8_t)task_vector[i].uxCurrentPriority;
                response.tasks.data[i].min_stack_remaining = (uint16_t)task_vector[i].usStackHighWaterMark;
                response.tasks.data[i].task_name.len = _constrain(strlen(task_vector[i].pcTaskName), 0, sizeof(response.tasks.data->task_name.data));
                response.tasks.data[i].run_time_pct = _constrain(ulStatsAsPct, 1, 100);
                memcpy(response.tasks.data[i].task_name.data, task_vector[i].pcTaskName, response.tasks.data[i].task_name.len);
            }
        }
    }
    vPortFree(pxTaskStatusArray);
#endif

    uint8_t buffer[IFOC_GETOSSTATS_RESPONSE_MAX_SIZE];
    const uint32_t len = ifoc_GetOSStatsResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_GETOSSTATS_SIGNATURE_OVERRIDE, IFOC_GETOSSTATS_ID, buffer, len);
}

void DroneCANProtocol::SendFOCGetTaskStatsResponse(DroneCAN::CanardRxTransfer* transfer)
{
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    ifoc_GetTaskStatsResponse response
    {
        .rt_task_time_us = (uint8_t)motor->task_times.rt_main_task.elapsed_time_us,
        .rt_to_rem_wait_time_us = (uint8_t)motor->task_times.rt_waiting_for_remaining.elapsed_time_us,
        .rem_task_time_us = (uint8_t)motor->task_times.rt_remaining_task.elapsed_time_us,
        .mid_task_time_us = (uint8_t)motor->task_times.mid_interval_task.elapsed_time_us,
        .rt_task_list = {},
        .mid_task_list = {}
    };
    static_assert(sizeof(response.rt_task_list.data) == sizeof(response.mid_task_list.data));
    const auto& task_list = motor->GetTaskProcessor().GetTaskList();
    auto it = task_list.cbegin();
    for(uint8_t i = 0; i < sizeof(response.rt_task_list.data) && it != task_list.cend(); ++i, ++it)
    {
        if((*it)->IsTaskRegistered(Task::TaskType::RT_TASK))
        {
            response.rt_task_list.data[response.rt_task_list.len].task_name.len = strnlen((*it)->GetName(), sizeof(response.rt_task_list.data->task_name.data));
            memcpy(response.rt_task_list.data[response.rt_task_list.len].task_name.data,
                (*it)->GetName(),
                response.rt_task_list.data[response.rt_task_list.len].task_name.len);
            response.rt_task_list.len++;
        }
        if((*it)->IsTaskRegistered(Task::TaskType::MID_TASK))
        {
            response.mid_task_list.data[response.mid_task_list.len].task_name.len = strnlen((*it)->GetName(), sizeof(response.rt_task_list.data->task_name.data));
            memcpy(response.mid_task_list.data[response.mid_task_list.len].task_name.data,
                (*it)->GetName(),
                response.mid_task_list.data[response.mid_task_list.len].task_name.len);
            response.mid_task_list.len++;
        }
    }
    // fix: last task name of MidTaskList not showing up properly
    if(response.mid_task_list.len < sizeof(response.mid_task_list.data)) response.mid_task_list.len++;
    uint8_t buffer[IFOC_GETTASKSTATS_RESPONSE_MAX_SIZE];
    const uint32_t len = ifoc_GetTaskStatsResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_GETTASKSTATS_SIGNATURE_OVERRIDE, IFOC_GETTASKSTATS_ID, buffer, len);
}

void DroneCANProtocol::SendFOCGetEncodersResponse(DroneCAN::CanardRxTransfer* transfer)
{
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    const auto& encoders = motor->GetEncoders();
    const uint8_t encoders_count = MIN(encoders.size(), sizeof(ifoc_GetEncodersResponse::encoders.data));
    ifoc_GetEncodersResponse response{};
    response.encoders.len = encoders_count;
    for(uint8_t i = 0; i < encoders_count; i++)
    {
        const auto& enc = encoders[i];
        auto& target = response.encoders.data[i];
        target.name.len = MIN(strlen(enc->GetName()), sizeof(ifoc_GetEncodersResponse::encoders.data->name.data));
        memcpy(target.name.data, enc->GetName(), target.name.len);
        target.type = (uint8_t)enc->GetEncoderType();
        target.primary = motor->GetPrimaryEncoderIndex() == i;
        target.result_valid = enc->IsResultValid();
        target.single_round_angle_rad = enc->compensated_single_round_angle_rad;
        target.multi_round_angle_rad = enc->multi_round_angle_rad;
        target.angular_speed_rad_s = enc->angular_speed_rad_s;
        target.full_rotations = enc->full_rotations;
    }

    uint8_t buffer[IFOC_GETENCODERS_RESPONSE_MAX_SIZE];
    const uint16_t len = ifoc_GetEncodersResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_GETENCODERS_SIGNATURE_OVERRIDE, IFOC_GETENCODERS_ID, buffer, len);
}

void DroneCANProtocol::SendFOCGetCurrentMotionResponse(DroneCAN::CanardRxTransfer* transfer)
{
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    const auto& current_motion = motor->GetCurrentMotionStruct(io_ref, io_torque_unit, io_speed_unit, io_pos_unit);
    ifoc_GetCurrentMotionResponse response
    {
        .current =
        {
            .torque = current_motion.torque.value,
            .speed = current_motion.speed.value,
            .pos = current_motion.pos.value
        }
    };

    uint8_t buffer[IFOC_GETCURRENTMOTION_RESPONSE_MAX_SIZE];
    const uint16_t len = ifoc_GetCurrentMotionResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_GETCURRENTMOTION_SIGNATURE_OVERRIDE, IFOC_GETCURRENTMOTION_ID, buffer, len);
}

void DroneCANProtocol::SendFOCGetTargetMotionResponse(DroneCAN::CanardRxTransfer* transfer)
{
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    const auto& current_motion = motor->GetTargetMotionStruct(io_ref, io_torque_unit, io_speed_unit, io_pos_unit);
    ifoc_GetTargetMotionResponse response
    {
        .target =
        {
            .torque = current_motion.torque.value,
            .speed = current_motion.speed.value,
            .pos = current_motion.pos.value
        }
    };

    uint8_t buffer[IFOC_GETTARGETMOTION_RESPONSE_MAX_SIZE];
    const uint16_t len = ifoc_GetTargetMotionResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_GETTARGETMOTION_SIGNATURE_OVERRIDE, IFOC_GETTARGETMOTION_ID, buffer, len);
}

void DroneCANProtocol::SendFOCSetRefFrameResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetRefFrameRequest request;
    if(ifoc_SetRefFrameRequest_decode(transfer, &request)) return;

    if((Motion::Ref)request.set_ref.reference != Motion::Ref::ELEC)
    {
        io_ref = (Motion::Ref)request.set_ref.reference;
        io_torque_unit = (Motion::TorqueUnit)request.set_ref.torque_unit;
        io_speed_unit = (Motion::SpeedUnit)request.set_ref.speed_unit;
        io_pos_unit = (Motion::PosUnit)request.set_ref.pos_unit;
    }

    ifoc_SetRefFrameResponse response
    {
        .get_ref =
        {
            .reference = (uint8_t)io_ref,
            .torque_unit = (bool)io_torque_unit,
            .speed_unit = (uint8_t)io_speed_unit,
            .pos_unit = (uint8_t)io_pos_unit
        }
    };

    uint8_t buffer[IFOC_SETREFFRAME_RESPONSE_MAX_SIZE];
    const uint16_t len = ifoc_SetRefFrameResponse_encode(&response, buffer);

    _SendResponse(transfer, IFOC_SETREFFRAME_SIGNATURE_OVERRIDE, IFOC_SETREFFRAME_ID, buffer, len);
}

void DroneCANProtocol::SendFOCSetMotorStateResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetMotorStateRequest request;
    if(ifoc_SetMotorStateRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    const MotorState req_state = (MotorState)request.set_state;
    ifoc_SetMotorStateResponse response
    {
        .state = (uint8_t)motor->state_machine.RequestState(req_state)
    };
    uint8_t buffer[IFOC_SETMOTORSTATE_RESPONSE_MAX_SIZE];
    const uint16_t len = ifoc_SetMotorStateResponse_encode(&response, buffer);
    _SendResponse(transfer, IFOC_SETMOTORSTATE_SIGNATURE_OVERRIDE, IFOC_SETMOTORSTATE_ID, buffer, len);
}

void DroneCANProtocol::SendFOCSetControlModeResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetControlModeRequest request;
    if(ifoc_SetControlModeRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    motor->SetControlMode((MotorControlMode)request.set_mode);
    ifoc_SetControlModeResponse response
    {
        .control_mode = (uint8_t)motor->GetControlMode()
    };
    uint8_t buffer[IFOC_SETCONTROLMODE_RESPONSE_MAX_SIZE];
    const uint16_t len = ifoc_SetControlModeResponse_encode(&response, buffer);
    _SendResponse(transfer, IFOC_SETCONTROLMODE_SIGNATURE_OVERRIDE, IFOC_SETCONTROLMODE_ID, buffer, len);
}

void DroneCANProtocol::SendFOCSetMITTargetResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetMITTargetRequest request;
    if(ifoc_SetMITTargetRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    // MIT mapping:
    // P_MAX = mit_output_pos_range_deg (OUTPUT, ABSOLUTE, DEGREE, [-P_MAX, P_MAX])
    // V_MAX = mit_output_vel_range_rpm (OUTPUT, ABSOLUTE, RPM, [-V_MAX, V_MAX])
    // T_MAX = mit_output_tor_range_nm  (OUTPUT, ABSOLUTE, NM, [-T_MAX, T_MAX])

    // To target motion: BASE, AMP, RADS, RAD
    // #1: Check parameter validity first
    if(motor->GetConfig().deduction_ratio() > 0.0f &&
        motor->GetConfig().torque_constant_valid() &&
        motor->GetConfig().torque_constant() > 0.0f &&
        motor->GetConfig().mit_output_pos_range_deg() > 0.0f &&
        motor->GetConfig().mit_output_vel_range_rpm() > 0.0f &&
        motor->GetConfig().mit_output_tor_range_nm() > 0.0f)
    {
        // #2: Check Kp/Kd(uint10,1023) validity
        if(request.kp_pu > 0 && request.kd_pu == 0) return;

        // Kp, Kd (output shaft, Nm/rad & Nm*s/rad), to base frame
        constexpr float kp_x = ((double)IFOC_SETMITTARGET_REQUEST_MIT_MAX_KP / 1023.0);
        const float modifier = 1.0f / (motor->GetConfig().deduction_ratio() * motor->GetConfig().deduction_ratio());
        const float target_Kp_Nm_per_rad = (float)request.kp_pu * kp_x * modifier;
        constexpr float kd_x = ((double)IFOC_SETMITTARGET_REQUEST_MIT_MAX_KD / 1023.0);
        const float target_Kd_Nms_per_rad = (float)request.kd_pu * kd_x * modifier;

        // #3: get pos & vel & tor in OUTPUT frame, with deg, rpm & nm
        // for position, pu limited to [-32767, +32767]
        if(request.position_pu == -32768) request.position_pu = -32767;
        const float target_pos_output_deg = ((float)request.position_pu / 32767.0f) * motor->GetConfig().mit_output_pos_range_deg();

        // for velocity, pu limited to [-511, 511]
        request.velocity_pu = _constrain(request.velocity_pu, -511, 511);
        const float target_vel_output_rpm = ((float)request.velocity_pu / 511.0f) * motor->GetConfig().mit_output_vel_range_rpm();

        // for torque, pu limited to [-511, 511]
        request.torque_pu = _constrain(request.torque_pu, -511, 511);
        const float target_tor_output_nm = ((float)request.torque_pu / 511.0f) * motor->GetConfig().mit_output_tor_range_nm();

        // #4: convert pos & vel & tor to BASE frame, with RAD, RADS & AMP
        const float target_pos_base_rad = DEG2RAD(target_pos_output_deg) * motor->GetConfig().deduction_ratio();
        const float target_vel_base_rad_s = RPM2RAD(target_vel_output_rpm, 1) * motor->GetConfig().deduction_ratio();
        const float x = 1.0f / (motor->GetConfig().deduction_ratio() * motor->GetConfig().torque_constant());
        const float target_tor_base_amp = target_tor_output_nm * x;
        // Also, we will get Iq limit.
        const float base_amp_limit = motor->GetConfig().mit_output_tor_range_nm() * x;

        // #5: construct target motion frame
        Motion target_motion
        {
            .ref = Motion::Ref::BASE,
            .torque = {target_tor_base_amp, base_amp_limit, Motion::TorqueUnit::AMP},
            .speed = {target_vel_base_rad_s, target_Kd_Nms_per_rad, Motion::SpeedUnit::RADS},
            .pos = {target_pos_base_rad, target_Kp_Nm_per_rad, Motion::PosUnit::RAD}
        };

        motor->SetControlMode(MotorControlMode::CTRL_MODE_HYBRID);
        motor->SetTargetMotion(target_motion);
    }
}

void DroneCANProtocol::SendFOCSetTrajTargetResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetTrajTargetRequest request;
    if(ifoc_SetTrajTargetRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    Motion target_motion
    {
        .ref = io_ref,
        .torque = {0.0f, 0.0f, io_torque_unit},
        .speed = {0.0f, 0.0f, io_speed_unit},
        .pos = {request.target, 0.0f, io_pos_unit}
    };
    if(request.relative)
    {
        if(request.rel_curr_based)
        {
            const auto curr_motion = motor->GetCurrentMotionStruct(target_motion);
            target_motion.pos.value += curr_motion.pos.value;
        }
        else
        {
            const auto last_target_motion = motor->GetTargetMotionStruct(target_motion);
            target_motion.pos.value += last_target_motion.pos.value;
        }
    }
    motor->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
    motor->SetTrajectoryTargetMotion(target_motion, request.s_curve);
}

void DroneCANProtocol::SendFOCSetPosTargetResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetPosTargetRequest request;
    if(ifoc_SetPosTargetRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    if(motor->GetConfig().deduction_ratio() <= 0.0f) return;
    // int10: [-512,511]
    static constexpr float _1_div_512 = 1.0f / 512.0f;
    const float real_velocity_output_rpm = (float)request.velocity_pu * _1_div_512 * motor->GetConfig().mit_output_vel_range_rpm();
    const float real_torque_output_nm = (float)request.torque_pu * _1_div_512 * motor->GetConfig().mit_output_tor_range_nm();
    // transform to base frame, RPM -> RADS, NM -> AMP
    const float real_velocity_base_rads = RPM2RAD(real_velocity_output_rpm * motor->GetConfig().deduction_ratio(), 1);
    float real_torque_base_amp = real_torque_output_nm / motor->GetConfig().deduction_ratio();
    if(motor->GetConfig().torque_constant_valid() && motor->GetConfig().torque_constant() > 0.0f) // [Nm/A]
    {
        real_torque_base_amp /= motor->GetConfig().torque_constant();
    }
    else real_torque_base_amp = 0.0f;
    Motion target_motion
    {
        .ref = io_ref,
        .torque = {0.0f, 0.0f, Motion::TorqueUnit::AMP},
        .speed = {0.0f, 0.0f, io_speed_unit},
        .pos = {request.target, 0.0f, io_pos_unit}
    };
    if(request.relative)
    {
        if(request.rel_curr_based)
        {
            const auto curr_motion = motor->GetCurrentMotionStruct(target_motion);
            target_motion.pos.value += curr_motion.pos.value;
        }
        else
        {
            const auto last_target_motion = motor->GetTargetMotionStruct(target_motion);
            target_motion.pos.value += last_target_motion.pos.value;
        }
    }
    target_motion.ConvertSpeedPosToDefault();
    if(target_motion.ref == Motion::Ref::OUTPUT) // OUTPUT -> BASE
    {
        target_motion.speed.value *= motor->GetConfig().deduction_ratio();
        target_motion.speed.limit *= motor->GetConfig().deduction_ratio();
        target_motion.pos.value *= motor->GetConfig().deduction_ratio();
        target_motion.pos.limit *= motor->GetConfig().deduction_ratio();
    }
    target_motion.ref = Motion::Ref::BASE;
    // Now we have BASE ref, with AMP, RADS and RAD.
    if(request.vel_tor_ff)
    {
        target_motion.torque.value = real_torque_base_amp;
        target_motion.speed.value = real_velocity_base_rads;
    }
    else
    {
        target_motion.torque.limit = ABS(real_torque_base_amp);
        target_motion.speed.limit = ABS(real_velocity_base_rads);
    }
    motor->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
    motor->SetTargetMotion(target_motion);
}

void DroneCANProtocol::SendFOCSetVelTargetResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetVelTargetRequest request;
    if(ifoc_SetVelTargetRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    // int10: [-512,511]
    static constexpr float _1_div_512 = 1.0f / 512.0f;
    const float real_torque_output_nm = (float)request.torque_pu * _1_div_512 * motor->GetConfig().mit_output_tor_range_nm();
    // transform to base frame, NM -> AMP
    float real_torque_base_amp = real_torque_output_nm / motor->GetConfig().deduction_ratio();
    if(motor->GetConfig().torque_constant_valid() && motor->GetConfig().torque_constant() > 0.0f) // [Nm/A]
    {
        real_torque_base_amp /= motor->GetConfig().torque_constant();
    }
    else real_torque_base_amp = 0.0f;
    Motion target_motion
    {
        .ref = io_ref,
        .torque = {0.0f, 0.0f, Motion::TorqueUnit::AMP},
        .speed = {request.target, 0.0f, io_speed_unit},
        .pos = {0.0f, 0.0f, io_pos_unit}
    };
    target_motion.ConvertSpeedPosToDefault();
    if(target_motion.ref == Motion::Ref::OUTPUT) // OUTPUT -> BASE
    {
        target_motion.speed.value *= motor->GetConfig().deduction_ratio();
        target_motion.speed.limit *= motor->GetConfig().deduction_ratio();
        target_motion.pos.value *= motor->GetConfig().deduction_ratio();
        target_motion.pos.limit *= motor->GetConfig().deduction_ratio();
    }
    target_motion.ref = Motion::Ref::BASE;
    // Now we have BASE ref, with AMP, RADS and RAD.
    if(request.torque_ff)
    {
        target_motion.torque.value = real_torque_base_amp;
    }
    else target_motion.torque.limit = ABS(real_torque_base_amp);
    motor->SetControlMode(MotorControlMode::CTRL_MODE_VELOCITY);
    motor->SetTargetMotion(target_motion);
}

void DroneCANProtocol::SendFOCSetTorqueTargetResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetTorqueTargetRequest request;
    if(ifoc_SetTorqueTargetRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    Motion target_motion
    {
        .ref = io_ref,
        .torque = {request.target, 0.0f, io_torque_unit},
        .speed = {0.0f, 0.0f, io_speed_unit},
        .pos = {0.0f, 0.0f, io_pos_unit}
    };
    motor->SetControlMode(MotorControlMode::CTRL_MODE_CURRENT);
    motor->SetTargetMotion(target_motion);
}

void DroneCANProtocol::SendFOCSetDebugCmdResponse(DroneCAN::CanardRxTransfer* transfer)
{
    ifoc_SetDebugCmdRequest request;
    if(ifoc_SetDebugCmdRequest_decode(transfer, &request)) return;
    const auto motor = GetMotor<FOCMotor>();
    motor->UpdateWatchdog();
    if(!motor->GetError() && motor->GetCurrentState() == MotorState::IDLE)
    {
        request.phase_a_duty = _constrain(request.phase_a_duty, 0.0f, 1.0f);
        request.phase_b_duty = _constrain(request.phase_b_duty, 0.0f, 1.0f);
        request.phase_c_duty = _constrain(request.phase_c_duty, 0.0f, 1.0f);
        // const auto ab = FOC_Clark({request.phase_a_duty, request.phase_b_duty, request.phase_c_duty});
        // const auto qd = FOC_Park(ab, motor->elec_angle_rad);
        if(!motor->IsArmed()) motor->Arm();
        // motor->Uqd_target = {qd.q * motor->GetBusSense()->voltage, qd.d * motor->GetBusSense()->voltage};
    }
}

void DroneCANProtocol::SendFileGetInfoResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_file_GetInfoRequest request;
    if(uavcan_protocol_file_GetInfoRequest_decode(transfer, &request)) return;
    if(request.path.path.len < sizeof(request.path.path.data)) request.path.path.data[request.path.path.len] = '\0';
    const auto size = BlobNVMStorage().GetKVSize((const char*)request.path.path.data);
    uavcan_protocol_file_GetInfoResponse response
    {
        .size = size,
        .error = {},
        .entry_type = {}
    };
    if(size > 0)
    {
        response.error.value = UAVCAN_PROTOCOL_FILE_ERROR_OK;
        response.entry_type.flags = UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE | UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE | UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_WRITEABLE;
    }
    else
    {
        response.error.value = UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND;
        response.entry_type.flags = 0;
    }
    uint8_t buffer[UAVCAN_PROTOCOL_FILE_GETINFO_RESPONSE_MAX_SIZE];
    const auto len = uavcan_protocol_file_GetInfoResponse_encode(&response, buffer);
    _SendResponse(transfer, UAVCAN_PROTOCOL_FILE_GETINFO_SIGNATURE, UAVCAN_PROTOCOL_FILE_GETINFO_ID, buffer, len);
}

void DroneCANProtocol::SendFileDeleteResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_file_DeleteRequest request;
    if(uavcan_protocol_file_DeleteRequest_decode(transfer, &request)) return;
    if(request.path.path.len < sizeof(request.path.path.data)) request.path.path.data[request.path.path.len] = '\0';
    uavcan_protocol_file_DeleteResponse response
    {
        .error = {.value = BlobNVMStorage().ClearNVM((const char*)request.path.path.data) == FuncRetCode::OK ? (int16_t)UAVCAN_PROTOCOL_FILE_ERROR_OK : (int16_t)UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND,}
    };
    uint8_t buffer[UAVCAN_PROTOCOL_FILE_DELETE_RESPONSE_MAX_SIZE];
    const auto len = uavcan_protocol_file_DeleteResponse_encode(&response, buffer);
    _SendResponse(transfer, UAVCAN_PROTOCOL_FILE_DELETE_SIGNATURE, UAVCAN_PROTOCOL_FILE_DELETE_ID, buffer, len);
}

void DroneCANProtocol::SendFileReadResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_file_ReadRequest request;
    if(uavcan_protocol_file_ReadRequest_decode(transfer, &request)) return;
    if(request.path.path.len < sizeof(request.path.path.data)) request.path.path.data[request.path.path.len] = '\0';
    // check key match
    if(!file_read_buffer.key ||
        file_read_buffer.key_length != request.path.path.len ||
        strncmp((const char*)request.path.path.data, file_read_buffer.key, file_read_buffer.key_length))
    {
        // get data length first
        file_read_buffer.data_length = BlobNVMStorage().GetKVSize((const char*)request.path.path.data);
        if(file_read_buffer.data_length > 0)
        {
            // rebuild key, and read data to buffer here.
            file_read_buffer.key_length = request.path.path.len;
            if(file_read_buffer.key)
            {
                vPortFree(file_read_buffer.key);
                file_read_buffer.key = nullptr;
            }
            file_read_buffer.key = (char*)pvPortMalloc(file_read_buffer.key_length + 1);
            if(!file_read_buffer.key) return;
            memcpy(file_read_buffer.key, request.path.path.data, file_read_buffer.key_length);
            file_read_buffer.key[file_read_buffer.key_length] = '\0';
            // read data
            if(file_read_buffer.data)
            {
                vPortFree(file_read_buffer.data);
                file_read_buffer.data = nullptr;
            }
            // read buffer
            file_read_buffer.data = (uint8_t*)pvPortMalloc(file_read_buffer.data_length);
            if(!file_read_buffer.data) return;
            if(BlobNVMStorage().ReadNVM(file_read_buffer.key, file_read_buffer.data, (uint16_t*)&file_read_buffer.data_length) != FuncRetCode::OK)
            {
                if(file_read_buffer.data)
                {
                    vPortFree(file_read_buffer.data);
                    file_read_buffer.data = nullptr;
                }
                file_read_buffer.data_length = 0;
            }
        }
    }
    file_read_buffer.last_read_tick = xTaskGetTickCount();

    uavcan_protocol_file_ReadResponse response{};
    if(!file_read_buffer.key || file_read_buffer.key_length == 0 || !file_read_buffer.data || file_read_buffer.data_length == 0)
    {
        response.error.value = UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND;
    }
    else
    {
        // determine offset
        if(request.offset >= file_read_buffer.data_length)
        {
            response.error.value = UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE;
        }
        else
        {
            const uint16_t size_to_read = MIN(sizeof(response.data.data), file_read_buffer.data_length - request.offset);
            const uint8_t* ptr = file_read_buffer.data + request.offset;
            memcpy(response.data.data, ptr, size_to_read);
            response.data.len = size_to_read;
        }
    }

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_RESPONSE_MAX_SIZE];
    const auto len = uavcan_protocol_file_ReadResponse_encode(&response, buffer);
    _SendResponse(transfer, UAVCAN_PROTOCOL_FILE_READ_SIGNATURE, UAVCAN_PROTOCOL_FILE_READ_ID, buffer, len);
}

void DroneCANProtocol::SendGetNodeInfoResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_GetNodeInfoResponse response
    {
        .status = BuildNodeStatus(),
        .software_version =
        {
            .major = get_sw_ver_major(),
            .minor = get_sw_ver_minor(),
            .optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT |
                                    UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC,
            .vcs_commit = get_sw_ver_vcs(),
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
    response.name.len = strnlen(node_name, sizeof(node_name));
    memcpy(response.name.data, node_name, response.name.len);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    const uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&response, buffer);

    _SendResponse(transfer, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID, buffer, len);
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
    uint8_t buffer[UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_MAX_SIZE];
    const uint32_t len = uavcan_protocol_GetTransportStatsResponse_encode(&response, buffer);

    _SendResponse(transfer, UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_SIGNATURE, UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_ID, buffer, len);
}

void DroneCANProtocol::SendParamGetSetResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_param_GetSetRequest request;
    if(uavcan_protocol_param_GetSetRequest_decode(transfer, &request)) return;
    uavcan_protocol_param_GetSetResponse response{};
    // #1: WRITE & READ, #2: READ ONLY
    // We can iterate through either name or index.
    // We will first check for request.name.
    // Add a node_name field. index = 0, Blob key: "n" "internal_id"
    const auto motor = GetMotor<FOCMotor>();
    MemberInfo info{};
    uint8_t* target_ptr = nullptr;
    char target_name[sizeof(uavcan_protocol_param_GetSetRequest::name.data) + 2 + 6]{};
    if(request.name.len > 0) // len > 0, likely a WRITE request.
    {
        char buffer[sizeof(uavcan_protocol_param_GetSetRequest::name.data) + 2]{};
        if(request.name.len < sizeof(buffer))
        {
            memcpy(buffer, request.name.data, request.name.len);
            buffer[request.name.len] = '\0';
            auto original_len = strnlen(buffer, sizeof(buffer));
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
                    reflect = &(motor->GetConfig().GetReflectMap());
                    start_ptr = (uint8_t*)&(motor->GetConfig());
                }
                else if(original_len >= strlen(DRONECAN_NODE_NAME) && strcmp(buffer, DRONECAN_NODE_NAME) == 0)
                {
                    info.first = Reflection::ProtoFieldType::STRING;
                    info.second = sizeof(node_name);
                    target_ptr = (uint8_t*)node_name;
                    memcpy(target_name, DRONECAN_NODE_NAME, sizeof(DRONECAN_NODE_NAME));
                    target_name[sizeof(DRONECAN_NODE_NAME)] = '\0';
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
        auto index = request.index;
        if(index == 0) // node_name
        {
            memcpy(target_name, DRONECAN_NODE_NAME, sizeof(DRONECAN_NODE_NAME));
            target_name[sizeof(DRONECAN_NODE_NAME)] = '\0';
            info.first = Reflection::ProtoFieldType::STRING;
            info.second = sizeof(node_name);
            target_ptr = (uint8_t*)node_name;
        }
        else
        {
            index--;
            const auto& board_map = BoardConfig().GetConfig().GetReflectMap();
            const auto& motor_map = motor->GetConfig().GetReflectMap();
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
                            target_ptr = (uint8_t*)&(motor->GetConfig()) + info.second;
                            break;
                        }
                        iter_index++;
                    }
                }
            }
        }
    }
    // here we got both info, target_ptr, and target_name.
    const auto target_name_len = strnlen(target_name, sizeof(target_name));
    if(target_ptr && target_name_len > 1)
    {
        // param exists, deciding WRITE + READ / READ ONLY.
        // WRITE
        switch(request.value.union_tag)
        {
            case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            {
                // int64_t, suitable for: INT32, INT64, UINT32, UINT64(possibly overflow)
                const int64_t temp = request.value.integer_value;
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
                const float temp = request.value.real_value;
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
                const uint8_t temp = request.value.boolean_value;
                if(info.first == Reflection::ProtoFieldType::BOOL) *(uint8_t*)target_ptr = (uint8_t)temp;
                break;
            }
            case UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE:
            {
                const auto& str = request.value.string_value;
                if(info.first == Reflection::ProtoFieldType::STRING && info.second > str.len + 1)
                {
                    memcpy(target_ptr, str.data, str.len);
                    target_ptr[str.len] = '\0';
                }
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
            case Reflection::ProtoFieldType::STRING:
            {
                response.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
                const auto len = MIN(strnlen((char*)target_ptr, info.second), sizeof(response.value.string_value.data));
                memcpy(response.value.string_value.data, target_ptr, len);
                response.value.string_value.len = len;
                break;
            }
            default:
            {
                return;
            }
        }
        if(target_name[target_name_len - 1] == '_') response.name.len = target_name_len - 1; // we dont want underscore
        else response.name.len = target_name_len;
        memcpy(response.name.data, target_name, response.name.len);
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    const uint16_t len = uavcan_protocol_param_GetSetResponse_encode(&response, buffer);

    _SendResponse(transfer, UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE, UAVCAN_PROTOCOL_PARAM_GETSET_ID, buffer, len);
}

void DroneCANProtocol::SendRestartNodeResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_RestartNodeRequest request;
    if(uavcan_protocol_RestartNodeRequest_decode(transfer, &request)) return;
    const bool is_restart_valid = request.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;

    if(is_restart_valid) HAL::SystemReboot(); // reboot immediately.

    uavcan_protocol_RestartNodeResponse response {.ok = is_restart_valid};

    uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE];
    const uint32_t len = uavcan_protocol_RestartNodeResponse_encode(&response, buffer);

    _SendResponse(transfer, UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_SIGNATURE, UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_ID, buffer, len);
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
    uavcan_protocol_param_ExecuteOpcodeRequest request;
    if(uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &request)) return;
    uavcan_protocol_param_ExecuteOpcodeResponse response{};
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
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
            // save motor config
            ret = motor->config.SaveNVMConfig();
            if(ret != FuncRetCode::OK)
            {
                response.argument = 3; // motor config save failed
                response.ok = false;
                break;
            }
            // save node name
            char key[sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX) + 1];
            memcpy(key, DRONECAN_NODE_NAME_DB_KEY_PREFIX, sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX) - 1);
            key[sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX) - 1] = motor->GetInternalID() + '0';
            key[sizeof(DRONECAN_NODE_NAME_DB_KEY_PREFIX)] = '\0';
            ret = BlobNVMStorage().SaveNVM(key, (uint8_t*)node_name, strnlen(node_name, sizeof(node_name)));
            if(ret != FuncRetCode::OK)
            {
                response.argument = 4;
                response.ok = false;
                break;
            }
            response.argument = 0;
            response.ok = true;
            break;
        }
        case 3: // reboot to BL
        {
            constexpr BootloaderMsg message{}; // empty message
            HAL::Bootloader::JumpToBL(message);
            break;
        }
        case 4: // beep identify
        {
            if(motor->GetCurrentState() != MotorState::IDLE)
            {
                response.argument = -1; // state not in IDLE
                response.ok = false;   // identify
                break;
            }
            const auto ret = motor->ToggleBeepIdentify();
            if(ret == FuncRetCode::OK) // current beep state: on
            {
                response.argument = 1;
                response.ok = true;
            }
            else if(ret == FuncRetCode::PARAM_NOT_EXIST) // current beep state: off
            {
                response.argument = 0;
                response.ok = true;
            }
            else
            {
                response.argument = -2;
                response.ok = false;
            }
            break;
        }
        default: break;
    }

    const uint32_t len = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&response, buffer);

    _SendResponse(transfer, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID, buffer, len);
}

void DroneCANProtocol::SendFWUpdateResponse(DroneCAN::CanardRxTransfer* transfer)
{
    uavcan_protocol_file_BeginFirmwareUpdateRequest request;
    if(uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &request)) return;
    const uint32_t path_len = request.image_file_remote_path.path.len;

    // uavcan_protocol_file_BeginFirmwareUpdateResponse response{};

    // reject path_len != sizeof(fw_update.path) (7)
    static_assert(sizeof(BootloaderMsg::update_image_path) == 7);
    if(path_len != sizeof(BootloaderMsg::update_image_path))
    {
        // response.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN;
        // goto response;
        return;
    }

    BootloaderMsg message{};
    message.server_node_id = request.source_node_id;
    memcpy(message.update_image_path, request.image_file_remote_path.path.data, path_len);

    HAL::Bootloader::JumpToBL(message);
}

void DroneCANProtocol::RequestDNAAllocation()
{
    const auto current_tick = xTaskGetTickCount();
    // srand(current_tick); // update random seed
    // dna.next_dna_request_tick = current_tick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
    //                             (rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
    ifoc_srand(current_tick); // update random seed
    dna.next_dna_request_tick = current_tick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
                                (ifoc_rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
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
    // srand(current_tick); // update random seed
    // dna.next_dna_request_tick = current_tick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
    //                             (rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
    ifoc_srand(current_tick); // update random seed
    dna.next_dna_request_tick = current_tick + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
                                (ifoc_rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);
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
        // canard.node_id = msg.node_id;
        SetNodeID(msg.node_id);
    }
}

void DroneCANProtocol::OnRxEvent(const DataType::Comm::CANMessage& message)
{
    if(!message.is_ext || message.is_rtr) return;
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xQueueSendFromISR(isr_msg_queue, &message, &xHigherPriorityTaskWoken);
    // if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    isr_msg_fifo.put(&message, 1);
}

void DroneCANProtocol::SetNodeID(uint8_t node_id)
{
    canard.node_id = node_id;
    const auto motor = GetMotor<FOCMotor>();
    uint32_t id = 0, mask = 0;
    BuildServiceFilter(canard.node_id, id, mask);
    can->SetHWFilter(motor->GetInternalID(), id, mask, true, false); // accept ext only
}

void DroneCANProtocol::BuildServiceFilter(uint8_t self_id, uint32_t& ret_id, uint32_t& ret_mask)
{
    // Reference: https://dronecan.github.io/Specification/4.1_CAN_bus_transport_layer/#:~:text=the%20specification).-,CAN%20frame%20format,-DroneCAN%20uses%20only
    ret_id = 0;
    ret_mask = 0;
    if(self_id == CANARD_BROADCAST_NODE_ID || self_id > CANARD_MAX_NODE_ID) return; // no filter will be built
    ret_mask = (1 << 7) | (0x7F << 8); // match those fields
    ret_id = (1 << 7) | (self_id << 8);
}
}
