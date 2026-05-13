#pragma once

#include "motion.hpp"
#include "protocol_base.hpp"
#include "../DataType/Ringbuf/obj_kfifo.hpp"
#include "../DataType/Headers/Base/motion.hpp"
#include "../Common/Interface/can_base.hpp"
#include "../ThirdParty/libcanard-dronecan/canard_dronecan.h"
#include "../Common/foc_task.hpp"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/NodeStatus.h"
#include "../DataType/Headers/Base/motor_state.h"
#include "../DataType/Headers/Base/motor_control_mode.h"

namespace iFOC::Protocol
{
class DroneCANProtocol final : public ProtocolBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(DroneCANProtocol);
public:
    explicit DroneCANProtocol(HAL::CANBase* base);
    ~DroneCANProtocol();
    void Init() override;
private:
    class PollingTask final : public Task
    {
        OVERRIDE_NEW();
        DELETE_COPY_CONSTRUCTOR(PollingTask);
    private:
        DroneCANProtocol* parent = nullptr;
        struct
        {
            TickType_t heartbeat = 0;
            TickType_t feedback = 0;
            TickType_t misc_feedback = 0;
        } last_send_tick;
        struct
        {
            bool is_init_send = false;
            bool last_motor_arm_state = false;
            MotorState last_motor_state = MotorState::IDLE;
            MotorControlMode last_motor_mode = MotorControlMode::CTRL_MODE_POSITION;
            uint64_t last_error = 0;
        } monitor;
        TickType_t xLastWakeTick = 0;
    public:
        explicit PollingTask(DroneCANProtocol* p);
        void InitNormal() override;
        void UpdateNormal() override;
        void UpdateMid(float Ts) override;
    };
    friend class PollingTask;
    static constexpr size_t CANARD_MEMORY_POOL_SIZE = 2048;
    static constexpr size_t ISR_MSG_QUEUE_SIZE = 32;
    static constexpr size_t FILE_READ_TIMEOUT_MS = 1000;
    PollingTask polling_task;
    DroneCAN::CanardInstance canard{};
    /* node_name moved to FOCMotorConfig::node_name (foc_motor_config.proto tag 3) */
    // QueueHandle_t isr_msg_queue{};
    DataType::Ringbuf::obj_kfifo_t<DataType::Comm::CANMessage> isr_msg_fifo{};
    DataType::Ringbuf::obj_kfifo_t<DataType::Comm::CANMessage> tx_msg_fifo{};
    uint64_t rx_frame_received = 0;
    uint64_t rx_frame_error = 0;
    uint64_t tx_frame_sent = 0;
    HAL::CANBase* can = nullptr;
    void* canard_memory_pool = nullptr;
    struct
    {
        uint8_t uavcan_protocol_nodestatus = 0;
        uint8_t uavcan_protocol_logmessage = 0;
        uint8_t ifoc_compact_feedback = 0;
        uint8_t ifoc_misc_feedback = 0;
        uint8_t node_allocation = 0;
    } next_transfer_id;
    struct
    {
        uint32_t next_dna_request_tick = 0;
        uint32_t id_allocation_uuid_offset = 0;
    } dna;
    struct
    {
        uint8_t key_length = 0;
        char* key = nullptr;
        uint8_t* data = nullptr;
        uint32_t data_length = 0;
        uint32_t last_read_tick = 0;
    } file_read_buffer;
    Motion::Ref io_ref = Motion::Ref::OUTPUT;
    Motion::TorqueUnit io_torque_unit = Motion::TorqueUnit::AMP;
    Motion::SpeedUnit io_speed_unit = Motion::SpeedUnit::RPM;
    Motion::PosUnit io_pos_unit = Motion::PosUnit::DEG;
    void ProcessTransfer(DroneCAN::CanardInstance* ins, DroneCAN::CanardRxTransfer* transfer);
    bool ShouldAccept(const DroneCAN::CanardInstance* ins,        ///< Library instance
                      uint64_t* out_data_type_signature,          ///< Must be set by the application!
                      uint16_t data_type_id,                      ///< Refer to the specification
                      DroneCAN::CanardTransferType transfer_type, ///< Refer to CanardTransferType
                      uint8_t source_node_id);
    void _SendResponse(DroneCAN::CanardRxTransfer* transfer, uint64_t signature, uint8_t id, const void* payload, uint16_t len);
    uavcan_protocol_NodeStatus BuildNodeStatus();
    void SendNodeStatus();
    void SendLogMessage(uint8_t level, const char* text, uint8_t max_buffer_len);

    void SendFOCCompactFeedback();
    void SendFOCMiscFeedback();
    void SendFOCGetClearErrorResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCGetClearErrorIndexResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCGetOSStatsResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCGetTaskStatsResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCGetEncodersResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCGetCurrentMotionResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCGetTargetMotionResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetRefFrameResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetMotorStateResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetControlModeResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetMITTargetResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetTrajTargetResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetPosTargetResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetVelTargetResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetTorqueTargetResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFOCSetDebugCmdResponse(DroneCAN::CanardRxTransfer* transfer);

    void SendFileGetInfoResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFileDeleteResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFileReadResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFileWriteResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendGetNodeInfoResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendGetTransportStatsResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendParamGetSetResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendRestartNodeResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendExecuteOpcodeResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendFWUpdateResponse(DroneCAN::CanardRxTransfer* transfer);
    void RequestDNAAllocation();
    void OnDNAAllocation(DroneCAN::CanardRxTransfer* transfer);
    void OnRxEvent(const DataType::Comm::CANMessage& message);

    void SetNodeID(uint8_t node_id);
    static void BuildServiceFilter(uint8_t self_id, uint32_t& ret_id, uint32_t& ret_mask);
};
}