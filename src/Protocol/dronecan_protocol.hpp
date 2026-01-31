#pragma once

#include "protocol_base.hpp"
#include "../Common/Interface/can_base.hpp"
#include "../ThirdParty/libcanard-dronecan/canard_dronecan.h"
#include "../Common/foc_task.hpp"
#include "../ThirdParty/libcanard-dronecan/dsdl/uavcan/protocol/NodeStatus.h"

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
        } last_send_tick;
    public:
        explicit PollingTask(DroneCANProtocol* p);
        void UpdateNormal() override;
    };
    friend class PollingTask;
    static constexpr size_t CANARD_MEMORY_POOL_SIZE = 1024;
    static constexpr size_t ISR_MSG_QUEUE_SIZE = 32;
    PollingTask polling_task;
    DroneCAN::CanardInstance canard{};
    QueueHandle_t isr_msg_queue{};
    uint64_t rx_frame_received = 0;
    uint64_t rx_frame_error = 0;
    uint64_t tx_frame_sent = 0;
    HAL::CANBase* can = nullptr;
    void* canard_memory_pool = nullptr;
    struct
    {
        uint8_t uavcan_protocol_nodestatus = 0;
        uint8_t ifoc_compact_feedback = 0;
        uint8_t node_allocation = 0;
    } next_transfer_id;
    struct
    {
        uint32_t next_dna_request_tick = 0;
        uint32_t id_allocation_uuid_offset = 0;
    } dna;
    void ProcessTransfer(DroneCAN::CanardInstance* ins, DroneCAN::CanardRxTransfer* transfer);
    bool ShouldAccept(const DroneCAN::CanardInstance* ins,        ///< Library instance
                      uint64_t* out_data_type_signature,          ///< Must be set by the application!
                      uint16_t data_type_id,                      ///< Refer to the specification
                      DroneCAN::CanardTransferType transfer_type, ///< Refer to CanardTransferType
                      uint8_t source_node_id);
    uavcan_protocol_NodeStatus BuildNodeStatus();
    void SendNodeStatus();
    void SendFOCCompactFeedback();
    void SendGetInfoResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendGetTransportStatsResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendParamGetSetResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendRestartNodeResponse(DroneCAN::CanardRxTransfer* transfer);
    void SendExecuteOpcodeResponse(DroneCAN::CanardRxTransfer* transfer);
    void RequestDNAAllocation();
    void OnDNAAllocation(DroneCAN::CanardRxTransfer* transfer);
    void OnRxEvent(const DataType::Comm::CANMessage& message);
};
}