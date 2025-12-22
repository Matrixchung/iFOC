#pragma once

#include "protocol_base.hpp"
#include "../Common/Interface/can_base.hpp"
#include "../ThirdParty/libcanard/canard.h"
#include "../Common/foc_task.hpp"
#include "../ThirdParty/libcanard/dsdl/uavcan/_register/Name_1_0.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/_register/Value_1_0.h"

namespace iFOC::Protocol
{
class UAVCANProtocol final : public ProtocolBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(UAVCANProtocol);
public:
    explicit UAVCANProtocol(HAL::CANBase* base);
    ~UAVCANProtocol();
    void Init() override;
private:
    class PollingTask final : public Task
    {
        OVERRIDE_NEW();
        DELETE_COPY_CONSTRUCTOR(PollingTask);
    private:
        UAVCANProtocol* parent = nullptr;
        // TickType_t last_heartbeat_tick = 0;
        struct
        {
            TickType_t heartbeat = 0;
            TickType_t port_list = 0;
        } last_send_tick;
    public:
        explicit PollingTask(UAVCANProtocol* p);
        void UpdateNormal() override;
    };
    friend class PollingTask;
    PollingTask polling_task;
    HAL::CANBase* can = nullptr;
    uint64_t rx_frame_received = 0;
    uint64_t tx_frame_sent = 0;
    uint64_t tx_frame_expired = 0;
    uint64_t tx_frame_failed = 0;
    CanardInstance canard{};
    CanardTxQueue tx_queue{};
    QueueHandle_t isr_msg_queue{};
    // CanardRxSubscription storage area begins
    // struct _SubscriptionHash { std::size_t operator() (const CanardRxSubscription &k) const; };
    // struct _SubscriptionEqual { bool operator() (const CanardRxSubscription &lhs, const CanardRxSubscription &rhs) const; };
    // HashSet<CanardRxSubscription, _SubscriptionHash, _SubscriptionEqual> rx_subscriptions{}; // resource-heavy stuff
    List<CanardRxSubscription> rx_subscriptions{}; // SUBSCRIPTION INSTANCES SHALL NOT BE MOVED WHILE IN USE.
    // CanardRxSubscription storage area ends
    struct
    {
        uint8_t uavcan_node_heartbeat = 0;
        uint8_t uavcan_node_port_list = 0;
    } next_transfer_id;
    void ProcessTransfer(const CanardRxTransfer& transfer);
    void SendHeartbeat();
    void SendPortList();
    void SendGetInfoResponse(const CanardRxTransfer& transfer); // For responses, we need the original metadata as param.
    void SendGetTransportStatsResponse(const CanardRxTransfer& transfer);
    void SendRegisterListResponse(const CanardRxTransfer& transfer);
    void SendRegisterAccessResponse(const CanardRxTransfer& transfer);
    void GetRegisterNameByGlobalIndex(char* dst, uint16_t max_size, uint16_t index);
    bool ReadRegisterByName(const uavcan_register_Name_1_0& name, uavcan_register_Value_1_0& dst_value);
    bool CheckRegisterByName(const uavcan_register_Name_1_0& name);
    bool WriteRegisterByName(const uavcan_register_Name_1_0& name, const uavcan_register_Value_1_0& value);
    void OnRxEvent(const DataType::Comm::CANMessage& message);
    FuncRetCode SubscribeTransfer(CanardTransferKind kind, CanardPortID port, size_t max_size, CanardMicrosecond timeout_us);
    FuncRetCode UnsubscribeTransfer(CanardTransferKind kind, CanardPortID port);
    int8_t TransmitFrame(CanardMutableFrame* frame) const;
    static void* canard_mem_alloc(void* ref, size_t size);
    static void canard_mem_free(void* ref, size_t size, void* ptr);
};
}