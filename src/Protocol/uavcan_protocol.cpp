#include "uavcan_protocol.hpp"

#include "foc_motor.hpp"

// DSDL definitions import
// Cyphal(UAVCAN v1) definitions below
#include "../ThirdParty/libcanard/dsdl/nunavut/support/serialization.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/node/Heartbeat_1_0.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/node/GetInfo_1_0.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/node/port/List_1_0.h"
// UAVCAN v0(Legacy) definitions below


#define KILO 1000L
#define MEGA ((int64_t) KILO * KILO)

static void _fill_subscriptions_to_subject_list(const CanardTreeNode* const tree, uavcan_node_port_SubjectIDList_1_0* const obj)
{
    if(tree)
    {
        _fill_subscriptions_to_subject_list(tree->lr[0], obj);
        auto sub = (const CanardRxSubscription*)tree;
        obj->sparse_list.elements[obj->sparse_list.count++].value = sub->port_id;
        _fill_subscriptions_to_subject_list(tree->lr[1], obj);
    }
}

static void _fill_subscriptions_to_svc_list(const CanardTreeNode* const tree, uavcan_node_port_ServiceIDList_1_0* const obj)
{
    if(tree)
    {
        _fill_subscriptions_to_svc_list(tree->lr[0], obj);
        auto sub = (const CanardRxSubscription*)tree;
        (void)nunavutSetBit(&obj->mask_bitpacked_[0], sizeof(obj->mask_bitpacked_), sub->port_id, true);
        _fill_subscriptions_to_svc_list(tree->lr[1], obj);
    }
}

namespace iFOC::Protocol
{
UAVCANProtocol::UAVCANProtocol(HAL::CANBase* base) : polling_task(this), can(base)
{
    isr_msg_queue = xQueueCreate(16, sizeof(DataType::Comm::CANMessage));
}

UAVCANProtocol::~UAVCANProtocol()
{
    vQueueDelete(isr_msg_queue);
}

void UAVCANProtocol::Init()
{
    const auto motor = GetMotor<FOCMotor>();
    const CanardMemoryResource memory = {nullptr, canard_mem_free, canard_mem_alloc};
    canard = canardInit(memory);
    canard.node_id = motor->GetConfig().node_id();
    tx_queue = canardTxInit(256, CANARD_MTU_CAN_CLASSIC, memory); // TODO: For CAN FD, the MTU is 64.
    can->RegisterRxHandler(std::bind(&UAVCANProtocol::OnRxEvent, this, std::placeholders::_1));
    polling_task.Start();
    // Subscribe topics below
    // Request - GetInfo
    SubscribeTransfer(CanardTransferKindRequest,
                      uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
                      uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_,
                      CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC);

}

void UAVCANProtocol::ProcessTransfer(const CanardRxTransfer& transfer)
{
    // real process
    if(transfer.metadata.transfer_kind == CanardTransferKindMessage) // Message
    {

    }
    else if(transfer.metadata.transfer_kind == CanardTransferKindRequest) // Request
    {
        switch(transfer.metadata.port_id)
        {
            case uavcan_node_GetInfo_1_0_FIXED_PORT_ID_:
            {
                // The request object is empty so we don't bother deserializing it. Just send the response.
                SendGetInfoResponse(transfer);
                break;
            }
            default: break; // unimplemented requests
        }
    }
    else if(transfer.metadata.transfer_kind == CanardTransferKindResponse) // Response
    {
        // Maybe we can ignore it cause we are a slave node?
        // A slave node may not receive Response from other nodes...
    }
}

void UAVCANProtocol::SendHeartbeat()
{
    uavcan_node_Heartbeat_1_0 heartbeat{};
    heartbeat.uptime = HAL::GetUptimeSeconds();
    heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
    heartbeat.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;

    CanardPayload payload{};
    constexpr size_t original_max_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    payload.size = original_max_size;
    payload.data = canard.memory.allocate(nullptr, payload.size);
    const auto err = uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, (uint8_t*)payload.data, &payload.size);
    if(err >= 0)
    {
        const CanardTransferMetadata meta
        {
            .priority = CanardPriorityNominal,
            .transfer_kind = CanardTransferKindMessage,
            .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            .remote_node_id = CANARD_NODE_ID_UNSET,
            .transfer_id = (CanardTransferID)(next_transfer_id.uavcan_node_heartbeat++)
        };
        const auto now_usec = xTaskGetTickCount() * 1000;
        canardTxPush(&tx_queue,
                     &canard,
                     now_usec + (uavcan_node_Heartbeat_1_0_MAX_PUBLICATION_PERIOD * MEGA),
                     &meta,
                     payload,
                     now_usec,
                     &tx_frame_expired
                     );
    }
    canard.memory.deallocate(nullptr, original_max_size, (void*)payload.data);
}

void UAVCANProtocol::SendPortList()
{
    uavcan_node_port_List_1_0 m{};
    uavcan_node_port_SubjectIDList_1_0_select_sparse_list_(&m.publishers);
    uavcan_node_port_SubjectIDList_1_0_select_sparse_list_(&m.subscribers);

    // fill the publish port ids.
    {
        size_t* const cnt = &m.publishers.sparse_list.count;
        m.publishers.sparse_list.elements[(*cnt)++].value = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
        m.publishers.sparse_list.elements[(*cnt)++].value = uavcan_node_port_List_1_0_FIXED_PORT_ID_;
    }

    _fill_subscriptions_to_subject_list(canard.rx_subscriptions[CanardTransferKindMessage], &m.subscribers);
    _fill_subscriptions_to_svc_list(canard.rx_subscriptions[CanardTransferKindRequest], &m.servers);
    _fill_subscriptions_to_svc_list(canard.rx_subscriptions[CanardTransferKindResponse], &m.clients);

    CanardPayload payload{};
    constexpr size_t original_max_size = uavcan_node_port_List_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_; // TODO: the message size is always small.
    payload.size = original_max_size;
    payload.data = canard.memory.allocate(nullptr, payload.size);
    const auto err = uavcan_node_port_List_1_0_serialize_(&m, (uint8_t*)payload.data, &payload.size);
    if(err >= 0)
    {
        const CanardTransferMetadata meta
        {
            .priority = CanardPriorityOptional, // OPTIONAL Prior.
            .transfer_kind = CanardTransferKindMessage,
            .port_id = uavcan_node_port_List_1_0_FIXED_PORT_ID_,
            .remote_node_id = CANARD_NODE_ID_UNSET,
            .transfer_id = next_transfer_id.uavcan_node_port_list++,
        };
        const auto now_usec = xTaskGetTickCount() * 1000;
        canardTxPush(&tx_queue,
                     &canard,
                     now_usec + MEGA,
                     &meta,
                     payload,
                     now_usec,
                     &tx_frame_expired
                    );
    }
    canard.memory.deallocate(nullptr, original_max_size, (void*)payload.data);
}

void UAVCANProtocol::SendGetInfoResponse(const CanardRxTransfer& transfer)
{
    uavcan_node_GetInfo_Response_1_0 response
    {
        .protocol_version = {CANARD_CYPHAL_SPECIFICATION_VERSION_MAJOR,
                                CANARD_CYPHAL_SPECIFICATION_VERSION_MINOR},
        .hardware_version = {1, 0}, // TODO: PLACEHOLDER HERE
        .software_version = {(uint8_t)YEAR(), (uint8_t)MONTH()}, // TODO: PLACEHOLDER HERE
        .software_vcs_revision_id = 0, // TODO: PLACEHOLDER HERE
        .unique_id = {},
        .name = {},
        .software_image_crc = {},
        .certificate_of_authenticity = {},
    };
    auto serial_number = HAL::GetSerialNumber();
    memcpy(response.unique_id, &serial_number, sizeof(serial_number));

    CanardPayload payload{};
    constexpr size_t original_max_size = uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    payload.size = original_max_size;
    payload.data = canard.memory.allocate(nullptr, payload.size);
    const auto err = uavcan_node_GetInfo_Response_1_0_serialize_(&response, (uint8_t*)payload.data, &payload.size);
    if(err >= 0)
    {
        CanardTransferMetadata response_meta = transfer.metadata;
        response_meta.transfer_kind = CanardTransferKindResponse;
        const auto now_usec = xTaskGetTickCount() * 1000;
        canardTxPush(&tx_queue,
                     &canard,
                     transfer.timestamp_usec + MEGA,
                     &response_meta,
                     payload,
                     now_usec,
                     &tx_frame_expired
                    );
    }
    canard.memory.deallocate(nullptr, original_max_size, (void*)payload.data);
}

UAVCANProtocol::PollingTask::PollingTask(UAVCANProtocol* p) : Task("UAVCANPoll"), parent(p)
{
    RegisterTask(TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 4;
    config.stack_depth = 1024;
}

void UAVCANProtocol::PollingTask::UpdateNormal()
{
    const auto motor = parent->GetMotor<FOCMotor>();
    parent->canard.node_id = motor->GetConfig().node_id();
    // #1: Response received transfer first
    DataType::Comm::CANMessage message{};
    if(xQueueReceive(parent->isr_msg_queue, &message, 1) == pdTRUE)
    {
        CanardFrame rx_frame{};
        rx_frame.extended_can_id = message.cob_id;
        rx_frame.payload.size = message.len;
        rx_frame.payload.data = message.data;
        CanardRxTransfer transfer{};
        const auto result = canardRxAccept(&parent->canard,
                                           xTaskGetTickCountFromISR() * 1000,
                                           &rx_frame,
                                           0,
                                           &transfer,
                                           nullptr);
        if(result < 0)
        {
            // An error has occurred: either an argument is invalid or we've ran out of memory.
            // It is possible to statically prove that an out-of-memory will never occur for a given application if
            // the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
            // Reception of an invalid frame is NOT an error.
        }
        else if(result == 1)
        {
            parent->ProcessTransfer(transfer);
            parent->canard.memory.deallocate(parent->canard.memory.user_reference,
                                             transfer.payload.allocated_size,
                                       transfer.payload.data);
        }
        else
        {
            // Nothing to do.
            // The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
            // Reception of an invalid frame is NOT reported as an error because it is not an error.
        }
    }
    // #2: Periodically tasks here
    // All Cyphal nodes that have a node-ID are required to publish this message to its fixed subject periodically.
    // Nodes that do not have a node-ID (also known as "anonymous nodes") shall not publish to this subject.
    const bool anonymous = parent->canard.node_id > CANARD_NODE_ID_MAX; // Library functions treat all values above CANARD_NODE_ID_MAX as anonymous.
    if(!anonymous)
    {
        // can_heartbeat & port list
        auto interval_ms = motor->GetConfig().can_heartbeat_interval_ms();
        if(interval_ms > 0)
        {
            interval_ms = _constrain(interval_ms, 1, uavcan_node_Heartbeat_1_0_MAX_PUBLICATION_PERIOD * 1000);
            if((xTaskGetTickCount() - last_send_tick.heartbeat) > interval_ms)
            {
                last_send_tick.heartbeat = xTaskGetTickCount();
                parent->SendHeartbeat();
            }
            // port list, at fixed interval
            if((xTaskGetTickCount() - last_send_tick.port_list) >(uavcan_node_port_List_1_0_MAX_PUBLICATION_PERIOD * 1000))
            {
                last_send_tick.port_list = xTaskGetTickCount();
                parent->SendPortList();
            }
        }
    }
    // #3: Generate Tx packets
    const auto ret = canardTxPoll(&parent->tx_queue,
                                       &parent->canard,
                                       xTaskGetTickCount() * 1000,
                                       parent,
                                       [](auto* const ref, const auto ddl, auto* frame)
                                       {
                                           (void)ddl;
                                            auto* const parent = static_cast<UAVCANProtocol*>(ref);
                                            return parent->TransmitFrame(frame);
                                       },
                                       &parent->tx_frame_expired,
                                       &parent->tx_frame_failed
                                      );
    if(ret > 0) parent->tx_frame_sent++;
    // sleep(1);
}

void UAVCANProtocol::OnRxEvent(const DataType::Comm::CANMessage& message)
{
    if(!message.is_ext || message.is_rtr) return;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(isr_msg_queue, &message, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// std::size_t UAVCANProtocol::_SubscriptionHash::operator()(const CanardRxSubscription& k) const
// {
//     return (std::hash<decltype(k.port_id)>()(k.port_id)) ^ (std::hash<decltype(k.extent)>()(k.extent) << 1);
// }
//
// bool UAVCANProtocol::_SubscriptionEqual::operator()(const CanardRxSubscription& lhs,
//         const CanardRxSubscription& rhs) const
// {
//     return lhs.port_id == rhs.port_id && lhs.extent == rhs.extent;
// }

FuncRetCode UAVCANProtocol::SubscribeTransfer(CanardTransferKind kind, CanardPortID port, size_t max_size,
    CanardMicrosecond timeout_us)
{
    /*
     * Notes from canardRxSubscribe():
     *  The return value is 1 if a new subscription has been created as requested.
     *  The return value is 0 if such subscription existed at the time the function was invoked. In this case,
     *  the existing subscription is terminated and then a new one is created in its place. Pending transfers may be lost.
     *  The return value is a negated invalid argument error if any of the input arguments are invalid.
     */
    /*
     * From struct CanardRxSubscription, we can get the only identifiable elements are: "extent" and "port_id" (READ-ONLY in struct definitions)
     * In case of preventing repeated duplicated subscriptions, we use custom vector to store the existing subscriptions.
     */
    // #1: search for existing subscriptions with same port & max_size(extent)
    CanardRxSubscription* target = nullptr;
    for(auto& i : rx_subscriptions) // using &
    {
        if(i.port_id == port && i.extent == max_size)
        {
            target = &i;
            break;
        }
    }
    // #2: allocate new one if target not found
    if(!target)
    {
        rx_subscriptions.emplace_back();
        target = &rx_subscriptions.back();
        if(!target) return FuncRetCode::BUFFER_FULL;
    }
    const auto result = canardRxSubscribe(&canard,
                                          kind,
                                          port,
                                          max_size,
                                          timeout_us,
                                          target);
    if(result < 0) return FuncRetCode::INVALID_INPUT;
    return FuncRetCode::OK;
}

FuncRetCode UAVCANProtocol::UnsubscribeTransfer(CanardTransferKind kind, CanardPortID port)
{

}

int8_t UAVCANProtocol::TransmitFrame(CanardMutableFrame* frame)
{
    DataType::Comm::CANMessage message
    {
        .cob_id = frame->extended_can_id,
        .is_ext = true,
        .is_rtr = false,
        .len = (uint8_t)_constrain(frame->payload.size, 0, sizeof(DataType::Comm::CANMessage::data))
    };
    memcpy(message.data, frame->payload.data, message.len);
    const auto ret = can->TransmitMessage(message);
    if(ret == FuncRetCode::OK) return 1;
    return 0;
}

void* UAVCANProtocol::canard_mem_alloc(void* ref, size_t size)
{
    (void)ref;
    return pvPortMalloc(size);
}

void UAVCANProtocol::canard_mem_free(void* ref, size_t size, void* ptr)
{
    (void)ref;
    (void)size;
    vPortFree(ptr);
}
}
