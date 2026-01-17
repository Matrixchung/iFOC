#include "cyphal_protocol.hpp"

#include "foc_motor.hpp"

// DSDL definitions import
// Cyphal(UAVCAN v1) definitions below
#include "../ThirdParty/libcanard/dsdl/nunavut/support/serialization.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/node/Heartbeat_1_0.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/node/GetInfo_1_0.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/node/GetTransportStatistics_0_1.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/node/port/List_1_0.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/_register/List_1_0.h"
#include "../ThirdParty/libcanard/dsdl/uavcan/_register/Access_1_0.h"


#define KILO 1000L
#define MEGA ((int64_t) KILO * KILO)
#define NODE_NAME ("com.ifoc.driver") // TODO: Replace with compile-time -D or other definitions

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
CyphalProtocol::CyphalProtocol(HAL::CANBase* base) : polling_task(this), can(base)
{
    isr_msg_queue = xQueueCreate(16, sizeof(DataType::Comm::CANMessage));
}

CyphalProtocol::~CyphalProtocol()
{
    vQueueDelete(isr_msg_queue);
}

void CyphalProtocol::Init()
{
    const auto motor = GetMotor<FOCMotor>();
    const CanardMemoryResource memory = {nullptr, canard_mem_free, canard_mem_alloc};
    canard = canardInit(memory);
    canard.node_id = motor->GetConfig().node_id();
    if(canard.node_id <= CANARD_NODE_ID_MAX) // set HW filter
    {
        CanardFilter filter = canardMakeFilterForServices(canard.node_id);
        can->SetHWFilter(motor->GetInternalID(), filter.extended_can_id, filter.extended_mask);
    }
    tx_queue = canardTxInit(128, CANARD_MTU_CAN_CLASSIC, memory); // TODO: For CAN FD, the MTU is 64.
    polling_task.Start();
    // ### Subscribe Cyphal topics below ###
    // Request - GetInfo
    SubscribeTransfer(CanardTransferKindRequest,
                      uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
                      uavcan_node_GetInfo_Request_1_0_EXTENT_BYTES_,
                      CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC);
    // Request - GetTransportStatistics
    SubscribeTransfer(CanardTransferKindRequest,
                      uavcan_node_GetTransportStatistics_0_1_FIXED_PORT_ID_,
                      uavcan_node_GetTransportStatistics_Request_0_1_EXTENT_BYTES_,
                      CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC);
    // Request - register_List
    SubscribeTransfer(CanardTransferKindRequest,
                      uavcan_register_List_1_0_FIXED_PORT_ID_,
                      uavcan_register_List_Request_1_0_EXTENT_BYTES_,
                      CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC);
    // Request - register_Access
    SubscribeTransfer(CanardTransferKindRequest,
                      uavcan_register_Access_1_0_FIXED_PORT_ID_,
                      uavcan_register_Access_Request_1_0_EXTENT_BYTES_,
                      CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC);
    // Finally, attach the interrupt handler.
    can->RegisterRxHandler(std::bind(&CyphalProtocol::OnRxEvent, this, std::placeholders::_1));
}

void CyphalProtocol::ProcessTransfer(const CanardRxTransfer& transfer)
{
    // real process
    if(transfer.metadata.transfer_kind == CanardTransferKindMessage) // Message
    {
        // Many of the motion commands from here...

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
            case uavcan_node_GetTransportStatistics_0_1_FIXED_PORT_ID_:
            {
                SendGetTransportStatsResponse(transfer);
                break;
            }
            case uavcan_register_Access_1_0_FIXED_PORT_ID_:
            {
                SendRegisterAccessResponse(transfer);
                break;
            }
            case uavcan_register_List_1_0_FIXED_PORT_ID_:
            {
                SendRegisterListResponse(transfer);
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

void CyphalProtocol::SendHeartbeat()
{
    const auto motor = GetMotor<FOCMotor>();
    const auto error = motor->GetError();
    uavcan_node_Heartbeat_1_0 heartbeat
    {
        .uptime = HAL::GetUptimeSeconds(),
        .health = {.value = (error == 0 ? (uint8_t)uavcan_node_Health_1_0_NOMINAL : (uint8_t)uavcan_node_Health_1_0_ADVISORY)},
        .mode = {.value = uavcan_node_Mode_1_0_OPERATIONAL},
        .vendor_specific_status_code = (uint8_t)motor->GetCurrentState()
    };

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

void CyphalProtocol::SendPortList()
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

void CyphalProtocol::SendGetInfoResponse(const CanardRxTransfer& transfer)
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
    response.name.count = strlen(NODE_NAME);
    memcpy(&response.name.elements, NODE_NAME, response.name.count);

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

void CyphalProtocol::SendGetTransportStatsResponse(const CanardRxTransfer& transfer)
{
    uavcan_node_GetTransportStatistics_Response_0_1 response{};
    // fill transfer statistics
    response.transfer_statistics.num_emitted = tx_frame_sent;
    response.transfer_statistics.num_errored = tx_frame_failed + tx_frame_expired;
    response.transfer_statistics.num_received = rx_frame_received;
    // fill interface stats
    response.network_interface_statistics.elements[response.network_interface_statistics.count++] = response.transfer_statistics;

    CanardPayload payload{};
    constexpr size_t original_max_size = uavcan_node_GetTransportStatistics_Response_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    payload.size = original_max_size;
    payload.data = canard.memory.allocate(nullptr, payload.size);
    const auto err = uavcan_node_GetTransportStatistics_Response_0_1_serialize_(&response, (uint8_t*)payload.data, &payload.size);
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
                     &tx_frame_expired);
    }
    canard.memory.deallocate(nullptr, original_max_size, (void*)payload.data);
}

void CyphalProtocol::SendRegisterListResponse(const CanardRxTransfer& transfer)
{
    uavcan_register_List_Request_1_0 request{};
    auto size = transfer.payload.size;
    if(uavcan_register_List_Request_1_0_deserialize_(&request, (const uint8_t*)transfer.payload.data, &size) >= 0)
    {
        uavcan_register_List_Response_1_0 resp{};
        char str[64]{};
        GetRegisterNameByGlobalIndex(str, sizeof(str), request.index);
        auto len = strlen(str);
        if(len > 1) // ignore underscore
        {
            resp.name.name.count = nunavutChooseMin(len - 1, uavcan_register_Name_1_0_name_ARRAY_CAPACITY_);
            memcpy(resp.name.name.elements, str, resp.name.name.count);
        }

        CanardPayload payload{};
        constexpr size_t original_max_size = uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
        payload.size = original_max_size;
        payload.data = canard.memory.allocate(nullptr, payload.size);
        const auto err = uavcan_register_List_Response_1_0_serialize_(&resp, (uint8_t*)payload.data, &payload.size);
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
                         &tx_frame_expired);
        }
        canard.memory.deallocate(nullptr, original_max_size, (void*)payload.data);
    }
}

void CyphalProtocol::SendRegisterAccessResponse(const CanardRxTransfer& transfer)
{
    /*
     *   READ/WRITE BEHAVIORS
     *
     * The write operation is performed first, unless skipped by sending an "empty" value in the request.
     * The server may attempt to convert the type of the supplied value to the correct type if there is a type mitmatch
     * (e.g. uint8 may be converted to uint16); however, servers are not required to perform implicit type conversion,
     * and the rules of such conversion are not explicitly specified, so this behavior should not be relied upon.
     *
     * On the next step the register will be read regardless of the outcome of the write operation. As such, if the write
     * operation could not be performed (e.g. due to a type mismatch or any other issue), the register will retain its old
     * value. By evaluating the response the caller can determine whether the register was written successfully.
     *
     * If only read is desired but not write, the caller shall provide a value of type 'empty'. That will signal the
     * server that the write operation shall be skipped, and it will proceed to read the register immediately.
     *
     * If the requested register does not exist, the write operation will have no effect and the returned value will
     * be empty. Existing register should NOT return 'empty' when read since that would make them indistinguishable from
     * nonexistent registers.
     */
    uavcan_register_Access_Request_1_0 request{};
    auto size = transfer.payload.size;
    if(uavcan_register_Access_Request_1_0_deserialize_(&request, (const uint8_t*)transfer.payload.data, &size) >= 0)
    {
        uavcan_register_Access_Response_1_0 resp{};

        // If we're asked to write a new value:
        if(!uavcan_register_Value_1_0_is_empty_(&request.value))
        {
            // read first to ensure there IS such a register
            if(CheckRegisterByName(request.name))
            {
                // write here
                WriteRegisterByName(request.name, request.value);
            }
        }

        // Read current register state
        uavcan_register_Value_1_0_select_empty_(&resp.value);
        ReadRegisterByName(request.name, resp.value);

        resp._mutable = true; // mutable
        resp.persistent = true; // persistent registers

        resp.timestamp.microsecond = uavcan_time_SynchronizedTimestamp_1_0_UNKNOWN;

        CanardPayload payload{};
        constexpr size_t original_max_size = uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
        payload.size = original_max_size;
        payload.data = canard.memory.allocate(nullptr, payload.size);
        const auto err = uavcan_register_Access_Response_1_0_serialize_(&resp, (uint8_t*)payload.data, &payload.size);
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
                         &tx_frame_expired);
        }
        canard.memory.deallocate(nullptr, original_max_size, (void*)payload.data);
    }
}

void CyphalProtocol::GetRegisterNameByGlobalIndex(char* dst, uint16_t max_size, uint16_t index)
{
    // This service allows the caller to discover the names of all registers available on the server
    // by iterating the index field from zero until an empty name is returned. -- Cyphal Specs v1.0

    // Considering the protocol is linked to every motor instance,
    // so the MotorConfig could only appear once in every cyphal node.
    // But for convenience, BoardConfig across different cyphal nodes may be the same.
    // For example: given a dual-motor driver, we will have two nodes (let their ids be 1 and 2)
    // node #1 will have registers: [BoardConfig, MotorConfig], node #2 will have the same registers.
    // Changing either of the BoardConfig will result in reflection of the other one's.

    // global index rule: first BoardConfig, then MotorConfig
    // naming rule: board.xxx, and motor.xxx, remember removing the ending underscore "_"
    const auto& board_map = BoardConfig().GetConfig().GetReflectMap();
    const auto& motor_map = GetMotor<FOCMotor>()->GetConfig().GetReflectMap();
    const uint16_t total_registers = board_map.size() +
                                     motor_map.size();
    // index: [0 - total_registers - 1]
    if(index >= total_registers) return;
    uint16_t iter_index = 0;
    for(const auto& [name, info] : board_map)
    {
        if(iter_index == index)
        {
            snprintf(dst, max_size, "board.%s", name);
            return;
        }
        iter_index++;
    }
    for(const auto& [name, info] : motor_map)
    {
        if(iter_index == index)
        {
            snprintf(dst, max_size, "motor.%s", name);
            return;
        }
        iter_index++;
    }
    return;
}

bool CyphalProtocol::ReadRegisterByName(const uavcan_register_Name_1_0& name, uavcan_register_Value_1_0& dst_value)
{
    /*
     *  Protobuf Field Type  |      Cyphal Report Type
     *        FLOAT                       real32
     *        INT32            integer32 (accept integer32/16/8)
     *        INT64                      integer64
     *       UINT32            natural32 (accept natural32/16/8)
     *       UINT64                      natural64
     *        BOOL                       natural8
     */
    uavcan_register_Value_1_0_select_empty_(&dst_value);

    char buffer[uavcan_register_Name_1_0_name_ARRAY_CAPACITY_ + 2]{};
    if(name.name.count >= sizeof(buffer)) return false;
    memcpy(buffer, name.name.elements, name.name.count);
    buffer[name.name.count] = '\0';
    auto original_len = strlen(buffer);
    if(original_len <= 7) return false;

    const ReflectMap* reflect = nullptr;
    uint8_t *start_ptr = nullptr;
    // check access region
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
    else return false;

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
        const auto& info = it->second;
        uint8_t *ptr = start_ptr + info.second;
        switch(info.first)
        {
            case Reflection::ProtoFieldType::FLOAT:
            {
                uavcan_register_Value_1_0_select_real32_(&dst_value);
                dst_value.real32.value.count = 1;
                memcpy(dst_value.real32.value.elements, ptr, sizeof(float));
                return true;
            }
            case Reflection::ProtoFieldType::DOUBLE:
            {
                uavcan_register_Value_1_0_select_real64_(&dst_value);
                dst_value.real64.value.count = 1;
                memcpy(dst_value.real64.value.elements, ptr, sizeof(double));
                return true;
            }
            case Reflection::ProtoFieldType::INT32:
            {
                uavcan_register_Value_1_0_select_integer32_(&dst_value);
                dst_value.integer32.value.count = 1;
                memcpy(dst_value.integer32.value.elements, ptr, sizeof(int32_t));
                return true;
            }
            case Reflection::ProtoFieldType::INT64:
            {
                uavcan_register_Value_1_0_select_integer64_(&dst_value);
                dst_value.integer64.value.count = 1;
                memcpy(dst_value.integer64.value.elements, ptr, sizeof(int64_t));
                return true;
            }
            case Reflection::ProtoFieldType::UINT32:
            {
                uavcan_register_Value_1_0_select_natural32_(&dst_value);
                dst_value.natural32.value.count = 1;
                memcpy(dst_value.natural32.value.elements, ptr, sizeof(uint32_t));
                return true;
            }
            case Reflection::ProtoFieldType::UINT64:
            {
                uavcan_register_Value_1_0_select_natural64_(&dst_value);
                dst_value.natural64.value.count = 1;
                memcpy(dst_value.natural64.value.elements, ptr, sizeof(uint64_t));
                return true;
            }
            case Reflection::ProtoFieldType::BOOL:
            {
                uavcan_register_Value_1_0_select_natural8_(&dst_value);
                dst_value.natural8.value.count = 1;
                memcpy(dst_value.natural8.value.elements, ptr, sizeof(uint8_t));
                return true;
            }
            default: break;
        }
    }
    return false;
}

bool CyphalProtocol::CheckRegisterByName(const uavcan_register_Name_1_0& name)
{
    char buffer[uavcan_register_Name_1_0_name_ARRAY_CAPACITY_ + 2]{};
    if(name.name.count >= sizeof(buffer)) return false;
    memcpy(buffer, name.name.elements, name.name.count);
    buffer[name.name.count] = '\0';
    auto original_len = strlen(buffer);
    if(original_len <= 7) return false;

    const ReflectMap* reflect = nullptr;
    // check access region
    if(strncmp(buffer, "board.", 6) == 0)
    {
        reflect = &BoardConfig().GetConfig().GetReflectMap();
    }
    else if(strncmp(buffer, "motor.", 6) == 0)
    {
        reflect = &(GetMotor<FOCMotor>()->GetConfig().GetReflectMap());
    }
    else return false;

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

    if(const auto& it = reflect->find(buffer); it != reflect->end()) return true;
    return false;
}

bool CyphalProtocol::WriteRegisterByName(const uavcan_register_Name_1_0& name, const uavcan_register_Value_1_0& value)
{
    if(uavcan_register_Value_1_0_is_empty_(&value)) return false; // an empty value can't be written
    // currently unsupported data types:
    if(uavcan_register_Value_1_0_is_unstructured_(&value)) return false;
    if(uavcan_register_Value_1_0_is_string_(&value)) return false;
    if(uavcan_register_Value_1_0_is_bit_(&value)) return false;

    /*
     *  Protobuf Field Type  |      Cyphal Report Type
     *        FLOAT                       real32
     *        INT32            integer32 (accept integer32/16/8)
     *        INT64                      integer64
     *       UINT32            natural32 (accept natural32/16/8)
     *       UINT64                      natural64
     *        BOOL                       natural8
     */
    // Cyphal is little-endian protocol maybe?

    // parse name
    char buffer[uavcan_register_Name_1_0_name_ARRAY_CAPACITY_ + 2]{};
    if(name.name.count >= sizeof(buffer)) return false;
    memcpy(buffer, name.name.elements, name.name.count);
    buffer[name.name.count] = '\0';
    auto original_len = strlen(buffer);
    if(original_len <= 7) return false;

    const ReflectMap* reflect = nullptr;
    uint8_t *start_ptr = nullptr;
    // check access region
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
    else return false;

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
        const auto& info = it->second;
        uint8_t *ptr = start_ptr + info.second;
        switch(info.first)
        {
            case Reflection::ProtoFieldType::FLOAT:
            {
                if(!uavcan_register_Value_1_0_is_real32_(&value)) return false;
                if(value.real32.value.count != 1) return false;
                float temp = 0.0f;
                memcpy(&temp, value.real32.value.elements, sizeof(float));
                *(float*)ptr = temp;
                return true;
            }
            case Reflection::ProtoFieldType::INT32:
            {
                if(uavcan_register_Value_1_0_is_integer32_(&value))
                {
                    if(value.integer32.value.count != 1) return false;
                    int32_t temp = 0;
                    memcpy(&temp, value.integer32.value.elements, sizeof(int32_t));
                    *(int32_t*)ptr = temp;
                    return true;
                }
                if(uavcan_register_Value_1_0_is_integer16_(&value))
                {
                    if(value.integer16.value.count != 1) return false;
                    int16_t temp = 0;
                    memcpy(&temp, value.integer16.value.elements, sizeof(int16_t));
                    *(int32_t*)ptr = (int32_t)temp;
                    return true;
                }
                if(uavcan_register_Value_1_0_is_integer8_(&value))
                {
                    if(value.integer8.value.count != 1) return false;
                    int8_t temp = 0;
                    memcpy(&temp, value.integer8.value.elements, sizeof(int8_t));
                    *(int32_t*)ptr = (int32_t)temp;
                    return true;
                }
                return false;
            }
            case Reflection::ProtoFieldType::INT64:
            {
                if(!uavcan_register_Value_1_0_is_integer64_(&value)) return false;
                if(value.integer64.value.count != 1) return false;
                int64_t temp = 0;
                memcpy(&temp, value.integer64.value.elements, sizeof(int64_t));
                *(int64_t*)ptr = temp;
                return true;
            }
            case Reflection::ProtoFieldType::UINT32:
            {
                if(uavcan_register_Value_1_0_is_natural32_(&value))
                {
                    if(value.natural32.value.count != 1) return false;
                    uint32_t temp = 0;
                    memcpy(&temp, value.natural32.value.elements, sizeof(uint32_t));
                    *(uint32_t*)ptr = temp;
                    return true;
                }
                if(uavcan_register_Value_1_0_is_natural16_(&value))
                {
                    if(value.natural16.value.count != 1) return false;
                    uint16_t temp = 0;
                    memcpy(&temp, value.natural16.value.elements, sizeof(uint16_t));
                    *(uint32_t*)ptr = (uint32_t)temp;
                    return true;
                }
                if(uavcan_register_Value_1_0_is_natural8_(&value))
                {
                    if(value.natural8.value.count != 1) return false;
                    uint8_t temp = 0;
                    memcpy(&temp, value.natural8.value.elements, sizeof(uint8_t));
                    *(uint32_t*)ptr = (uint32_t)temp;
                    return true;
                }
                return false;
            }
            case Reflection::ProtoFieldType::UINT64:
            {
                if(!uavcan_register_Value_1_0_is_natural64_(&value)) return false;
                if(value.natural64.value.count != 1) return false;
                uint64_t temp = 0;
                memcpy(&temp, value.natural64.value.elements, sizeof(uint64_t));
                *(uint64_t*)ptr = temp;
                return true;
            }
            case Reflection::ProtoFieldType::BOOL:
            {
                if(!uavcan_register_Value_1_0_is_natural8_(&value)) return false;
                if(value.natural8.value.count != 1) return false;
                uint8_t temp = 0;
                memcpy(&temp, value.natural8.value.elements, sizeof(uint8_t));
                *(uint8_t*)ptr = temp;
                return true;
            }
            default: break;
        }
    }
    return false;
}

CyphalProtocol::PollingTask::PollingTask(CyphalProtocol* p) : Task("UAVCANPoll"), parent(p)
{
    RegisterTask(TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 4;
    config.stack_depth = 1024;
}

void CyphalProtocol::PollingTask::UpdateNormal()
{
    const auto motor = parent->GetMotor<FOCMotor>();
    const auto new_node_id = motor->GetConfig().node_id();
    if(new_node_id != parent->canard.node_id)
    {
        parent->canard.node_id = new_node_id;
        if(new_node_id <= CANARD_NODE_ID_MAX)
        {
            CanardFilter filter = canardMakeFilterForServices(new_node_id);
            parent->can->SetHWFilter(motor->GetInternalID(), filter.extended_can_id, filter.extended_mask);
        }
    }
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
            parent->rx_frame_received++;
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
                                            auto* const parent = static_cast<CyphalProtocol*>(ref);
                                            return parent->TransmitFrame(frame);
                                       },
                                       &parent->tx_frame_expired,
                                       &parent->tx_frame_failed
                                      );
    if(ret > 0) parent->tx_frame_sent++;
    // sleep(1);
}

void CyphalProtocol::OnRxEvent(const DataType::Comm::CANMessage& message)
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

FuncRetCode CyphalProtocol::SubscribeTransfer(CanardTransferKind kind, CanardPortID port, size_t max_size,
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
     * In case of preventing repeated duplicated subscriptions, we use custom vector(x) list to store the existing subscriptions.
     */
    // IMPORTANT: SUBSCRIPTION INSTANCES SHALL NOT BE MOVED WHILE IN USE.
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

FuncRetCode CyphalProtocol::UnsubscribeTransfer(CanardTransferKind kind, CanardPortID port)
{
    return FuncRetCode::NOT_SUPPORTED;
}

int8_t CyphalProtocol::TransmitFrame(CanardMutableFrame* frame) const
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

void* CyphalProtocol::canard_mem_alloc(void* ref, size_t size)
{
    (void)ref;
    return pvPortMalloc(size);
}

void CyphalProtocol::canard_mem_free(void* ref, size_t size, void* ptr)
{
    (void)ref;
    (void)size;
    vPortFree(ptr);
}
}
