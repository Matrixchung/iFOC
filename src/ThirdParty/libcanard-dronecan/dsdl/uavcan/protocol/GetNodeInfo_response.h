#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "HardwareVersion.h"
#include "NodeStatus.h"
#include "SoftwareVersion.h"


#define UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE 377
#define UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE (0xEE468A8121C46A9EULL)
#define UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID 1

struct uavcan_protocol_GetNodeInfoResponse {
    struct uavcan_protocol_NodeStatus status;
    struct uavcan_protocol_SoftwareVersion software_version;
    struct uavcan_protocol_HardwareVersion hardware_version;
    struct { uint8_t len; uint8_t data[80]; }name;
};

uint32_t uavcan_protocol_GetNodeInfoResponse_encode(struct uavcan_protocol_GetNodeInfoResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_GetNodeInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_GetNodeInfoResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_GetNodeInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao);
static inline bool _uavcan_protocol_GetNodeInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao);
void _uavcan_protocol_GetNodeInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_NodeStatus_encode(buffer, bit_ofs, &msg->status, false);
    _uavcan_protocol_SoftwareVersion_encode(buffer, bit_ofs, &msg->software_version, false);
    _uavcan_protocol_HardwareVersion_encode(buffer, bit_ofs, &msg->hardware_version, false);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t name_len = msg->name.len > 80 ? 80 : msg->name.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &name_len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < name_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->name.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode uavcan_protocol_GetNodeInfoResponse, return true on failure, false on success
*/
bool _uavcan_protocol_GetNodeInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_uavcan_protocol_NodeStatus_decode(transfer, bit_ofs, &msg->status, false)) {return true;}

    if (_uavcan_protocol_SoftwareVersion_decode(transfer, bit_ofs, &msg->software_version, false)) {return true;}

    if (_uavcan_protocol_HardwareVersion_decode(transfer, bit_ofs, &msg->hardware_version, false)) {return true;}

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->name.len);
        *bit_ofs += 7;
    } else {
        msg->name.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->name.len > 80) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->name.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->name.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_GetNodeInfoResponse sample_uavcan_protocol_GetNodeInfoResponse_msg(void);
#endif
