#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "DataTypeKind.h"


#define UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_MAX_SIZE 93
#define UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_SIGNATURE (0x1B283338A7BED2D8ULL)
#define UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_ID 2

#define UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_FLAG_KNOWN 1
#define UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_FLAG_SUBSCRIBED 2
#define UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_FLAG_PUBLISHING 4
#define UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_FLAG_SERVING 8

struct uavcan_protocol_GetDataTypeInfoResponse {
    uint64_t signature;
    uint16_t id;
    struct uavcan_protocol_DataTypeKind kind;
    uint8_t flags;
    struct { uint8_t len; uint8_t data[80]; }name;
};

uint32_t uavcan_protocol_GetDataTypeInfoResponse_encode(struct uavcan_protocol_GetDataTypeInfoResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_GetDataTypeInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_GetDataTypeInfoResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_GetDataTypeInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoResponse* msg, bool tao);
static inline bool _uavcan_protocol_GetDataTypeInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoResponse* msg, bool tao);
void _uavcan_protocol_GetDataTypeInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 64, &msg->signature);
    *bit_ofs += 64;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &msg->id);
    *bit_ofs += 16;
    _uavcan_protocol_DataTypeKind_encode(buffer, bit_ofs, &msg->kind, false);
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->flags);
    *bit_ofs += 8;
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
 decode uavcan_protocol_GetDataTypeInfoResponse, return true on failure, false on success
*/
bool _uavcan_protocol_GetDataTypeInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 64, false, &msg->signature);
    *bit_ofs += 64;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->id);
    *bit_ofs += 16;

    if (_uavcan_protocol_DataTypeKind_decode(transfer, bit_ofs, &msg->kind, false)) {return true;}

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->flags);
    *bit_ofs += 8;

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
struct uavcan_protocol_GetDataTypeInfoResponse sample_uavcan_protocol_GetDataTypeInfoResponse_msg(void);
#endif
