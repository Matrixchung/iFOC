#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "EntryType.h"
#include "Error.h"


#define UAVCAN_PROTOCOL_FILE_GETINFO_RESPONSE_MAX_SIZE 8
#define UAVCAN_PROTOCOL_FILE_GETINFO_RESPONSE_SIGNATURE (0x5004891EE8A27531ULL)
#define UAVCAN_PROTOCOL_FILE_GETINFO_RESPONSE_ID 45

struct uavcan_protocol_file_GetInfoResponse {
    uint64_t size;
    struct uavcan_protocol_file_Error error;
    struct uavcan_protocol_file_EntryType entry_type;
};

uint32_t uavcan_protocol_file_GetInfoResponse_encode(struct uavcan_protocol_file_GetInfoResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_file_GetInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_file_GetInfoResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_file_GetInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoResponse* msg, bool tao);
static inline bool _uavcan_protocol_file_GetInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoResponse* msg, bool tao);
void _uavcan_protocol_file_GetInfoResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 40, &msg->size);
    *bit_ofs += 40;
    _uavcan_protocol_file_Error_encode(buffer, bit_ofs, &msg->error, false);
    _uavcan_protocol_file_EntryType_encode(buffer, bit_ofs, &msg->entry_type, tao);
}

/*
 decode uavcan_protocol_file_GetInfoResponse, return true on failure, false on success
*/
bool _uavcan_protocol_file_GetInfoResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 40, false, &msg->size);
    *bit_ofs += 40;

    if (_uavcan_protocol_file_Error_decode(transfer, bit_ofs, &msg->error, false)) {return true;}

    if (_uavcan_protocol_file_EntryType_decode(transfer, bit_ofs, &msg->entry_type, tao)) {return true;}

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_GetInfoResponse sample_uavcan_protocol_file_GetInfoResponse_msg(void);
#endif
