#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "Path.h"


#define UAVCAN_PROTOCOL_FILE_DELETE_REQUEST_MAX_SIZE 201
#define UAVCAN_PROTOCOL_FILE_DELETE_REQUEST_SIGNATURE (0x78648C99170B47AAULL)
#define UAVCAN_PROTOCOL_FILE_DELETE_REQUEST_ID 47

struct uavcan_protocol_file_DeleteRequest {
    struct uavcan_protocol_file_Path path;
};

uint32_t uavcan_protocol_file_DeleteRequest_encode(struct uavcan_protocol_file_DeleteRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_file_DeleteRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_file_DeleteRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_file_DeleteRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_DeleteRequest* msg, bool tao);
static inline bool _uavcan_protocol_file_DeleteRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_DeleteRequest* msg, bool tao);
void _uavcan_protocol_file_DeleteRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_DeleteRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_file_Path_encode(buffer, bit_ofs, &msg->path, tao);
}

/*
 decode uavcan_protocol_file_DeleteRequest, return true on failure, false on success
*/
bool _uavcan_protocol_file_DeleteRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_DeleteRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_uavcan_protocol_file_Path_decode(transfer, bit_ofs, &msg->path, tao)) {return true;}

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_DeleteRequest sample_uavcan_protocol_file_DeleteRequest_msg(void);
#endif
