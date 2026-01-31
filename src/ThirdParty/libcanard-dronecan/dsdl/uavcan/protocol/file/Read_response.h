#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "Error.h"


#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_MAX_SIZE 260
#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_SIGNATURE (0x8DCDCA939F33F678ULL)
#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_ID 48

struct uavcan_protocol_file_ReadResponse {
    struct uavcan_protocol_file_Error error;
    struct { uint16_t len; uint8_t data[256]; }data;
};

uint32_t uavcan_protocol_file_ReadResponse_encode(struct uavcan_protocol_file_ReadResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_file_ReadResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_file_ReadResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_file_ReadResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao);
static inline bool _uavcan_protocol_file_ReadResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao);
void _uavcan_protocol_file_ReadResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_file_Error_encode(buffer, bit_ofs, &msg->error, false);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint16_t data_len = msg->data.len > 256 ? 256 : msg->data.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 9, &data_len);
        *bit_ofs += 9;
    }
    for (size_t i=0; i < data_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode uavcan_protocol_file_ReadResponse, return true on failure, false on success
*/
bool _uavcan_protocol_file_ReadResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_uavcan_protocol_file_Error_decode(transfer, bit_ofs, &msg->error, false)) {return true;}

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 9, false, &msg->data.len);
        *bit_ofs += 9;
    } else {
        msg->data.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->data.len > 256) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->data.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_ReadResponse sample_uavcan_protocol_file_ReadResponse_msg(void);
#endif
