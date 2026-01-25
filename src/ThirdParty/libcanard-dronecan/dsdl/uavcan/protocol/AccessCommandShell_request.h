#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_MAX_SIZE 130
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_SIGNATURE (0x59276B5921C9246EULL)
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_ID 6

#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_NEWLINE 10
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_MIN_OUTPUT_LIFETIME_SEC 10
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_FLAG_RESET_SHELL 1
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_FLAG_CLEAR_OUTPUT_BUFFERS 2
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_FLAG_READ_STDOUT 64
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_REQUEST_FLAG_READ_STDERR 128

struct uavcan_protocol_AccessCommandShellRequest {
    uint8_t flags;
    struct { uint8_t len; uint8_t data[128]; }input;
};

uint32_t uavcan_protocol_AccessCommandShellRequest_encode(struct uavcan_protocol_AccessCommandShellRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_AccessCommandShellRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_AccessCommandShellRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_AccessCommandShellRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellRequest* msg, bool tao);
static inline bool _uavcan_protocol_AccessCommandShellRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellRequest* msg, bool tao);
void _uavcan_protocol_AccessCommandShellRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->flags);
    *bit_ofs += 8;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t input_len = msg->input.len > 128 ? 128 : msg->input.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &input_len);
        *bit_ofs += 8;
    }
    for (size_t i=0; i < input_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->input.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode uavcan_protocol_AccessCommandShellRequest, return true on failure, false on success
*/
bool _uavcan_protocol_AccessCommandShellRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->flags);
    *bit_ofs += 8;

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->input.len);
        *bit_ofs += 8;
    } else {
        msg->input.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->input.len > 128) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->input.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->input.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_AccessCommandShellRequest sample_uavcan_protocol_AccessCommandShellRequest_msg(void);
#endif
