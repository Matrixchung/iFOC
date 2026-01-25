#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "Protocol.h"


#define UAVCAN_TUNNEL_TARGETTED_MAX_SIZE 127
#define UAVCAN_TUNNEL_TARGETTED_SIGNATURE (0xB138E7EA72A2A2E9ULL)
#define UAVCAN_TUNNEL_TARGETTED_ID 3001

#define UAVCAN_TUNNEL_TARGETTED_OPTION_LOCK_PORT 1

struct uavcan_tunnel_Targetted {
    struct uavcan_tunnel_Protocol protocol;
    uint8_t target_node;
    int8_t serial_id;
    uint8_t options;
    uint32_t baudrate;
    struct { uint8_t len; uint8_t data[120]; }buffer;
};

uint32_t uavcan_tunnel_Targetted_encode(struct uavcan_tunnel_Targetted* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_tunnel_Targetted_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_tunnel_Targetted* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_tunnel_Targetted_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_tunnel_Targetted* msg, bool tao);
static inline bool _uavcan_tunnel_Targetted_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_tunnel_Targetted* msg, bool tao);
void _uavcan_tunnel_Targetted_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_tunnel_Targetted* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_tunnel_Protocol_encode(buffer, bit_ofs, &msg->protocol, false);
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->target_node);
    *bit_ofs += 7;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &msg->serial_id);
    *bit_ofs += 5;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 4, &msg->options);
    *bit_ofs += 4;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 24, &msg->baudrate);
    *bit_ofs += 24;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t buffer_len = msg->buffer.len > 120 ? 120 : msg->buffer.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &buffer_len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < buffer_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->buffer.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode uavcan_tunnel_Targetted, return true on failure, false on success
*/
bool _uavcan_tunnel_Targetted_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_tunnel_Targetted* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_uavcan_tunnel_Protocol_decode(transfer, bit_ofs, &msg->protocol, false)) {return true;}

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->target_node);
    *bit_ofs += 7;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, true, &msg->serial_id);
    *bit_ofs += 5;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->options);
    *bit_ofs += 4;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 24, false, &msg->baudrate);
    *bit_ofs += 24;

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->buffer.len);
        *bit_ofs += 7;
    } else {
        msg->buffer.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->buffer.len > 120) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->buffer.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->buffer.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_tunnel_Targetted sample_uavcan_tunnel_Targetted_msg(void);
#endif
