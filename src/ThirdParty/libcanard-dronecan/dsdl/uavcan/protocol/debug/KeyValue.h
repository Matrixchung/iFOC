#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MAX_SIZE 63
#define UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE (0xE02F25D6E0C98AE0ULL)
#define UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID 16370

struct uavcan_protocol_debug_KeyValue {
    float value;
    struct { uint8_t len; uint8_t data[58]; }key;
};

uint32_t uavcan_protocol_debug_KeyValue_encode(struct uavcan_protocol_debug_KeyValue* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_debug_KeyValue_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_debug_KeyValue* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_debug_KeyValue_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao);
static inline bool _uavcan_protocol_debug_KeyValue_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao);
void _uavcan_protocol_debug_KeyValue_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 32, &msg->value);
    *bit_ofs += 32;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t key_len = msg->key.len > 58 ? 58 : msg->key.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 6, &key_len);
        *bit_ofs += 6;
    }
    for (size_t i=0; i < key_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->key.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode uavcan_protocol_debug_KeyValue, return true on failure, false on success
*/
bool _uavcan_protocol_debug_KeyValue_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->value);
    *bit_ofs += 32;

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->key.len);
        *bit_ofs += 6;
    } else {
        msg->key.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->key.len > 58) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->key.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->key.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_debug_KeyValue sample_uavcan_protocol_debug_KeyValue_msg(void);
#endif
