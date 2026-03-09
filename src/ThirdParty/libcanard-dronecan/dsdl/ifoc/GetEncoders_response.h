#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "Encoder.h"


#define IFOC_GETENCODERS_RESPONSE_MAX_SIZE 188
#define IFOC_GETENCODERS_RESPONSE_SIGNATURE (0xCA3F930D7927EFF1ULL)
#define IFOC_GETENCODERS_RESPONSE_ID 117

struct ifoc_GetEncodersResponse {
    struct { uint8_t len; struct ifoc_Encoder data[6]; }encoders;
};

uint32_t ifoc_GetEncodersResponse_encode(struct ifoc_GetEncodersResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetEncodersResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetEncodersResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetEncodersResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetEncodersResponse* msg, bool tao);
static inline bool _ifoc_GetEncodersResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetEncodersResponse* msg, bool tao);
void _ifoc_GetEncodersResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetEncodersResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t encoders_len = msg->encoders.len > 6 ? 6 : msg->encoders.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 3, &encoders_len);
        *bit_ofs += 3;
    }
    for (size_t i=0; i < encoders_len; i++) {
        _ifoc_Encoder_encode(buffer, bit_ofs, &msg->encoders.data[i], false);
    }
}

/*
 decode ifoc_GetEncodersResponse, return true on failure, false on success
*/
bool _ifoc_GetEncodersResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetEncodersResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->encoders.len);
        *bit_ofs += 3;
    }


    if (tao) {
        msg->encoders.len = 0;
        size_t max_len = 6;
        uint32_t max_bits = (transfer->payload_len*8)-7; // TAO elements must be >= 8 bits
        while (max_bits > *bit_ofs) {
            if (!max_len-- || _ifoc_Encoder_decode(transfer, bit_ofs, &msg->encoders.data[msg->encoders.len], false)) {return true;}
            msg->encoders.len++;
        }
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->encoders.len > 6) {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i=0; i < msg->encoders.len; i++) {
            if (_ifoc_Encoder_decode(transfer, bit_ofs, &msg->encoders.data[i], false)) {return true;}
        }
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetEncodersResponse sample_ifoc_GetEncodersResponse_msg(void);
#endif
