#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETCLEARERRORINDEX_REQUEST_MAX_SIZE 65
#define IFOC_GETCLEARERRORINDEX_REQUEST_SIGNATURE (0xD4A800E76896100AULL)
#define IFOC_GETCLEARERRORINDEX_REQUEST_ID 102

struct ifoc_GetClearErrorIndexRequest {
    struct { uint8_t len; uint8_t data[64]; }clear_index;
};

uint32_t ifoc_GetClearErrorIndexRequest_encode(struct ifoc_GetClearErrorIndexRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetClearErrorIndexRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetClearErrorIndexRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetClearErrorIndexRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetClearErrorIndexRequest* msg, bool tao);
static inline bool _ifoc_GetClearErrorIndexRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetClearErrorIndexRequest* msg, bool tao);
void _ifoc_GetClearErrorIndexRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetClearErrorIndexRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t clear_index_len = msg->clear_index.len > 64 ? 64 : msg->clear_index.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &clear_index_len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < clear_index_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->clear_index.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode ifoc_GetClearErrorIndexRequest, return true on failure, false on success
*/
bool _ifoc_GetClearErrorIndexRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetClearErrorIndexRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->clear_index.len);
        *bit_ofs += 7;
    } else {
        msg->clear_index.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->clear_index.len > 64) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->clear_index.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->clear_index.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetClearErrorIndexRequest sample_ifoc_GetClearErrorIndexRequest_msg(void);
#endif
