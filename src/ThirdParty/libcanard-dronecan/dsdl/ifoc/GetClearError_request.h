#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETCLEARERROR_REQUEST_MAX_SIZE 8
#define IFOC_GETCLEARERROR_REQUEST_SIGNATURE (0xC0CF429A8AFB1F6BULL)
#define IFOC_GETCLEARERROR_REQUEST_ID 101

struct ifoc_GetClearErrorRequest {
    uint64_t clear_mask;
};

uint32_t ifoc_GetClearErrorRequest_encode(struct ifoc_GetClearErrorRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetClearErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetClearErrorRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetClearErrorRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetClearErrorRequest* msg, bool tao);
static inline bool _ifoc_GetClearErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetClearErrorRequest* msg, bool tao);
void _ifoc_GetClearErrorRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetClearErrorRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 64, &msg->clear_mask);
    *bit_ofs += 64;
}

/*
 decode ifoc_GetClearErrorRequest, return true on failure, false on success
*/
bool _ifoc_GetClearErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetClearErrorRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 64, false, &msg->clear_mask);
    *bit_ofs += 64;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetClearErrorRequest sample_ifoc_GetClearErrorRequest_msg(void);
#endif
