#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_CLEARERROR_REQUEST_MAX_SIZE 8
#define IFOC_CLEARERROR_REQUEST_SIGNATURE (0xC0CF429A8AFB1F6BULL)
#define IFOC_CLEARERROR_REQUEST_ID 202

struct ifoc_ClearErrorRequest {
    uint64_t clear_mask;
};

uint32_t ifoc_ClearErrorRequest_encode(struct ifoc_ClearErrorRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_ClearErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_ClearErrorRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_ClearErrorRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_ClearErrorRequest* msg, bool tao);
static inline bool _ifoc_ClearErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_ClearErrorRequest* msg, bool tao);
void _ifoc_ClearErrorRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_ClearErrorRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 64, &msg->clear_mask);
    *bit_ofs += 64;
}

/*
 decode ifoc_ClearErrorRequest, return true on failure, false on success
*/
bool _ifoc_ClearErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_ClearErrorRequest* msg, bool tao) {
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
struct ifoc_ClearErrorRequest sample_ifoc_ClearErrorRequest_msg(void);
#endif
