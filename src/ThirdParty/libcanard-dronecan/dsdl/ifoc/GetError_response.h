#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETERROR_RESPONSE_MAX_SIZE 8
#define IFOC_GETERROR_RESPONSE_SIGNATURE (0xBEACCC1CDD5A6D91ULL)
#define IFOC_GETERROR_RESPONSE_ID 201

struct ifoc_GetErrorResponse {
    uint64_t error;
};

uint32_t ifoc_GetErrorResponse_encode(struct ifoc_GetErrorResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetErrorResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetErrorResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetErrorResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetErrorResponse* msg, bool tao);
static inline bool _ifoc_GetErrorResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetErrorResponse* msg, bool tao);
void _ifoc_GetErrorResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetErrorResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 64, &msg->error);
    *bit_ofs += 64;
}

/*
 decode ifoc_GetErrorResponse, return true on failure, false on success
*/
bool _ifoc_GetErrorResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetErrorResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 64, false, &msg->error);
    *bit_ofs += 64;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetErrorResponse sample_ifoc_GetErrorResponse_msg(void);
#endif
