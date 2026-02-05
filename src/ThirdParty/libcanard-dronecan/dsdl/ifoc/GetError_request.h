#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETERROR_REQUEST_MAX_SIZE 0
#define IFOC_GETERROR_REQUEST_SIGNATURE (0xBEACCC1CDD5A6D91ULL)
#define IFOC_GETERROR_REQUEST_ID 201

struct ifoc_GetErrorRequest {
};

uint32_t ifoc_GetErrorRequest_encode(struct ifoc_GetErrorRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetErrorRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetErrorRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetErrorRequest* msg, bool tao);
static inline bool _ifoc_GetErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetErrorRequest* msg, bool tao);
void _ifoc_GetErrorRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetErrorRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_GetErrorRequest, return true on failure, false on success
*/
bool _ifoc_GetErrorRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetErrorRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetErrorRequest sample_ifoc_GetErrorRequest_msg(void);
#endif
