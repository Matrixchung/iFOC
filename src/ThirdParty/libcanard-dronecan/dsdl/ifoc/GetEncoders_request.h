#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETENCODERS_REQUEST_MAX_SIZE 0
#define IFOC_GETENCODERS_REQUEST_SIGNATURE (0xCA3F930D7927EFF1ULL)
#define IFOC_GETENCODERS_REQUEST_ID 117

struct ifoc_GetEncodersRequest {
};

uint32_t ifoc_GetEncodersRequest_encode(struct ifoc_GetEncodersRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetEncodersRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetEncodersRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetEncodersRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetEncodersRequest* msg, bool tao);
static inline bool _ifoc_GetEncodersRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetEncodersRequest* msg, bool tao);
void _ifoc_GetEncodersRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetEncodersRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_GetEncodersRequest, return true on failure, false on success
*/
bool _ifoc_GetEncodersRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetEncodersRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetEncodersRequest sample_ifoc_GetEncodersRequest_msg(void);
#endif
