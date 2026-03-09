#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETCURRENTMOTION_REQUEST_MAX_SIZE 0
#define IFOC_GETCURRENTMOTION_REQUEST_SIGNATURE (0xA75C0DC7709D1754ULL)
#define IFOC_GETCURRENTMOTION_REQUEST_ID 116

struct ifoc_GetCurrentMotionRequest {
};

uint32_t ifoc_GetCurrentMotionRequest_encode(struct ifoc_GetCurrentMotionRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetCurrentMotionRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetCurrentMotionRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetCurrentMotionRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionRequest* msg, bool tao);
static inline bool _ifoc_GetCurrentMotionRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionRequest* msg, bool tao);
void _ifoc_GetCurrentMotionRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_GetCurrentMotionRequest, return true on failure, false on success
*/
bool _ifoc_GetCurrentMotionRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetCurrentMotionRequest sample_ifoc_GetCurrentMotionRequest_msg(void);
#endif
