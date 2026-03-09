#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "Motion.h"


#define IFOC_GETCURRENTMOTION_RESPONSE_MAX_SIZE 6
#define IFOC_GETCURRENTMOTION_RESPONSE_SIGNATURE (0xA75C0DC7709D1754ULL)
#define IFOC_GETCURRENTMOTION_RESPONSE_ID 116

struct ifoc_GetCurrentMotionResponse {
    struct ifoc_Motion current;
};

uint32_t ifoc_GetCurrentMotionResponse_encode(struct ifoc_GetCurrentMotionResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetCurrentMotionResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetCurrentMotionResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetCurrentMotionResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionResponse* msg, bool tao);
static inline bool _ifoc_GetCurrentMotionResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionResponse* msg, bool tao);
void _ifoc_GetCurrentMotionResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _ifoc_Motion_encode(buffer, bit_ofs, &msg->current, tao);
}

/*
 decode ifoc_GetCurrentMotionResponse, return true on failure, false on success
*/
bool _ifoc_GetCurrentMotionResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetCurrentMotionResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_ifoc_Motion_decode(transfer, bit_ofs, &msg->current, tao)) {return true;}

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetCurrentMotionResponse sample_ifoc_GetCurrentMotionResponse_msg(void);
#endif
