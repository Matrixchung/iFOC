#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "Motion.h"


#define IFOC_GETTARGETMOTION_RESPONSE_MAX_SIZE 6
#define IFOC_GETTARGETMOTION_RESPONSE_SIGNATURE (0x43E26AEB03DF665FULL)
#define IFOC_GETTARGETMOTION_RESPONSE_ID 115

struct ifoc_GetTargetMotionResponse {
    struct ifoc_Motion target;
};

uint32_t ifoc_GetTargetMotionResponse_encode(struct ifoc_GetTargetMotionResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetTargetMotionResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetTargetMotionResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetTargetMotionResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionResponse* msg, bool tao);
static inline bool _ifoc_GetTargetMotionResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionResponse* msg, bool tao);
void _ifoc_GetTargetMotionResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _ifoc_Motion_encode(buffer, bit_ofs, &msg->target, tao);
}

/*
 decode ifoc_GetTargetMotionResponse, return true on failure, false on success
*/
bool _ifoc_GetTargetMotionResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_ifoc_Motion_decode(transfer, bit_ofs, &msg->target, tao)) {return true;}

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetTargetMotionResponse sample_ifoc_GetTargetMotionResponse_msg(void);
#endif
