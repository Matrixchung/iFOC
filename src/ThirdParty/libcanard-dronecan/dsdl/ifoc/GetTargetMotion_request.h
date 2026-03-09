#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETTARGETMOTION_REQUEST_MAX_SIZE 0
#define IFOC_GETTARGETMOTION_REQUEST_SIGNATURE (0x43E26AEB03DF665FULL)
#define IFOC_GETTARGETMOTION_REQUEST_ID 115

struct ifoc_GetTargetMotionRequest {
};

uint32_t ifoc_GetTargetMotionRequest_encode(struct ifoc_GetTargetMotionRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetTargetMotionRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetTargetMotionRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetTargetMotionRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionRequest* msg, bool tao);
static inline bool _ifoc_GetTargetMotionRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionRequest* msg, bool tao);
void _ifoc_GetTargetMotionRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_GetTargetMotionRequest, return true on failure, false on success
*/
bool _ifoc_GetTargetMotionRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTargetMotionRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetTargetMotionRequest sample_ifoc_GetTargetMotionRequest_msg(void);
#endif
