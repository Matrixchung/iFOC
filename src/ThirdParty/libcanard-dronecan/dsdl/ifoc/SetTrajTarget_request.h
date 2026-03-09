#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETTRAJTARGET_REQUEST_MAX_SIZE 5
#define IFOC_SETTRAJTARGET_REQUEST_SIGNATURE (0x6CD673D9D7A2C14FULL)
#define IFOC_SETTRAJTARGET_REQUEST_ID 110

struct ifoc_SetTrajTargetRequest {
    float target;
    bool relative;
    bool rel_curr_based;
    bool s_curve;
};

uint32_t ifoc_SetTrajTargetRequest_encode(struct ifoc_SetTrajTargetRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetTrajTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetTrajTargetRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetTrajTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetRequest* msg, bool tao);
static inline bool _ifoc_SetTrajTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetRequest* msg, bool tao);
void _ifoc_SetTrajTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 32, &msg->target);
    *bit_ofs += 32;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->relative);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->rel_curr_based);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->s_curve);
    *bit_ofs += 1;
}

/*
 decode ifoc_SetTrajTargetRequest, return true on failure, false on success
*/
bool _ifoc_SetTrajTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->target);
    *bit_ofs += 32;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->relative);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->rel_curr_based);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->s_curve);
    *bit_ofs += 1;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetTrajTargetRequest sample_ifoc_SetTrajTargetRequest_msg(void);
#endif
