#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETPOSTARGET_REQUEST_MAX_SIZE 7
#define IFOC_SETPOSTARGET_REQUEST_SIGNATURE (0xF9C5FB30FE7CF1CFULL)
#define IFOC_SETPOSTARGET_REQUEST_ID 111

struct ifoc_SetPosTargetRequest {
    float target;
    int16_t velocity_pu;
    int16_t torque_pu;
    bool relative;
    bool rel_curr_based;
    bool vel_tor_ff;
};

uint32_t ifoc_SetPosTargetRequest_encode(struct ifoc_SetPosTargetRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetPosTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetPosTargetRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetPosTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetPosTargetRequest* msg, bool tao);
static inline bool _ifoc_SetPosTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetPosTargetRequest* msg, bool tao);
void _ifoc_SetPosTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetPosTargetRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 32, &msg->target);
    *bit_ofs += 32;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 10, &msg->velocity_pu);
    *bit_ofs += 10;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 10, &msg->torque_pu);
    *bit_ofs += 10;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->relative);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->rel_curr_based);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->vel_tor_ff);
    *bit_ofs += 1;
}

/*
 decode ifoc_SetPosTargetRequest, return true on failure, false on success
*/
bool _ifoc_SetPosTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetPosTargetRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->target);
    *bit_ofs += 32;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 10, true, &msg->velocity_pu);
    *bit_ofs += 10;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 10, true, &msg->torque_pu);
    *bit_ofs += 10;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->relative);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->rel_curr_based);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->vel_tor_ff);
    *bit_ofs += 1;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetPosTargetRequest sample_ifoc_SetPosTargetRequest_msg(void);
#endif
