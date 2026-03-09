#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETVELTARGET_REQUEST_MAX_SIZE 6
#define IFOC_SETVELTARGET_REQUEST_SIGNATURE (0xB6D72D27204ECDBBULL)
#define IFOC_SETVELTARGET_REQUEST_ID 112

struct ifoc_SetVelTargetRequest {
    float target;
    int16_t torque_pu;
    bool torque_ff;
};

uint32_t ifoc_SetVelTargetRequest_encode(struct ifoc_SetVelTargetRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetVelTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetVelTargetRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetVelTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetVelTargetRequest* msg, bool tao);
static inline bool _ifoc_SetVelTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetVelTargetRequest* msg, bool tao);
void _ifoc_SetVelTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetVelTargetRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 32, &msg->target);
    *bit_ofs += 32;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 10, &msg->torque_pu);
    *bit_ofs += 10;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->torque_ff);
    *bit_ofs += 1;
}

/*
 decode ifoc_SetVelTargetRequest, return true on failure, false on success
*/
bool _ifoc_SetVelTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetVelTargetRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->target);
    *bit_ofs += 32;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 10, true, &msg->torque_pu);
    *bit_ofs += 10;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->torque_ff);
    *bit_ofs += 1;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetVelTargetRequest sample_ifoc_SetVelTargetRequest_msg(void);
#endif
