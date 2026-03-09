#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETTORQUETARGET_REQUEST_MAX_SIZE 4
#define IFOC_SETTORQUETARGET_REQUEST_SIGNATURE (0x647A77E42ED5127AULL)
#define IFOC_SETTORQUETARGET_REQUEST_ID 113

struct ifoc_SetTorqueTargetRequest {
    float target;
};

uint32_t ifoc_SetTorqueTargetRequest_encode(struct ifoc_SetTorqueTargetRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetTorqueTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetTorqueTargetRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetTorqueTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetRequest* msg, bool tao);
static inline bool _ifoc_SetTorqueTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetRequest* msg, bool tao);
void _ifoc_SetTorqueTargetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 32, &msg->target);
    *bit_ofs += 32;
}

/*
 decode ifoc_SetTorqueTargetRequest, return true on failure, false on success
*/
bool _ifoc_SetTorqueTargetRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->target);
    *bit_ofs += 32;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetTorqueTargetRequest sample_ifoc_SetTorqueTargetRequest_msg(void);
#endif
