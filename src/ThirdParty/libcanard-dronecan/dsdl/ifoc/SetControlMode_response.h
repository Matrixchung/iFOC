#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETCONTROLMODE_RESPONSE_MAX_SIZE 1
#define IFOC_SETCONTROLMODE_RESPONSE_SIGNATURE (0x7D6527915FC92C4AULL)
#define IFOC_SETCONTROLMODE_RESPONSE_ID 108

#define IFOC_SETCONTROLMODE_RESPONSE_CONTROL_MODE_POSITION 0
#define IFOC_SETCONTROLMODE_RESPONSE_CONTROL_MODE_VELOCITY 1
#define IFOC_SETCONTROLMODE_RESPONSE_CONTROL_MODE_CURRENT 2
#define IFOC_SETCONTROLMODE_RESPONSE_CONTROL_MODE_HYBRID 3

struct ifoc_SetControlModeResponse {
    uint8_t control_mode;
};

uint32_t ifoc_SetControlModeResponse_encode(struct ifoc_SetControlModeResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetControlModeResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetControlModeResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetControlModeResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetControlModeResponse* msg, bool tao);
static inline bool _ifoc_SetControlModeResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetControlModeResponse* msg, bool tao);
void _ifoc_SetControlModeResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetControlModeResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 2, &msg->control_mode);
    *bit_ofs += 2;
}

/*
 decode ifoc_SetControlModeResponse, return true on failure, false on success
*/
bool _ifoc_SetControlModeResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetControlModeResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 2, false, &msg->control_mode);
    *bit_ofs += 2;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetControlModeResponse sample_ifoc_SetControlModeResponse_msg(void);
#endif
