#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETTRAJTARGET_RESPONSE_MAX_SIZE 0
#define IFOC_SETTRAJTARGET_RESPONSE_SIGNATURE (0x6CD673D9D7A2C14FULL)
#define IFOC_SETTRAJTARGET_RESPONSE_ID 110

struct ifoc_SetTrajTargetResponse {
};

uint32_t ifoc_SetTrajTargetResponse_encode(struct ifoc_SetTrajTargetResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetTrajTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetTrajTargetResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetTrajTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetResponse* msg, bool tao);
static inline bool _ifoc_SetTrajTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetResponse* msg, bool tao);
void _ifoc_SetTrajTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_SetTrajTargetResponse, return true on failure, false on success
*/
bool _ifoc_SetTrajTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTrajTargetResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetTrajTargetResponse sample_ifoc_SetTrajTargetResponse_msg(void);
#endif
