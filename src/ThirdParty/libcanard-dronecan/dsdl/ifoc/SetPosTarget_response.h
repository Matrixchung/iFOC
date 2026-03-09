#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETPOSTARGET_RESPONSE_MAX_SIZE 0
#define IFOC_SETPOSTARGET_RESPONSE_SIGNATURE (0xF9C5FB30FE7CF1CFULL)
#define IFOC_SETPOSTARGET_RESPONSE_ID 111

struct ifoc_SetPosTargetResponse {
};

uint32_t ifoc_SetPosTargetResponse_encode(struct ifoc_SetPosTargetResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetPosTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetPosTargetResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetPosTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetPosTargetResponse* msg, bool tao);
static inline bool _ifoc_SetPosTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetPosTargetResponse* msg, bool tao);
void _ifoc_SetPosTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetPosTargetResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_SetPosTargetResponse, return true on failure, false on success
*/
bool _ifoc_SetPosTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetPosTargetResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetPosTargetResponse sample_ifoc_SetPosTargetResponse_msg(void);
#endif
