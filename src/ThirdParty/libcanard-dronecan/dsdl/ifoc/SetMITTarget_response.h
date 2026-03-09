#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETMITTARGET_RESPONSE_MAX_SIZE 0
#define IFOC_SETMITTARGET_RESPONSE_SIGNATURE (0xF45159FFC4798FF1ULL)
#define IFOC_SETMITTARGET_RESPONSE_ID 109

struct ifoc_SetMITTargetResponse {
};

uint32_t ifoc_SetMITTargetResponse_encode(struct ifoc_SetMITTargetResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetMITTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetMITTargetResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetMITTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetMITTargetResponse* msg, bool tao);
static inline bool _ifoc_SetMITTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetMITTargetResponse* msg, bool tao);
void _ifoc_SetMITTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetMITTargetResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_SetMITTargetResponse, return true on failure, false on success
*/
bool _ifoc_SetMITTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetMITTargetResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetMITTargetResponse sample_ifoc_SetMITTargetResponse_msg(void);
#endif
