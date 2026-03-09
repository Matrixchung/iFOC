#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "ReferenceFrame.h"


#define IFOC_SETREFFRAME_RESPONSE_MAX_SIZE 1
#define IFOC_SETREFFRAME_RESPONSE_SIGNATURE (0xDDB2D2D9FCFA1CAEULL)
#define IFOC_SETREFFRAME_RESPONSE_ID 103

struct ifoc_SetRefFrameResponse {
    struct ifoc_ReferenceFrame get_ref;
};

uint32_t ifoc_SetRefFrameResponse_encode(struct ifoc_SetRefFrameResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetRefFrameResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetRefFrameResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetRefFrameResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetRefFrameResponse* msg, bool tao);
static inline bool _ifoc_SetRefFrameResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetRefFrameResponse* msg, bool tao);
void _ifoc_SetRefFrameResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetRefFrameResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _ifoc_ReferenceFrame_encode(buffer, bit_ofs, &msg->get_ref, tao);
}

/*
 decode ifoc_SetRefFrameResponse, return true on failure, false on success
*/
bool _ifoc_SetRefFrameResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetRefFrameResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_ifoc_ReferenceFrame_decode(transfer, bit_ofs, &msg->get_ref, tao)) {return true;}

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetRefFrameResponse sample_ifoc_SetRefFrameResponse_msg(void);
#endif
