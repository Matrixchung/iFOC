#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "ReferenceFrame.h"


#define IFOC_SETREFFRAME_REQUEST_MAX_SIZE 1
#define IFOC_SETREFFRAME_REQUEST_SIGNATURE (0xDDB2D2D9FCFA1CAEULL)
#define IFOC_SETREFFRAME_REQUEST_ID 103

struct ifoc_SetRefFrameRequest {
    struct ifoc_ReferenceFrame set_ref;
};

uint32_t ifoc_SetRefFrameRequest_encode(struct ifoc_SetRefFrameRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetRefFrameRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetRefFrameRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetRefFrameRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetRefFrameRequest* msg, bool tao);
static inline bool _ifoc_SetRefFrameRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetRefFrameRequest* msg, bool tao);
void _ifoc_SetRefFrameRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetRefFrameRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _ifoc_ReferenceFrame_encode(buffer, bit_ofs, &msg->set_ref, tao);
}

/*
 decode ifoc_SetRefFrameRequest, return true on failure, false on success
*/
bool _ifoc_SetRefFrameRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetRefFrameRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (_ifoc_ReferenceFrame_decode(transfer, bit_ofs, &msg->set_ref, tao)) {return true;}

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetRefFrameRequest sample_ifoc_SetRefFrameRequest_msg(void);
#endif
