#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETDEBUGCMD_RESPONSE_MAX_SIZE 0
#define IFOC_SETDEBUGCMD_RESPONSE_SIGNATURE (0xC45308F27B99D605ULL)
#define IFOC_SETDEBUGCMD_RESPONSE_ID 120

struct ifoc_SetDebugCmdResponse {
};

uint32_t ifoc_SetDebugCmdResponse_encode(struct ifoc_SetDebugCmdResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetDebugCmdResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetDebugCmdResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetDebugCmdResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdResponse* msg, bool tao);
static inline bool _ifoc_SetDebugCmdResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdResponse* msg, bool tao);
void _ifoc_SetDebugCmdResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_SetDebugCmdResponse, return true on failure, false on success
*/
bool _ifoc_SetDebugCmdResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetDebugCmdResponse sample_ifoc_SetDebugCmdResponse_msg(void);
#endif
