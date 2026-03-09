#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETDEBUGCMD_REQUEST_MAX_SIZE 6
#define IFOC_SETDEBUGCMD_REQUEST_SIGNATURE (0xC45308F27B99D605ULL)
#define IFOC_SETDEBUGCMD_REQUEST_ID 120

struct ifoc_SetDebugCmdRequest {
    float phase_a_duty;
    float phase_b_duty;
    float phase_c_duty;
};

uint32_t ifoc_SetDebugCmdRequest_encode(struct ifoc_SetDebugCmdRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetDebugCmdRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetDebugCmdRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetDebugCmdRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdRequest* msg, bool tao);
static inline bool _ifoc_SetDebugCmdRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdRequest* msg, bool tao);
void _ifoc_SetDebugCmdRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->phase_a_duty);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->phase_b_duty);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->phase_c_duty);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

/*
 decode ifoc_SetDebugCmdRequest, return true on failure, false on success
*/
bool _ifoc_SetDebugCmdRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetDebugCmdRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->phase_a_duty = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->phase_b_duty = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->phase_c_duty = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetDebugCmdRequest sample_ifoc_SetDebugCmdRequest_msg(void);
#endif
