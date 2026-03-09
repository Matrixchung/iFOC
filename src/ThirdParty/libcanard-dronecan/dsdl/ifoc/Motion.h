#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_MOTION_MAX_SIZE 6
#define IFOC_MOTION_SIGNATURE (0xA547CF72C30D98EEULL)

struct ifoc_Motion {
    float torque;
    float speed;
    float pos;
};

uint32_t ifoc_Motion_encode(struct ifoc_Motion* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_Motion_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_Motion* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_Motion_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_Motion* msg, bool tao);
static inline bool _ifoc_Motion_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_Motion* msg, bool tao);
void _ifoc_Motion_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_Motion* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->torque);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->speed);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->pos);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

/*
 decode ifoc_Motion, return true on failure, false on success
*/
bool _ifoc_Motion_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_Motion* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->torque = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->speed = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->pos = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_Motion sample_ifoc_Motion_msg(void);
#endif
