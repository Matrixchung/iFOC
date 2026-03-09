#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_ENCODER_MAX_SIZE 32
#define IFOC_ENCODER_SIGNATURE (0x813B23036978C201ULL)

#define IFOC_ENCODER_TYPE_ABSOLUTE_ENCODER 0
#define IFOC_ENCODER_TYPE_INCREMENTAL_ENCODER 1
#define IFOC_ENCODER_TYPE_SENSORLESS_ENCODER 2

struct ifoc_Encoder {
    struct { uint8_t len; uint8_t data[16]; }name;
    uint8_t type;
    bool primary;
    bool result_valid;
    float single_round_angle_rad;
    float multi_round_angle_rad;
    float angular_speed_rad_s;
    int64_t full_rotations;
};

uint32_t ifoc_Encoder_encode(struct ifoc_Encoder* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_Encoder_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_Encoder* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_Encoder_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_Encoder* msg, bool tao);
static inline bool _ifoc_Encoder_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_Encoder* msg, bool tao);
void _ifoc_Encoder_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_Encoder* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t name_len = msg->name.len > 16 ? 16 : msg->name.len;
#pragma GCC diagnostic pop
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &name_len);
    *bit_ofs += 5;
    for (size_t i=0; i < name_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->name.data[i]);
        *bit_ofs += 8;
    }
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 2, &msg->type);
    *bit_ofs += 2;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->primary);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->result_valid);
    *bit_ofs += 1;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->single_round_angle_rad);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->multi_round_angle_rad);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->angular_speed_rad_s);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 64, &msg->full_rotations);
    *bit_ofs += 64;
}

/*
 decode ifoc_Encoder, return true on failure, false on success
*/
bool _ifoc_Encoder_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_Encoder* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->name.len);
    *bit_ofs += 5;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->name.len > 16) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->name.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->name.data[i]);
        *bit_ofs += 8;
    }

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 2, false, &msg->type);
    *bit_ofs += 2;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->primary);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->result_valid);
    *bit_ofs += 1;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->single_round_angle_rad = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->multi_round_angle_rad = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->angular_speed_rad_s = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 64, true, &msg->full_rotations);
    *bit_ofs += 64;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_Encoder sample_ifoc_Encoder_msg(void);
#endif
