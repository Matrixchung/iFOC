#define CANARD_DSDLC_INTERNAL
#include "Encoder.h"
#include <string.h>

using namespace DroneCAN;

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t ifoc_Encoder_encode(struct ifoc_Encoder* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, IFOC_ENCODER_MAX_SIZE);
    _ifoc_Encoder_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool ifoc_Encoder_decode(const CanardRxTransfer* transfer, struct ifoc_Encoder* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > IFOC_ENCODER_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_ifoc_Encoder_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_Encoder sample_ifoc_Encoder_msg(void) {
    struct ifoc_Encoder msg;

    msg.name.len = (uint8_t)random_range_unsigned_val(0, 16);
    for (size_t i=0; i < msg.name.len; i++) {
        msg.name.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    msg.type = (uint8_t)random_bitlen_unsigned_val(2);
    msg.primary = (bool)random_bitlen_unsigned_val(1);
    msg.result_valid = (bool)random_bitlen_unsigned_val(1);
    msg.single_round_angle_rad = random_float16_val();
    msg.multi_round_angle_rad = random_float16_val();
    msg.angular_speed_rad_s = random_float16_val();
    msg.full_rotations = (int64_t)random_bitlen_signed_val(64);
    return msg;
}
#endif
