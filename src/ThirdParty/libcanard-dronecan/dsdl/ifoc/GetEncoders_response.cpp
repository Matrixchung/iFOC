#define CANARD_DSDLC_INTERNAL
#include "GetEncoders_response.h"
#include <string.h>

using namespace DroneCAN;

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t ifoc_GetEncodersResponse_encode(struct ifoc_GetEncodersResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, IFOC_GETENCODERS_RESPONSE_MAX_SIZE);
    _ifoc_GetEncodersResponse_encode(buffer, &bit_ofs, msg, 
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
bool ifoc_GetEncodersResponse_decode(const CanardRxTransfer* transfer, struct ifoc_GetEncodersResponse* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > IFOC_GETENCODERS_RESPONSE_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_ifoc_GetEncodersResponse_decode(transfer, &bit_ofs, msg,
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
struct ifoc_GetEncodersResponse sample_ifoc_GetEncodersResponse_msg(void) {
    struct ifoc_GetEncodersResponse msg;

    msg.encoders.len = (uint8_t)random_range_unsigned_val(0, 6);
    for (size_t i=0; i < msg.encoders.len; i++) {
        msg.encoders.data[i] = sample_ifoc_Encoder_msg();
    }
    return msg;
}
#endif
