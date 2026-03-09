#define CANARD_DSDLC_INTERNAL
#include "SetVelTarget_request.h"
#include "SetVelTarget_response.h"
#include <string.h>

using namespace DroneCAN;

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t ifoc_SetVelTargetRequest_encode(struct ifoc_SetVelTargetRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, IFOC_SETVELTARGET_REQUEST_MAX_SIZE);
    _ifoc_SetVelTargetRequest_encode(buffer, &bit_ofs, msg, 
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
bool ifoc_SetVelTargetRequest_decode(const CanardRxTransfer* transfer, struct ifoc_SetVelTargetRequest* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > IFOC_SETVELTARGET_REQUEST_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_ifoc_SetVelTargetRequest_decode(transfer, &bit_ofs, msg,
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
struct ifoc_SetVelTargetRequest sample_ifoc_SetVelTargetRequest_msg(void) {
    struct ifoc_SetVelTargetRequest msg;

    msg.target = random_float_val();
    msg.torque_pu = (int16_t)random_bitlen_signed_val(10);
    msg.torque_ff = (bool)random_bitlen_unsigned_val(1);
    return msg;
}
#endif
