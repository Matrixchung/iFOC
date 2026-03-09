#define CANARD_DSDLC_INTERNAL
#include "SetDebugCmd_request.h"
#include "SetDebugCmd_response.h"
#include <string.h>

using namespace DroneCAN;

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t ifoc_SetDebugCmdRequest_encode(struct ifoc_SetDebugCmdRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, IFOC_SETDEBUGCMD_REQUEST_MAX_SIZE);
    _ifoc_SetDebugCmdRequest_encode(buffer, &bit_ofs, msg, 
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
bool ifoc_SetDebugCmdRequest_decode(const CanardRxTransfer* transfer, struct ifoc_SetDebugCmdRequest* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > IFOC_SETDEBUGCMD_REQUEST_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_ifoc_SetDebugCmdRequest_decode(transfer, &bit_ofs, msg,
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
struct ifoc_SetDebugCmdRequest sample_ifoc_SetDebugCmdRequest_msg(void) {
    struct ifoc_SetDebugCmdRequest msg;

    msg.phase_a_duty = random_float16_val();
    msg.phase_b_duty = random_float16_val();
    msg.phase_c_duty = random_float16_val();
    return msg;
}
#endif
