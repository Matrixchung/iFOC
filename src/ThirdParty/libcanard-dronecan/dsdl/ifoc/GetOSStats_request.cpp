#define CANARD_DSDLC_INTERNAL
#include "GetOSStats_request.h"
#include "GetOSStats_response.h"
#include <string.h>

using namespace DroneCAN;

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t ifoc_GetOSStatsRequest_encode(struct ifoc_GetOSStatsRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, IFOC_GETOSSTATS_REQUEST_MAX_SIZE);
    _ifoc_GetOSStatsRequest_encode(buffer, &bit_ofs, msg, 
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
bool ifoc_GetOSStatsRequest_decode(const CanardRxTransfer* transfer, struct ifoc_GetOSStatsRequest* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > IFOC_GETOSSTATS_REQUEST_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_ifoc_GetOSStatsRequest_decode(transfer, &bit_ofs, msg,
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
struct ifoc_GetOSStatsRequest sample_ifoc_GetOSStatsRequest_msg(void) {
    struct ifoc_GetOSStatsRequest msg;

    return msg;
}
#endif
