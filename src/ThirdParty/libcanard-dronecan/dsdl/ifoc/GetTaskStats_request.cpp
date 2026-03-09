#define CANARD_DSDLC_INTERNAL
#include "GetTaskStats_request.h"
#include "GetTaskStats_response.h"
#include <string.h>

using namespace DroneCAN;

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t ifoc_GetTaskStatsRequest_encode(struct ifoc_GetTaskStatsRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, IFOC_GETTASKSTATS_REQUEST_MAX_SIZE);
    _ifoc_GetTaskStatsRequest_encode(buffer, &bit_ofs, msg, 
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
bool ifoc_GetTaskStatsRequest_decode(const CanardRxTransfer* transfer, struct ifoc_GetTaskStatsRequest* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > IFOC_GETTASKSTATS_REQUEST_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_ifoc_GetTaskStatsRequest_decode(transfer, &bit_ofs, msg,
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
struct ifoc_GetTaskStatsRequest sample_ifoc_GetTaskStatsRequest_msg(void) {
    struct ifoc_GetTaskStatsRequest msg;

    return msg;
}
#endif
