#define CANARD_DSDLC_INTERNAL
#include "RTOSTaskStatus.h"
#include <string.h>

using namespace DroneCAN;

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t ifoc_RTOSTaskStatus_encode(struct ifoc_RTOSTaskStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, IFOC_RTOSTASKSTATUS_MAX_SIZE);
    _ifoc_RTOSTaskStatus_encode(buffer, &bit_ofs, msg, 
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
bool ifoc_RTOSTaskStatus_decode(const CanardRxTransfer* transfer, struct ifoc_RTOSTaskStatus* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > IFOC_RTOSTASKSTATUS_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_ifoc_RTOSTaskStatus_decode(transfer, &bit_ofs, msg,
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
struct ifoc_RTOSTaskStatus sample_ifoc_RTOSTaskStatus_msg(void) {
    struct ifoc_RTOSTaskStatus msg;

    msg.task_state = (uint8_t)random_bitlen_unsigned_val(3);
    msg.priority = (uint8_t)random_bitlen_unsigned_val(5);
    msg.min_stack_remaining = (uint16_t)random_bitlen_unsigned_val(16);
    msg.task_name.len = (uint8_t)random_range_unsigned_val(0, 16);
    for (size_t i=0; i < msg.task_name.len; i++) {
        msg.task_name.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    return msg;
}
#endif
