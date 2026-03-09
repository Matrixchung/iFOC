#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_RTOSTASKSTATUS_MAX_SIZE 21
#define IFOC_RTOSTASKSTATUS_SIGNATURE (0xFCE361D1843151C0ULL)

#define IFOC_RTOSTASKSTATUS_TASK_STATE_RUNNING 0
#define IFOC_RTOSTASKSTATUS_TASK_STATE_READY 1
#define IFOC_RTOSTASKSTATUS_TASK_STATE_BLOCKED 2
#define IFOC_RTOSTASKSTATUS_TASK_STATE_SUSPENDED 3
#define IFOC_RTOSTASKSTATUS_TASK_STATE_DELETED 4
#define IFOC_RTOSTASKSTATUS_TASK_STATE_INVALID 5

struct ifoc_RTOSTaskStatus {
    uint8_t task_state;
    uint8_t priority;
    uint8_t run_time_pct;
    uint16_t min_stack_remaining;
    struct { uint8_t len; uint8_t data[16]; }task_name;
};

uint32_t ifoc_RTOSTaskStatus_encode(struct ifoc_RTOSTaskStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_RTOSTaskStatus_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_RTOSTaskStatus* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_RTOSTaskStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_RTOSTaskStatus* msg, bool tao);
static inline bool _ifoc_RTOSTaskStatus_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_RTOSTaskStatus* msg, bool tao);
void _ifoc_RTOSTaskStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_RTOSTaskStatus* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 3, &msg->task_state);
    *bit_ofs += 3;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &msg->priority);
    *bit_ofs += 5;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->run_time_pct);
    *bit_ofs += 7;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &msg->min_stack_remaining);
    *bit_ofs += 16;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t task_name_len = msg->task_name.len > 16 ? 16 : msg->task_name.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &task_name_len);
        *bit_ofs += 5;
    }
    for (size_t i=0; i < task_name_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->task_name.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode ifoc_RTOSTaskStatus, return true on failure, false on success
*/
bool _ifoc_RTOSTaskStatus_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_RTOSTaskStatus* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->task_state);
    *bit_ofs += 3;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->priority);
    *bit_ofs += 5;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->run_time_pct);
    *bit_ofs += 7;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->min_stack_remaining);
    *bit_ofs += 16;

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->task_name.len);
        *bit_ofs += 5;
    } else {
        msg->task_name.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->task_name.len > 16) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->task_name.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->task_name.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_RTOSTaskStatus sample_ifoc_RTOSTaskStatus_msg(void);
#endif
