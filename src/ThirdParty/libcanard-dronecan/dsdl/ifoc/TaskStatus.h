#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_TASKSTATUS_MAX_SIZE 13
#define IFOC_TASKSTATUS_SIGNATURE (0x8A4B56FEC305006EULL)

struct ifoc_TaskStatus {
    struct { uint8_t len; uint8_t data[12]; }task_name;
};

uint32_t ifoc_TaskStatus_encode(struct ifoc_TaskStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_TaskStatus_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_TaskStatus* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_TaskStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_TaskStatus* msg, bool tao);
static inline bool _ifoc_TaskStatus_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_TaskStatus* msg, bool tao);
void _ifoc_TaskStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_TaskStatus* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t task_name_len = msg->task_name.len > 12 ? 12 : msg->task_name.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 4, &task_name_len);
        *bit_ofs += 4;
    }
    for (size_t i=0; i < task_name_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->task_name.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode ifoc_TaskStatus, return true on failure, false on success
*/
bool _ifoc_TaskStatus_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_TaskStatus* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->task_name.len);
        *bit_ofs += 4;
    } else {
        msg->task_name.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->task_name.len > 12) {
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
struct ifoc_TaskStatus sample_ifoc_TaskStatus_msg(void);
#endif
