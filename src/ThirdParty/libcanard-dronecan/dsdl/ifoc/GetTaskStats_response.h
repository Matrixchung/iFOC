#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "TaskStatus.h"


#define IFOC_GETTASKSTATS_RESPONSE_MAX_SIZE 405
#define IFOC_GETTASKSTATS_RESPONSE_SIGNATURE (0x330C5183220931B4ULL)
#define IFOC_GETTASKSTATS_RESPONSE_ID 118

struct ifoc_GetTaskStatsResponse {
    uint8_t rt_task_time_us;
    uint8_t rt_to_rem_wait_time_us;
    uint8_t rem_task_time_us;
    uint8_t mid_task_time_us;
    struct { uint8_t len; struct ifoc_TaskStatus data[16]; }rt_task_list;
    struct { uint8_t len; struct ifoc_TaskStatus data[16]; }mid_task_list;
};

uint32_t ifoc_GetTaskStatsResponse_encode(struct ifoc_GetTaskStatsResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetTaskStatsResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetTaskStatsResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetTaskStatsResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsResponse* msg, bool tao);
static inline bool _ifoc_GetTaskStatsResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsResponse* msg, bool tao);
void _ifoc_GetTaskStatsResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 6, &msg->rt_task_time_us);
    *bit_ofs += 6;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 6, &msg->rt_to_rem_wait_time_us);
    *bit_ofs += 6;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &msg->rem_task_time_us);
    *bit_ofs += 5;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->mid_task_time_us);
    *bit_ofs += 7;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t rt_task_list_len = msg->rt_task_list.len > 16 ? 16 : msg->rt_task_list.len;
#pragma GCC diagnostic pop
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &rt_task_list_len);
    *bit_ofs += 5;
    for (size_t i=0; i < rt_task_list_len; i++) {
        _ifoc_TaskStatus_encode(buffer, bit_ofs, &msg->rt_task_list.data[i], false);
    }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t mid_task_list_len = msg->mid_task_list.len > 16 ? 16 : msg->mid_task_list.len;
#pragma GCC diagnostic pop
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &mid_task_list_len);
    *bit_ofs += 5;
    for (size_t i=0; i < mid_task_list_len; i++) {
        _ifoc_TaskStatus_encode(buffer, bit_ofs, &msg->mid_task_list.data[i], tao && i==msg->mid_task_list.len);
    }
}

/*
 decode ifoc_GetTaskStatsResponse, return true on failure, false on success
*/
bool _ifoc_GetTaskStatsResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->rt_task_time_us);
    *bit_ofs += 6;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->rt_to_rem_wait_time_us);
    *bit_ofs += 6;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->rem_task_time_us);
    *bit_ofs += 5;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->mid_task_time_us);
    *bit_ofs += 7;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->rt_task_list.len);
    *bit_ofs += 5;
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->rt_task_list.len > 16) {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i=0; i < msg->rt_task_list.len; i++) {
            if (_ifoc_TaskStatus_decode(transfer, bit_ofs, &msg->rt_task_list.data[i], false)) {return true;}
        }
    }

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->mid_task_list.len);
    *bit_ofs += 5;
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->mid_task_list.len > 16) {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i=0; i < msg->mid_task_list.len; i++) {
            if (_ifoc_TaskStatus_decode(transfer, bit_ofs, &msg->mid_task_list.data[i], tao && i==msg->mid_task_list.len)) {return true;}
        }
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetTaskStatsResponse sample_ifoc_GetTaskStatsResponse_msg(void);
#endif
