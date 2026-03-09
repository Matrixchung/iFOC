#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETTASKSTATS_REQUEST_MAX_SIZE 0
#define IFOC_GETTASKSTATS_REQUEST_SIGNATURE (0x330C5183220931B4ULL)
#define IFOC_GETTASKSTATS_REQUEST_ID 118

struct ifoc_GetTaskStatsRequest {
};

uint32_t ifoc_GetTaskStatsRequest_encode(struct ifoc_GetTaskStatsRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetTaskStatsRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetTaskStatsRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetTaskStatsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsRequest* msg, bool tao);
static inline bool _ifoc_GetTaskStatsRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsRequest* msg, bool tao);
void _ifoc_GetTaskStatsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_GetTaskStatsRequest, return true on failure, false on success
*/
bool _ifoc_GetTaskStatsRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetTaskStatsRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetTaskStatsRequest sample_ifoc_GetTaskStatsRequest_msg(void);
#endif
