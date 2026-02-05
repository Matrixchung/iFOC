#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_GETOSSTATS_REQUEST_MAX_SIZE 0
#define IFOC_GETOSSTATS_REQUEST_SIGNATURE (0x7EBACAD13E0DFA28ULL)
#define IFOC_GETOSSTATS_REQUEST_ID 209

struct ifoc_GetOSStatsRequest {
};

uint32_t ifoc_GetOSStatsRequest_encode(struct ifoc_GetOSStatsRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetOSStatsRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetOSStatsRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetOSStatsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetOSStatsRequest* msg, bool tao);
static inline bool _ifoc_GetOSStatsRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetOSStatsRequest* msg, bool tao);
void _ifoc_GetOSStatsRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetOSStatsRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_GetOSStatsRequest, return true on failure, false on success
*/
bool _ifoc_GetOSStatsRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetOSStatsRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetOSStatsRequest sample_ifoc_GetOSStatsRequest_msg(void);
#endif
