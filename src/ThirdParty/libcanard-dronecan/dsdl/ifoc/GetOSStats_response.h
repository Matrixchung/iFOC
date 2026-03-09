#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "RTOSTaskStatus.h"
#include "SoftwareVersion.h"


#define IFOC_GETOSSTATS_RESPONSE_MAX_SIZE 371
#define IFOC_GETOSSTATS_RESPONSE_SIGNATURE (0x33F3DDF45EFD3F68ULL)
#define IFOC_GETOSSTATS_RESPONSE_ID 119

struct ifoc_GetOSStatsResponse {
    uint32_t mem_used;
    uint32_t mem_total;
    uint32_t nvm_used;
    uint32_t nvm_total;
    struct uavcan_protocol_SoftwareVersion app_version;
    struct uavcan_protocol_SoftwareVersion bootloader_version;
    struct { uint8_t len; struct ifoc_RTOSTaskStatus data[16]; }tasks;
};

uint32_t ifoc_GetOSStatsResponse_encode(struct ifoc_GetOSStatsResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_GetOSStatsResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_GetOSStatsResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_GetOSStatsResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetOSStatsResponse* msg, bool tao);
static inline bool _ifoc_GetOSStatsResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetOSStatsResponse* msg, bool tao);
void _ifoc_GetOSStatsResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_GetOSStatsResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 24, &msg->mem_used);
    *bit_ofs += 24;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 24, &msg->mem_total);
    *bit_ofs += 24;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 24, &msg->nvm_used);
    *bit_ofs += 24;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 24, &msg->nvm_total);
    *bit_ofs += 24;
    _uavcan_protocol_SoftwareVersion_encode(buffer, bit_ofs, &msg->app_version, false);
    _uavcan_protocol_SoftwareVersion_encode(buffer, bit_ofs, &msg->bootloader_version, false);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t tasks_len = msg->tasks.len > 16 ? 16 : msg->tasks.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &tasks_len);
        *bit_ofs += 5;
    }
    for (size_t i=0; i < tasks_len; i++) {
        _ifoc_RTOSTaskStatus_encode(buffer, bit_ofs, &msg->tasks.data[i], false);
    }
}

/*
 decode ifoc_GetOSStatsResponse, return true on failure, false on success
*/
bool _ifoc_GetOSStatsResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_GetOSStatsResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 24, false, &msg->mem_used);
    *bit_ofs += 24;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 24, false, &msg->mem_total);
    *bit_ofs += 24;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 24, false, &msg->nvm_used);
    *bit_ofs += 24;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 24, false, &msg->nvm_total);
    *bit_ofs += 24;

    if (_uavcan_protocol_SoftwareVersion_decode(transfer, bit_ofs, &msg->app_version, false)) {return true;}

    if (_uavcan_protocol_SoftwareVersion_decode(transfer, bit_ofs, &msg->bootloader_version, false)) {return true;}

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->tasks.len);
        *bit_ofs += 5;
    }


    if (tao) {
        msg->tasks.len = 0;
        size_t max_len = 16;
        uint32_t max_bits = (transfer->payload_len*8)-7; // TAO elements must be >= 8 bits
        while (max_bits > *bit_ofs) {
            if (!max_len-- || _ifoc_RTOSTaskStatus_decode(transfer, bit_ofs, &msg->tasks.data[msg->tasks.len], false)) {return true;}
            msg->tasks.len++;
        }
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->tasks.len > 16) {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i=0; i < msg->tasks.len; i++) {
            if (_ifoc_RTOSTaskStatus_decode(transfer, bit_ofs, &msg->tasks.data[i], false)) {return true;}
        }
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_GetOSStatsResponse sample_ifoc_GetOSStatsResponse_msg(void);
#endif
