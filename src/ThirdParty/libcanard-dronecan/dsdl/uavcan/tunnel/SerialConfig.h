#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_TUNNEL_SERIALCONFIG_MAX_SIZE 8
#define UAVCAN_TUNNEL_SERIALCONFIG_SIGNATURE (0x4237AACEE87E82ADULL)
#define UAVCAN_TUNNEL_SERIALCONFIG_ID 2011

struct uavcan_tunnel_SerialConfig {
    uint8_t channel_id;
    uint32_t baud;
    uint32_t options;
};

uint32_t uavcan_tunnel_SerialConfig_encode(struct uavcan_tunnel_SerialConfig* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_tunnel_SerialConfig_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_tunnel_SerialConfig* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_tunnel_SerialConfig_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_tunnel_SerialConfig* msg, bool tao);
static inline bool _uavcan_tunnel_SerialConfig_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_tunnel_SerialConfig* msg, bool tao);
void _uavcan_tunnel_SerialConfig_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_tunnel_SerialConfig* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->channel_id);
    *bit_ofs += 8;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 32, &msg->baud);
    *bit_ofs += 32;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 24, &msg->options);
    *bit_ofs += 24;
}

/*
 decode uavcan_tunnel_SerialConfig, return true on failure, false on success
*/
bool _uavcan_tunnel_SerialConfig_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_tunnel_SerialConfig* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->channel_id);
    *bit_ofs += 8;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->baud);
    *bit_ofs += 32;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 24, false, &msg->options);
    *bit_ofs += 24;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_tunnel_SerialConfig sample_uavcan_tunnel_SerialConfig_msg(void);
#endif
