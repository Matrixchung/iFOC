#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "SingleLightCommand.h"


#define UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_MAX_SIZE 61
#define UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_SIGNATURE (0x2031D93C8BDD1EC4ULL)
#define UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID 1081

struct uavcan_equipment_indication_LightsCommand {
    struct { uint8_t len; struct uavcan_equipment_indication_SingleLightCommand data[20]; }commands;
};

uint32_t uavcan_equipment_indication_LightsCommand_encode(struct uavcan_equipment_indication_LightsCommand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_indication_LightsCommand_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_equipment_indication_LightsCommand* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_indication_LightsCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_indication_LightsCommand* msg, bool tao);
static inline bool _uavcan_equipment_indication_LightsCommand_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_indication_LightsCommand* msg, bool tao);
void _uavcan_equipment_indication_LightsCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_indication_LightsCommand* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t commands_len = msg->commands.len > 20 ? 20 : msg->commands.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &commands_len);
        *bit_ofs += 5;
    }
    for (size_t i=0; i < commands_len; i++) {
        _uavcan_equipment_indication_SingleLightCommand_encode(buffer, bit_ofs, &msg->commands.data[i], false);
    }
}

/*
 decode uavcan_equipment_indication_LightsCommand, return true on failure, false on success
*/
bool _uavcan_equipment_indication_LightsCommand_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_indication_LightsCommand* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->commands.len);
        *bit_ofs += 5;
    }


    if (tao) {
        msg->commands.len = 0;
        size_t max_len = 20;
        uint32_t max_bits = (transfer->payload_len*8)-7; // TAO elements must be >= 8 bits
        while (max_bits > *bit_ofs) {
            if (!max_len-- || _uavcan_equipment_indication_SingleLightCommand_decode(transfer, bit_ofs, &msg->commands.data[msg->commands.len], false)) {return true;}
            msg->commands.len++;
        }
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (msg->commands.len > 20) {
            return true; /* invalid value */
        }
#pragma GCC diagnostic pop
        for (size_t i=0; i < msg->commands.len; i++) {
            if (_uavcan_equipment_indication_SingleLightCommand_decode(transfer, bit_ofs, &msg->commands.data[i], false)) {return true;}
        }
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_indication_LightsCommand sample_uavcan_equipment_indication_LightsCommand_msg(void);
#endif
