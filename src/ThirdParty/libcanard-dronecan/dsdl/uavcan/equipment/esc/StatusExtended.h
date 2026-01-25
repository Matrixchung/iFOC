#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_MAX_SIZE 7
#define UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_SIGNATURE (0x2DC203C50960EDCULL)
#define UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_ID 1036

struct uavcan_equipment_esc_StatusExtended {
    uint8_t input_pct;
    uint8_t output_pct;
    int16_t motor_temperature_degC;
    uint16_t motor_angle;
    uint32_t status_flags;
    uint8_t esc_index;
};

uint32_t uavcan_equipment_esc_StatusExtended_encode(struct uavcan_equipment_esc_StatusExtended* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_esc_StatusExtended_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_equipment_esc_StatusExtended* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_esc_StatusExtended_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao);
static inline bool _uavcan_equipment_esc_StatusExtended_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao);
void _uavcan_equipment_esc_StatusExtended_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->input_pct);
    *bit_ofs += 7;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->output_pct);
    *bit_ofs += 7;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 9, &msg->motor_temperature_degC);
    *bit_ofs += 9;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 9, &msg->motor_angle);
    *bit_ofs += 9;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 19, &msg->status_flags);
    *bit_ofs += 19;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &msg->esc_index);
    *bit_ofs += 5;
}

/*
 decode uavcan_equipment_esc_StatusExtended, return true on failure, false on success
*/
bool _uavcan_equipment_esc_StatusExtended_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->input_pct);
    *bit_ofs += 7;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->output_pct);
    *bit_ofs += 7;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 9, true, &msg->motor_temperature_degC);
    *bit_ofs += 9;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 9, false, &msg->motor_angle);
    *bit_ofs += 9;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 19, false, &msg->status_flags);
    *bit_ofs += 19;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->esc_index);
    *bit_ofs += 5;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_esc_StatusExtended sample_uavcan_equipment_esc_StatusExtended_msg(void);
#endif
