#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE 14
#define UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE (0xA9AF28AEA2FBB254ULL)
#define UAVCAN_EQUIPMENT_ESC_STATUS_ID 1034

struct uavcan_equipment_esc_Status {
    uint32_t error_count;
    float voltage;
    float current;
    float temperature;
    int32_t rpm;
    uint8_t power_rating_pct;
    uint8_t esc_index;
};

uint32_t uavcan_equipment_esc_Status_encode(struct uavcan_equipment_esc_Status* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_esc_Status_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_equipment_esc_Status* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_esc_Status_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_Status* msg, bool tao);
static inline bool _uavcan_equipment_esc_Status_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_Status* msg, bool tao);
void _uavcan_equipment_esc_Status_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_Status* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 32, &msg->error_count);
    *bit_ofs += 32;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->voltage);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->current);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->temperature);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 18, &msg->rpm);
    *bit_ofs += 18;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->power_rating_pct);
    *bit_ofs += 7;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &msg->esc_index);
    *bit_ofs += 5;
}

/*
 decode uavcan_equipment_esc_Status, return true on failure, false on success
*/
bool _uavcan_equipment_esc_Status_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_Status* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->error_count);
    *bit_ofs += 32;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->voltage = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->current = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->temperature = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 18, true, &msg->rpm);
    *bit_ofs += 18;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->power_rating_pct);
    *bit_ofs += 7;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->esc_index);
    *bit_ofs += 5;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_esc_Status sample_uavcan_equipment_esc_Status_msg(void);
#endif
