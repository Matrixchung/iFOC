#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_EQUIPMENT_POWER_PRIMARYPOWERSUPPLYSTATUS_MAX_SIZE 6
#define UAVCAN_EQUIPMENT_POWER_PRIMARYPOWERSUPPLYSTATUS_SIGNATURE (0xBBA05074AD757480ULL)
#define UAVCAN_EQUIPMENT_POWER_PRIMARYPOWERSUPPLYSTATUS_ID 1090

struct uavcan_equipment_power_PrimaryPowerSupplyStatus {
    float hours_to_empty_at_10sec_avg_power;
    float hours_to_empty_at_10sec_avg_power_variance;
    bool external_power_available;
    uint8_t remaining_energy_pct;
    uint8_t remaining_energy_pct_stdev;
};

uint32_t uavcan_equipment_power_PrimaryPowerSupplyStatus_encode(struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_power_PrimaryPowerSupplyStatus_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_power_PrimaryPowerSupplyStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg, bool tao);
static inline bool _uavcan_equipment_power_PrimaryPowerSupplyStatus_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg, bool tao);
void _uavcan_equipment_power_PrimaryPowerSupplyStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->hours_to_empty_at_10sec_avg_power);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->hours_to_empty_at_10sec_avg_power_variance);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->external_power_available);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->remaining_energy_pct);
    *bit_ofs += 7;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &msg->remaining_energy_pct_stdev);
    *bit_ofs += 7;
}

/*
 decode uavcan_equipment_power_PrimaryPowerSupplyStatus, return true on failure, false on success
*/
bool _uavcan_equipment_power_PrimaryPowerSupplyStatus_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->hours_to_empty_at_10sec_avg_power = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->hours_to_empty_at_10sec_avg_power_variance = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->external_power_available);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->remaining_energy_pct);
    *bit_ofs += 7;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->remaining_energy_pct_stdev);
    *bit_ofs += 7;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_power_PrimaryPowerSupplyStatus sample_uavcan_equipment_power_PrimaryPowerSupplyStatus_msg(void);
#endif
