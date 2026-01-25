#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_MAX_SIZE 4
#define UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_SIGNATURE (0x8D9A6A920C1D616CULL)

#define UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS 0
#define UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION 1
#define UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_FORCE 2
#define UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_SPEED 3
#define UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM 4

struct uavcan_equipment_actuator_Command {
    uint8_t actuator_id;
    uint8_t command_type;
    float command_value;
};

uint32_t uavcan_equipment_actuator_Command_encode(struct uavcan_equipment_actuator_Command* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_actuator_Command_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_equipment_actuator_Command* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_actuator_Command_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_actuator_Command* msg, bool tao);
static inline bool _uavcan_equipment_actuator_Command_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_actuator_Command* msg, bool tao);
void _uavcan_equipment_actuator_Command_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_actuator_Command* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->actuator_id);
    *bit_ofs += 8;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->command_type);
    *bit_ofs += 8;
    {
        uint16_t float16_val = DroneCAN::canardConvertNativeFloatToFloat16(msg->command_value);
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

/*
 decode uavcan_equipment_actuator_Command, return true on failure, false on success
*/
bool _uavcan_equipment_actuator_Command_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_actuator_Command* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->actuator_id);
    *bit_ofs += 8;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->command_type);
    *bit_ofs += 8;

    {
        uint16_t float16_val;
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->command_value = DroneCAN::canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_actuator_Command sample_uavcan_equipment_actuator_Command_msg(void);
#endif
