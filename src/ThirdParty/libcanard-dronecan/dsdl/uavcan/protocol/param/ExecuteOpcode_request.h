#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_MAX_SIZE 7
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_SIGNATURE (0x3B131AC5EB69D2CDULL)
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_ID 10

#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE 0
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE 1

struct uavcan_protocol_param_ExecuteOpcodeRequest {
    uint8_t opcode;
    int64_t argument;
};

uint32_t uavcan_protocol_param_ExecuteOpcodeRequest_encode(struct uavcan_protocol_param_ExecuteOpcodeRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_param_ExecuteOpcodeRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_param_ExecuteOpcodeRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_param_ExecuteOpcodeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeRequest* msg, bool tao);
static inline bool _uavcan_protocol_param_ExecuteOpcodeRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeRequest* msg, bool tao);
void _uavcan_protocol_param_ExecuteOpcodeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->opcode);
    *bit_ofs += 8;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 48, &msg->argument);
    *bit_ofs += 48;
}

/*
 decode uavcan_protocol_param_ExecuteOpcodeRequest, return true on failure, false on success
*/
bool _uavcan_protocol_param_ExecuteOpcodeRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->opcode);
    *bit_ofs += 8;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 48, true, &msg->argument);
    *bit_ofs += 48;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_param_ExecuteOpcodeRequest sample_uavcan_protocol_param_ExecuteOpcodeRequest_msg(void);
#endif
