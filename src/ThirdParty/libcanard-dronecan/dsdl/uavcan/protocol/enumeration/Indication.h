#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"
#include "NumericValue.h"


#define UAVCAN_PROTOCOL_ENUMERATION_INDICATION_MAX_SIZE 102
#define UAVCAN_PROTOCOL_ENUMERATION_INDICATION_SIGNATURE (0x884CB63050A84F35ULL)
#define UAVCAN_PROTOCOL_ENUMERATION_INDICATION_ID 380

struct uavcan_protocol_enumeration_Indication {
    struct uavcan_protocol_param_NumericValue value;
    struct { uint8_t len; uint8_t data[92]; }parameter_name;
};

uint32_t uavcan_protocol_enumeration_Indication_encode(struct uavcan_protocol_enumeration_Indication* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_enumeration_Indication_decode(const DroneCAN::CanardRxTransfer* transfer, struct uavcan_protocol_enumeration_Indication* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_enumeration_Indication_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_Indication* msg, bool tao);
static inline bool _uavcan_protocol_enumeration_Indication_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_Indication* msg, bool tao);
void _uavcan_protocol_enumeration_Indication_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_Indication* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    *bit_ofs += 6;
    _uavcan_protocol_param_NumericValue_encode(buffer, bit_ofs, &msg->value, false);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    const uint8_t parameter_name_len = msg->parameter_name.len > 92 ? 92 : msg->parameter_name.len;
#pragma GCC diagnostic pop
    if (!tao) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 7, &parameter_name_len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < parameter_name_len; i++) {
        DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 8, &msg->parameter_name.data[i]);
        *bit_ofs += 8;
    }
}

/*
 decode uavcan_protocol_enumeration_Indication, return true on failure, false on success
*/
bool _uavcan_protocol_enumeration_Indication_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_Indication* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    *bit_ofs += 6;

    if (_uavcan_protocol_param_NumericValue_decode(transfer, bit_ofs, &msg->value, false)) {return true;}

    if (!tao) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->parameter_name.len);
        *bit_ofs += 7;
    } else {
        msg->parameter_name.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    if (msg->parameter_name.len > 92) {
        return true; /* invalid value */
    }
#pragma GCC diagnostic pop
    for (size_t i=0; i < msg->parameter_name.len; i++) {
        DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->parameter_name.data[i]);
        *bit_ofs += 8;
    }

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_enumeration_Indication sample_uavcan_protocol_enumeration_Indication_msg(void);
#endif
