#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_REFERENCEFRAME_MAX_SIZE 1
#define IFOC_REFERENCEFRAME_SIGNATURE (0x9A9E6057ED9B423CULL)

#define IFOC_REFERENCEFRAME_REFERENCE_ELEC 0
#define IFOC_REFERENCEFRAME_REFERENCE_BASE 1
#define IFOC_REFERENCEFRAME_REFERENCE_OUTPUT 2
#define IFOC_REFERENCEFRAME_TORQUE_UNIT_AMP 0
#define IFOC_REFERENCEFRAME_TORQUE_UNIT_NM 1
#define IFOC_REFERENCEFRAME_SPEED_UNIT_RADS 0
#define IFOC_REFERENCEFRAME_SPEED_UNIT_DEGS 1
#define IFOC_REFERENCEFRAME_SPEED_UNIT_REVS 2
#define IFOC_REFERENCEFRAME_SPEED_UNIT_RPM 3
#define IFOC_REFERENCEFRAME_SPEED_UNIT_HZ 4
#define IFOC_REFERENCEFRAME_POS_UNIT_RAD 0
#define IFOC_REFERENCEFRAME_POS_UNIT_DEG 1
#define IFOC_REFERENCEFRAME_POS_UNIT_REV 2

struct ifoc_ReferenceFrame {
    uint8_t reference;
    bool torque_unit;
    uint8_t speed_unit;
    uint8_t pos_unit;
};

uint32_t ifoc_ReferenceFrame_encode(struct ifoc_ReferenceFrame* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_ReferenceFrame_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_ReferenceFrame* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_ReferenceFrame_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_ReferenceFrame* msg, bool tao);
static inline bool _ifoc_ReferenceFrame_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_ReferenceFrame* msg, bool tao);
void _ifoc_ReferenceFrame_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_ReferenceFrame* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 2, &msg->reference);
    *bit_ofs += 2;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->torque_unit);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 3, &msg->speed_unit);
    *bit_ofs += 3;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 2, &msg->pos_unit);
    *bit_ofs += 2;
}

/*
 decode ifoc_ReferenceFrame, return true on failure, false on success
*/
bool _ifoc_ReferenceFrame_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_ReferenceFrame* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 2, false, &msg->reference);
    *bit_ofs += 2;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->torque_unit);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->speed_unit);
    *bit_ofs += 3;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 2, false, &msg->pos_unit);
    *bit_ofs += 2;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_ReferenceFrame sample_ifoc_ReferenceFrame_msg(void);
#endif
