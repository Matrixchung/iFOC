#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETMOTORSTATE_REQUEST_MAX_SIZE 1
#define IFOC_SETMOTORSTATE_REQUEST_SIGNATURE (0x14928CAED045AC23ULL)
#define IFOC_SETMOTORSTATE_REQUEST_ID 107

#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_IDLE 0
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_STARTUP_SEQUENCE 1
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_BASIC_PARAM_CALIBRATION 2
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_ENCODER_INDEX_SEARCH 3
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_ENCODER_CALIBRATION 4
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_EXTENDED_PARAM_CALIBRATION 5
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_SENSORED_CLOSED_LOOP_CONTROL 6
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_SENSORLESS_CLOSED_LOOP_CONTROL 7
#define IFOC_SETMOTORSTATE_REQUEST_SET_STATE_OPEN_LOOP_VELOCITY_CONTROL 8

struct ifoc_SetMotorStateRequest {
    uint8_t set_state;
};

uint32_t ifoc_SetMotorStateRequest_encode(struct ifoc_SetMotorStateRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetMotorStateRequest_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetMotorStateRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetMotorStateRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetMotorStateRequest* msg, bool tao);
static inline bool _ifoc_SetMotorStateRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetMotorStateRequest* msg, bool tao);
void _ifoc_SetMotorStateRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetMotorStateRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 4, &msg->set_state);
    *bit_ofs += 4;
}

/*
 decode ifoc_SetMotorStateRequest, return true on failure, false on success
*/
bool _ifoc_SetMotorStateRequest_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetMotorStateRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->set_state);
    *bit_ofs += 4;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetMotorStateRequest sample_ifoc_SetMotorStateRequest_msg(void);
#endif
