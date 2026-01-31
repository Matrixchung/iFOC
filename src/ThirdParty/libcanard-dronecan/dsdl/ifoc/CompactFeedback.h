#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_COMPACTFEEDBACK_MAX_SIZE 7
#define IFOC_COMPACTFEEDBACK_SIGNATURE (0xEAC668D7CCD67FBDULL)
#define IFOC_COMPACTFEEDBACK_ID 20600

#define IFOC_COMPACTFEEDBACK_RPM_AMP_RANGE_RATIO 2.0
#define IFOC_COMPACTFEEDBACK_MIN_BROADCASTING_PERIOD_MS 1
#define IFOC_COMPACTFEEDBACK_MAX_BROADCASTING_PERIOD_MS 100
#define IFOC_COMPACTFEEDBACK_STATE_IDLE 0
#define IFOC_COMPACTFEEDBACK_STATE_STARTUP_SEQUENCE 1
#define IFOC_COMPACTFEEDBACK_STATE_BASIC_PARAM_CALIBRATION 2
#define IFOC_COMPACTFEEDBACK_STATE_ENCODER_INDEX_SEARCH 3
#define IFOC_COMPACTFEEDBACK_STATE_ENCODER_CALIBRATION 4
#define IFOC_COMPACTFEEDBACK_STATE_EXTENDED_PARAM_CALIBRATION 5
#define IFOC_COMPACTFEEDBACK_STATE_SENSORED_CLOSED_LOOP_CONTROL 6
#define IFOC_COMPACTFEEDBACK_STATE_SENSORLESS_CLOSED_LOOP_CONTROL 7
#define IFOC_COMPACTFEEDBACK_STATE_OPEN_LOOP_VELOCITY_CONTROL 8
#define IFOC_COMPACTFEEDBACK_CONTROL_MODE_POSITION 0
#define IFOC_COMPACTFEEDBACK_CONTROL_MODE_VELOCITY 1
#define IFOC_COMPACTFEEDBACK_CONTROL_MODE_CURRENT 2
#define IFOC_COMPACTFEEDBACK_CONTROL_MODE_HYBRID 3

struct ifoc_CompactFeedback {
    uint8_t state;
    uint8_t control_mode;
    bool has_error;
    bool is_armed;
    uint16_t output_single_round;
    int16_t output_velocity_rpm;
    int16_t phase_q_current_amp;
};

uint32_t ifoc_CompactFeedback_encode(struct ifoc_CompactFeedback* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_CompactFeedback_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_CompactFeedback* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_CompactFeedback_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_CompactFeedback* msg, bool tao);
static inline bool _ifoc_CompactFeedback_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_CompactFeedback* msg, bool tao);
void _ifoc_CompactFeedback_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_CompactFeedback* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 4, &msg->state);
    *bit_ofs += 4;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 2, &msg->control_mode);
    *bit_ofs += 2;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->has_error);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 1, &msg->is_armed);
    *bit_ofs += 1;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &msg->output_single_round);
    *bit_ofs += 16;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &msg->output_velocity_rpm);
    *bit_ofs += 16;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 16, &msg->phase_q_current_amp);
    *bit_ofs += 16;
}

/*
 decode ifoc_CompactFeedback, return true on failure, false on success
*/
bool _ifoc_CompactFeedback_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_CompactFeedback* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->state);
    *bit_ofs += 4;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 2, false, &msg->control_mode);
    *bit_ofs += 2;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->has_error);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->is_armed);
    *bit_ofs += 1;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->output_single_round);
    *bit_ofs += 16;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->output_velocity_rpm);
    *bit_ofs += 16;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 16, true, &msg->phase_q_current_amp);
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_CompactFeedback sample_ifoc_CompactFeedback_msg(void);
#endif
