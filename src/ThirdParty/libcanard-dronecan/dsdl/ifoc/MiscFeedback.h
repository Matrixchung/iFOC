#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_MISCFEEDBACK_MAX_SIZE 7
#define IFOC_MISCFEEDBACK_SIGNATURE (0xC1051A58150F6C1FULL)
#define IFOC_MISCFEEDBACK_ID 20601

#define IFOC_MISCFEEDBACK_VOLT_PER_LSB 0.2
#define IFOC_MISCFEEDBACK_AMPERE_PER_LSB 0.2
#define IFOC_MISCFEEDBACK_MIN_BROADCASTING_PERIOD_MS 10
#define IFOC_MISCFEEDBACK_MAX_BROADCASTING_PERIOD_MS 500

struct ifoc_MiscFeedback {
    uint16_t dc_bus_voltage;
    int16_t dc_bus_current;
    int16_t core_temp_celsius;
    int16_t mosfet_temp_celsius;
    int16_t motor_temp_celsius;
    uint8_t rt_task_time_us;
};

uint32_t ifoc_MiscFeedback_encode(struct ifoc_MiscFeedback* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_MiscFeedback_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_MiscFeedback* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_MiscFeedback_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_MiscFeedback* msg, bool tao);
static inline bool _ifoc_MiscFeedback_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_MiscFeedback* msg, bool tao);
void _ifoc_MiscFeedback_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_MiscFeedback* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 12, &msg->dc_bus_voltage);
    *bit_ofs += 12;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 12, &msg->dc_bus_current);
    *bit_ofs += 12;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 9, &msg->core_temp_celsius);
    *bit_ofs += 9;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 9, &msg->mosfet_temp_celsius);
    *bit_ofs += 9;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 9, &msg->motor_temp_celsius);
    *bit_ofs += 9;
    DroneCAN::canardEncodeScalar(buffer, *bit_ofs, 5, &msg->rt_task_time_us);
    *bit_ofs += 5;
}

/*
 decode ifoc_MiscFeedback, return true on failure, false on success
*/
bool _ifoc_MiscFeedback_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_MiscFeedback* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 12, false, &msg->dc_bus_voltage);
    *bit_ofs += 12;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 12, true, &msg->dc_bus_current);
    *bit_ofs += 12;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 9, true, &msg->core_temp_celsius);
    *bit_ofs += 9;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 9, true, &msg->mosfet_temp_celsius);
    *bit_ofs += 9;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 9, true, &msg->motor_temp_celsius);
    *bit_ofs += 9;

    DroneCAN::canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->rt_task_time_us);
    *bit_ofs += 5;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_MiscFeedback sample_ifoc_MiscFeedback_msg(void);
#endif
