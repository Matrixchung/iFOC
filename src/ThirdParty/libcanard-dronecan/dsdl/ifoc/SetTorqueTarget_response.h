#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canard_dronecan.h"


#define IFOC_SETTORQUETARGET_RESPONSE_MAX_SIZE 0
#define IFOC_SETTORQUETARGET_RESPONSE_SIGNATURE (0x647A77E42ED5127AULL)
#define IFOC_SETTORQUETARGET_RESPONSE_ID 113

struct ifoc_SetTorqueTargetResponse {
};

uint32_t ifoc_SetTorqueTargetResponse_encode(struct ifoc_SetTorqueTargetResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool ifoc_SetTorqueTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, struct ifoc_SetTorqueTargetResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _ifoc_SetTorqueTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetResponse* msg, bool tao);
static inline bool _ifoc_SetTorqueTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetResponse* msg, bool tao);
void _ifoc_SetTorqueTargetResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode ifoc_SetTorqueTargetResponse, return true on failure, false on success
*/
bool _ifoc_SetTorqueTargetResponse_decode(const DroneCAN::CanardRxTransfer* transfer, uint32_t* bit_ofs, struct ifoc_SetTorqueTargetResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct ifoc_SetTorqueTargetResponse sample_ifoc_SetTorqueTargetResponse_msg(void);
#endif
