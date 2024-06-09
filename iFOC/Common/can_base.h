#ifndef _FOC_CAN_BASE_H
#define _FOC_CAN_BASE_H

#include "stdint.h"

class CANBase
{
public:
    virtual void ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index) = 0;
    virtual void InitHW() = 0;
    virtual void SendPayload(uint32_t id, uint8_t *payload, uint8_t len) = 0;
    uint8_t rx_buffer[16] = {0x00};
};

#endif