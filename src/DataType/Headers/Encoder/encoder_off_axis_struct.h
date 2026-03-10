#pragma once

#include "stdint.h"

typedef struct encoder_off_axis_struct_t
{
    union
    {
        uint8_t reg;
        struct
        {
            uint8_t hw_ready           : 1; /* [0] */
            uint8_t mag_field_weak     : 1; /* [1] */
            uint8_t mag_field_overflow : 1; /* [2] */
            uint8_t                    : 5; /* [3:7] */
        } bit;
    } flags;
    float channel_a_mV;
    float channel_b_mV;
    uint8_t crc8;
    uint8_t tail_0x70;
    uint8_t tail_0x5f;
} __attribute__((packed)) encoder_off_axis_struct_t;