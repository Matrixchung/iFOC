#pragma once

#include "stdint.h"

/*
 * Initialized by: Bootloader (set bootloader_presented = 1, version)
 * Read by: Bootloader & Application (app reads bootloader_presented & version)
 * Write by: Application (set update_requested, controllable_hardfault, ...)
 *
 */

typedef struct bkp_struct_t
{
    union
    {
        uint8_t reg;
        struct
        {
            uint8_t bootloader_presented   : 1; /* [0] */
            uint8_t update_requested       : 1; /* [1] */
            uint8_t controllable_hardfault : 1; /* [2] */
            uint8_t app_init_success       : 1; /* [3] */
            uint8_t enable_can_resistor    : 1; /* [4] */
            uint8_t                        : 3; /* [5:7] */ // reserved
        } bit;
    } flags;
    struct
    {
        uint8_t major;
        uint8_t minor;
        uint8_t vcs_commit[4];
    } version;
    uint8_t controllable_hardfault_count;
    uint8_t file_server_node_id;
    uint8_t dronecan_image_path[7];
    uint8_t reserved;
    uint8_t crc8;
} __attribute__((packed)) bkp_struct_t;