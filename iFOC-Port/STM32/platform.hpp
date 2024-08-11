#ifndef _PLATFORM_HPP
#define _PLATFORM_HPP

#include "global_include.h"

uint64_t GetSerialNumber()
{
    // This procedure of building a USB serial number should be identical
    // to the way the STM's built-in USB bootloader does it. This means
    // that the device will have the same serial number in normal and DFU mode.
    uint32_t uuid0 = HAL_GetUIDw0();
    uint32_t uuid1 = HAL_GetUIDw1();
    uint32_t uuid2 = HAL_GetUIDw2();
    uint32_t uuid_mixed_part = uuid0 + uuid2;
    uint64_t raw = ((uint64_t) uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);
    uint32_t x = (uint32_t)raw;
    uint32_t y = (uint32_t)(raw >> 32);
    x = HAL_CRC_Calculate(&hcrc, &x, 1);
    y = HAL_CRC_Calculate(&hcrc, &y, 1);
    return ((uint64_t)y << 16 | (uint64_t)x); // only first 6-bytes (u16 + u32)
}

#endif