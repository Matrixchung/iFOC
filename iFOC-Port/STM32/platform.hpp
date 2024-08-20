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

// https://community.st.com/t5/stm32-mcus/how-to-jump-to-system-bootloader-from-application-code-on-stm32/ta-p/49424
#define C0     1
#define F030x8 2
#define F030xC 3
#define F03xx  4
#define F05    5
#define F07    6
#define F09    7
#define F10xx  8
#define F105   9
#define F107   10
#define F10XL  11
#define F2     12
#define F3     13
#define F4     14
#define F7     15
#define G0     16
#define G4     17
#define H503   18
#define H563   19
#define H573   20
#define H7x    21
#define H7A    22
#define H7B    23
#define L0     24
#define L1     25
#define L5     26
#define WBA    27
#define WBX    28
#define WL     29
#define U5     30

#ifdef MCU
#if MCU == C0 || MCU == F2 || MCU == F4 || MCU == G0 || MCU == G4 || MCU == L4 || MCU == WBX || MCU == WL
#define BL_ADDR 0x1FFF0000
#elif MCU == F030x8 || MCU == F03xx || MCU == F05
#define BL_ADDR 0x1FFFEC00
#elif MCU == F030xC || MCU == F09 || MCU == F3
#define BL_ADDR 0x1FFFD800
#elif MCU == F07
#define BL_ADDR 0x1FFFC800
#elif MCU == F10xx
#define BL_ADDR 0x1FFFF000
#elif MCU == F105 || MCU == F107
#define BL_ADDR 0x1FFFB000
#elif MCU == F10XL
#define BL_ADDR 0x1FFFE000
#elif MCU == F7 || MCU == L0 || MCU == L1
#define BL_ADDR 0x1FF00000
#elif MCU == H503
#define BL_ADDR 0x0BF87000
#elif MCU == H563 || MCU == H573
#define BL_ADDR 0x0BF97000
#elif MCU == H7x
#define BL_ADDR 0x1FF09800
#elif MCU == H7A
#define BL_ADDR 0x1FF0A800
#elif MCU == H7B
#define BL_ADDR 0x1FF0A000
#elif MCU == L5 || MCU == U5
#define BL_ADDR 0x0BF90000
#elif MCU == WBA
#define BL_ADDR 0x0BF88000
#else
#error "ST MCU defined but not supported!"
#endif
#else
#error "ST MCU type not defined!"
#endif

void JumpToBootloader()
{
    uint8_t i = 0;
    void (*SysMemBootJump)(void);
    volatile uint32_t bootloader_addr = BL_ADDR;
    __disable_irq();
    SysTick->CTRL = 0;
    HAL_RCC_DeInit();
    for(i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    __enable_irq();
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(bootloader_addr + 4)));
    __set_MSP(*(uint32_t *)bootloader_addr);
    SysMemBootJump(); // call bl jump
    while(1)
    {
        // we should never reach here
    }
}

#endif