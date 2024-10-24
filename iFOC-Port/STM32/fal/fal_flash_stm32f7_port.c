/*
 * Copyright (c) 2020, Armink, <armink.ztl@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <fal.h>

#include <stm32f7xx.h>

// STM32F72xxx & STM32F732xx / F733xx, with a total of 512 Kbytes
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes, start with user code area */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes */

/**
 * Get the sector of a given address
 *
 * @param Address flash address
 *
 * @return The sector of a given address
 */
static uint32_t stm32_get_sector(uint32_t Address)
{
    uint32_t sector = 0;

    if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
    {
        sector = FLASH_SECTOR_7;
    }
    return sector;
}

/**
 * Get the sector size
 *
 * @param sector sector
 *
 * @return sector size
 */
static uint32_t stm32_get_sector_size(uint32_t sector) {
    assert(IS_FLASH_SECTOR(sector));

    switch (sector) {
    case FLASH_SECTOR_0: return 16 * 1024;
    case FLASH_SECTOR_1: return 16 * 1024;
    case FLASH_SECTOR_2: return 16 * 1024;
    case FLASH_SECTOR_3: return 16 * 1024;
    case FLASH_SECTOR_4: return 64 * 1024;
    case FLASH_SECTOR_5: return 128 * 1024;
    case FLASH_SECTOR_6: return 128 * 1024;
    case FLASH_SECTOR_7: return 128 * 1024;
    default : return 128 * 1024;
    }
}

static int read(long offset, uint8_t *buf, size_t size)
{
    size_t i;
    uint32_t addr = stm32f7_onchip_flash.addr + offset;
    for (i = 0; i < size; i++, addr++, buf++)
    {
        *buf = *(__IO uint8_t *) addr;
    }
    return size;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
    uint32_t addr = stm32f7_onchip_flash.addr + offset;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);
    for (size_t i = 0; i < size; i++, buf++, addr++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, (uint64_t)(*buf)) == HAL_OK)
        {
            if(*(__IO uint8_t *)addr != *buf) // readback check
            {
                HAL_FLASH_Lock();
                return -1;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }
    HAL_FLASH_Lock();
    return size;
}

static int erase(long offset, size_t size)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SECTORError = 0;
    uint32_t addr = stm32f7_onchip_flash.addr + offset;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = stm32_get_sector(addr);
    EraseInitStruct.NbSectors = stm32_get_sector(addr + size - 1) - EraseInitStruct.Sector + 1;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);
    if(HAL_FLASHEx_Erase(&EraseInitStruct, (uint32_t *)&SECTORError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }
    HAL_FLASH_Lock();
    return size;
}

const struct fal_flash_dev stm32f7_onchip_flash =
{
    .name       = "stm32_onchip",
    .addr       = ADDR_FLASH_SECTOR_0,
    .len        = 512 * 1024,
    .blk_size   = 128 * 1024,
    .ops        = {NULL, read, write, erase},
    .write_gran = 8
};

