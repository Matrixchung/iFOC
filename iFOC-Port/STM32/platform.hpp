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
#if MCU == F7
    #define USE_INTERNAL_FLASH

#endif
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

#ifdef USE_INTERNAL_FLASH
#include "fal_cfg.h"
#include "flashdb.h"
#include "string.h"
typedef struct nvm_config_t
{
    uint32_t boot_time;
    uint32_t serial_number;
}nvm_config_t;
nvm_config_t user_config;
static struct fdb_kvdb kvdb = {0};
static void _lock_db(fdb_db_t db)
{
    __disable_irq();
}
static void _unlock_db(fdb_db_t db)
{
    __enable_irq();
}
#define FDB_LOG_TAG "[main]"
void blob_sample(fdb_kvdb_t kvdb)
{
    struct fdb_blob blob;
    nvm_config_t readback_config;
    memset(&readback_config, 0, sizeof(readback_config));
    fdb_kv_get_blob(kvdb, "user_config", fdb_blob_make(&blob, &readback_config, sizeof(readback_config)));
    if(blob.saved.len > 0)
    {
        FDB_INFO("Readout boot_time: %d, serial_number: %d", readback_config.boot_time, readback_config.serial_number);
        if(readback_config.serial_number != (uint32_t)GetSerialNumber())
        {
            readback_config.serial_number = (uint32_t)GetSerialNumber();
            FDB_INFO("Set serial_number to: %d\n", readback_config.serial_number);
        }
        readback_config.boot_time++;
        fdb_kv_set_blob(kvdb, "user_config", fdb_blob_make(&blob, &readback_config, sizeof(readback_config)));
    }
    else
    {
        FDB_INFO("user_config not found, creating\n");
        readback_config.boot_time = 1;
        readback_config.serial_number = 0;
        fdb_kv_set_blob(kvdb, "user_config", fdb_blob_make(&blob, &readback_config, sizeof(readback_config)));
    }
}
void StorageInit()
{
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_LOCK, (void *)_lock_db);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void *)_unlock_db);
    fdb_kvdb_init(&kvdb, "env", "nvm_storage", NULL, NULL);
    blob_sample(&kvdb);
}
// #if MCU == F7

// // STM32F72xxx & STM32F732xx / F733xx, with a total of 512 Kbytes
// #define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes, start with user code area */
// #define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes */
// #define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes */
// #define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes */
// #define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes */
// #define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes */
// #define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes */

// #define USER_STORAGE_START_ADDR (ADDR_FLASH_SECTOR_5)
// #define USER_STORAGE_END_ADDR   (ADDR_FLASH_SECTOR_6)

// static uint32_t GetSector(uint32_t Address)
// {
//   uint32_t sector = 0;

//   if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
//   {
//     sector = FLASH_SECTOR_0;
//   }
//   else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
//   {
//     sector = FLASH_SECTOR_1;
//   }
//   else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
//   {
//     sector = FLASH_SECTOR_2;
//   }
//   else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
//   {
//     sector = FLASH_SECTOR_3;
//   }
//   else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
//   {
//     sector = FLASH_SECTOR_4;
//   }
//   else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
//   {
//     sector = FLASH_SECTOR_5;
//   }
//   else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
//   {
//     sector = FLASH_SECTOR_6;
//   }
//   else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
//   {
//     sector = FLASH_SECTOR_7;
//   }
//   return sector;
// }

// uint32_t WriteFlash(uint32_t addr, uint32_t *data, const uint16_t len)
// {
//   if(addr < USER_STORAGE_START_ADDR) return HAL_ERROR;
//   uint32_t end_addr = addr + len;
//   if(end_addr > USER_STORAGE_END_ADDR) return HAL_ERROR;
//   FLASH_EraseInitTypeDef EraseInitStruct;
//   uint32_t SECTORError = 0;
//   HAL_FLASH_Unlock();
//   EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//   EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//   EraseInitStruct.Sector = GetSector(addr);
//   EraseInitStruct.NbSectors = GetSector(end_addr) - EraseInitStruct.Sector + 1;
//   if(HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) 
//   {
//     HAL_FLASH_Lock();
//     // __enable_irq();
//     return HAL_FLASH_GetError();
//   }
//   while(addr < end_addr)
//   {
//       if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *data) == HAL_OK)
//       {
//           addr += 4;
//           data += 1;
//       }
//       else
//       {
//           HAL_FLASH_Lock();
//           return HAL_ERROR;
//       }
//   }
//   HAL_FLASH_Lock();
//   return HAL_OK;
// }
// uint32_t CheckProgrammedFlash(uint32_t addr, uint32_t *data, const uint16_t len)
// {
//     uint32_t ret = 0; // number of bytes not programmed correctly (*data != *addr)
//     if(addr < USER_STORAGE_START_ADDR) return 0;
//     uint32_t end_addr = addr + len;
//     if(end_addr > USER_STORAGE_END_ADDR) return 0;
//     while(addr < end_addr)
//     {
//         uint32_t read_back_data = READ_U32(addr);
//         if(read_back_data != *data) ret++;
//         addr += 4;
//         data += 1;
//     }
//     return ret;
// }
// HAL_StatusTypeDef ReadFlash(uint32_t addr, uint32_t *dst, const uint16_t len)
// {
//     if(addr < USER_STORAGE_START_ADDR) return HAL_ERROR;
//     uint32_t end_addr = addr + len;
//     if(end_addr > USER_STORAGE_END_ADDR) return HAL_ERROR;
//     while(addr < end_addr)
//     {
//         *dst = READ_U32(addr);
//         addr += 4;
//         dst += 1;
//     }
//     return HAL_OK;
// }
// Example code
// #pragma pack(4) // IMPORTANT!
// typedef struct storage_t
// {
//     uint32_t poweron_times;
//     uint64_t serial_number;
//     uint32_t crc_missed_times;
//     uint16_t crc16;
// }storage_t;
// #pragma pack()

// #define STORAGE_ADDR USER_STORAGE_START_ADDR

// storage_t readback_storage;

// void handle_storage()
// {
//     ReadFlash(STORAGE_ADDR, (uint32_t*)&readback_storage, sizeof(readback_storage));
//     uint16_t readback_crc = getCRC16((uint8_t*)&readback_storage, sizeof(readback_storage) - sizeof(uint32_t));
//     if(readback_crc != readback_storage.crc16) 
//     {
//         readback_storage = {.poweron_times = 0, .serial_number = 0, .crc_missed_times = 0, .crc16 = 0};
//         readback_storage.crc_missed_times++;
//     }
//     readback_storage.poweron_times++;
//     readback_storage.serial_number = GetSerialNumber();
//     readback_storage.crc16 = getCRC16((uint8_t*)&readback_storage, sizeof(readback_storage) - sizeof(uint32_t));
//     WriteFlash(STORAGE_ADDR, (uint32_t*)&readback_storage, sizeof(readback_storage));
// }
// #endif
#endif

#endif