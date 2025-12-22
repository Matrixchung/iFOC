#include "hal_impl.hpp"
#include "hal_const.h"

#if defined(USE_HAL_DRIVER)

#include "stm32_nvm_address.h"

#if defined(STM32G431xx)
#define FLASH_ADDR_ALIGN (FLASH_WRITE_GRAN_BITS / 8)

static constexpr uint32_t GetPage(const uint32_t Addr)
{
    return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static constexpr uint32_t GetPageSize(const uint32_t page_number)
{
    return FLASH_PAGE_SIZE;
}

namespace iFOC::HAL::NVM
{
FuncRetCode Erase(uint32_t addr, size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    const auto first_page = GetPage(addr);
    size_t page_count = 0;
    size_t size_to_erase = 0;
    while(size_to_erase < size)
    {
        size_to_erase += GetPageSize(first_page + page_count);
        page_count++;
    }
    __unused uint32_t ret = 0;
    FLASH_EraseInitTypeDef EraseInitStruct
    {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_1,
        .Page = first_page,
        .NbPages = page_count
    };
    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &ret) != HAL_OK)
    {
        HAL_FLASH_Lock();
        __enable_irq();
        return FuncRetCode::HARDWARE_ERROR;
    }
    HAL_FLASH_Lock();
    __enable_irq();
    return FuncRetCode::OK;
}

FuncRetCode Write_NoErase(uint32_t addr, const uint8_t *buffer, size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    if(size % FLASH_ADDR_ALIGN) return FuncRetCode::ACCESS_VIOLATION;
    __ALIGN_BEGIN volatile uint64_t write_data __ALIGN_END = 0;
    __ALIGN_BEGIN volatile uint64_t read_data __ALIGN_END = 0;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    while(size)
    {
        memcpy((uint64_t*)&write_data, buffer, 8);
        read_data = *(uint64_t *)addr;
        if(write_data != read_data) // For STM32Gx, see: https://github.com/armink/FlashDB/issues/279, https://bbs.21ic.com/icview-3324732-1-1.html
        {
            uint8_t retry_time = 0;
            while(retry_time < 3)
            {
                if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, write_data) == HAL_OK) break;
                retry_time++;
            }
            read_data = *(uint64_t *)addr;
            if(write_data != read_data || retry_time >= 3)
//            if(retry_time >= 3)
            {
                HAL_FLASH_Lock();
                return FuncRetCode::HARDWARE_ERROR;
            }
        }
        addr += 8;
        buffer += 8;
        if(size >= 8) size -= 8;
        else break;
    }
    HAL_FLASH_Lock();
    return FuncRetCode::OK;
}

FuncRetCode Read(uint32_t addr, uint8_t *buffer, const size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    for(size_t i = 0; i < size; i++, addr++, buffer++)
    {
        *buffer = *(uint8_t*)addr;
    }
    return FuncRetCode::OK;
}

}

#endif

#if defined(STM32F401xE)
#define FLASH_ADDR_ALIGN (FLASH_WRITE_GRAN_BITS / 8)
/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
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
    else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_5))*/
    {
        sector = FLASH_SECTOR_5;
    }

    return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
    uint32_t sectorsize = 0x00;

    if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
    {
        sectorsize = 16 * 1024;
    }
    else if(Sector == FLASH_SECTOR_4)
    {
        sectorsize = 64 * 1024;
    }
    else
    {
        sectorsize = 128 * 1024;
    }
    return sectorsize;
}

namespace iFOC::HAL::NVM
{
FuncRetCode Erase(uint32_t addr, size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    const auto first_sector = GetSector(addr);
    size_t sector_count = 0;
    size_t size_to_erase = 0;
    while(size_to_erase < size)
    {
        size_to_erase += GetSectorSize(first_sector + sector_count);
        sector_count++;
    }
    __unused uint32_t ret = 0;
    FLASH_EraseInitTypeDef EraseInitStruct
    {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = FLASH_BANK_1,
        .Sector = first_sector,
        .NbSectors = sector_count,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    __disable_irq();
    HAL_FLASH_Unlock();
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &ret) != HAL_OK)
    {
        HAL_FLASH_Lock();
        __enable_irq();
        return FuncRetCode::HARDWARE_ERROR;
    }
    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

    __HAL_FLASH_DATA_CACHE_RESET();
    __HAL_FLASH_INSTRUCTION_CACHE_RESET();

    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();
    HAL_FLASH_Lock();
    __enable_irq();
    return FuncRetCode::OK;
}

FuncRetCode Write_NoErase(uint32_t addr, const uint8_t *buffer, size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    if(size % FLASH_ADDR_ALIGN) return FuncRetCode::ACCESS_VIOLATION;
    __ALIGN_BEGIN volatile uint32_t write_data __ALIGN_END = 0;
    __ALIGN_BEGIN volatile uint32_t read_data __ALIGN_END = 0;
    HAL_FLASH_Unlock();
    while(size)
    {
        memcpy((uint32_t*)&write_data, buffer, 4);
        read_data = *(uint32_t *)addr;
        if(write_data != read_data) // For STM32Gx, see: https://github.com/armink/FlashDB/issues/279, https://bbs.21ic.com/icview-3324732-1-1.html
        {
            uint8_t retry_time = 0;
            while(retry_time < 3)
            {
                if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, write_data) == HAL_OK) break;
                retry_time++;
            }
            read_data = *(uint32_t *)addr;
            if(write_data != read_data || retry_time >= 3)
//            if(retry_time >= 3)
            {
                HAL_FLASH_Lock();
                return FuncRetCode::HARDWARE_ERROR;
            }
        }
        addr += 4;
        buffer += 4;
        if(size >= 4) size -= 4;
        else break;
    }
    HAL_FLASH_Lock();
    return FuncRetCode::OK;
}

FuncRetCode Read(uint32_t addr, uint8_t *buffer, const size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    for(size_t i = 0; i < size; i++, addr++, buffer++)
    {
        *buffer = *(uint8_t*)addr;
    }
    return FuncRetCode::OK;
}

}
#endif

#if defined(STM32F070x6)
#define FLASH_USER_START_ADDR (ADDR_FLASH_PAGE_31)
#define FLASH_USER_END_ADDR   (ADDR_FLASH_PAGE_31 + FLASH_PAGE_SIZE - 1)
#define FLASH_ADDR_ALIGN (4)
#endif

#if defined(STM32F103xE)
#define FLASH_USER_START_ADDR (ADDR_FLASH_PAGE_120)
#define FLASH_USER_END_ADDR   (ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE - 1)
#define FLASH_ADDR_ALIGN (4)
static constexpr uint32_t GetPage(uint32_t Addr)
{
    return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}
#endif

#endif