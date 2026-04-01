#include "hal_impl.hpp"
#include "hal_const.h"

#if defined(GD32_ENV)

#include "gd32_nvm_address.h"

#if defined(GD32G5X3) // 256 K

static uint32_t GetPage(const uint32_t addr)
{
    return (addr - FLASH_BASE) / FLASH_SECTOR_SIZE_BYTES;
}
static uint32_t GetPageSize(const uint32_t page_number)
{
    (void)page_number;
    return fmc_page_size_get();
}

namespace iFOC::HAL::NVM
{
FuncRetCode Erase(const uint32_t addr, const size_t size)
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
    fmc_unlock();
    for(uint32_t i = 0; i < page_count; i++)
    {
        const uint32_t page_idx = first_page + i;
        fmc_page_erase(FMC_BANK0, page_idx);
    }
    fmc_lock();
    return FuncRetCode::OK;
}

FuncRetCode Write_NoErase(uint32_t addr, const uint8_t* buffer, size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    if(size % (FLASH_WRITE_GRAN_BITS / 8)) return FuncRetCode::ACCESS_VIOLATION;
    volatile uint64_t write_data = 0; // DOUBLE WORD
    volatile uint64_t read_data = 0;
    fmc_unlock();
    while(size)
    {
        memcpy((uint64_t*)&write_data, buffer, 8);
        read_data = *(uint64_t*)addr;
        if(write_data != read_data)
        {
            uint8_t retry_time = 0;
            while(retry_time < 3)
            {
                if(fmc_doubleword_program(addr, write_data) == FMC_READY) break;
                retry_time++;
            }
            read_data = *(uint64_t*)addr;
            if(write_data != read_data || retry_time >= 3)
            {
                fmc_lock();
                return FuncRetCode::HARDWARE_ERROR;
            }
        }
        addr += 8;
        buffer += 8;
        if(size >= 8) size -= 8;
        else break;
    }
    fmc_lock();
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

#endif