#include "hal_impl.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV)

#include "at32wk_nvm_address.h"

#if defined(AT32F403AxG) // 1024 K
static constexpr uint32_t GetSector(const uint32_t Addr)
{
    return (Addr - FLASH_BASE) / FLASH_SECTOR_SIZE_BYTES;
}
static constexpr uint32_t GetSectorSize(const uint32_t sector_number)
{
    return FLASH_SECTOR_SIZE_BYTES;
}
namespace iFOC::HAL::NVM
{
FuncRetCode Erase(uint32_t addr, size_t size) // erase sectors
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    // generate sector count
    const auto first_sector = GetSector(addr);
    size_t sector_count = 0;
    size_t size_to_erase = 0;
    while(size_to_erase < size)
    {
        size_to_erase += GetSectorSize(first_sector + sector_count);
        sector_count++;
    }
    // generate first sector address
    const uint32_t first_sector_addr = FLASH_BASE + first_sector * FLASH_SECTOR_SIZE_BYTES; // absolute address
    flash_unlock();
    for(uint32_t i = 0; i < sector_count; i++)
    {
        const uint32_t sector_addr = first_sector_addr + i * FLASH_SECTOR_SIZE_BYTES;
#if FLASH_SECTOR_SIZE_BYTES == (1024 * 1)
        flash_sector_erase(sector_addr);
#elif FLASH_SECTOR_SIZE_BYTES == (1024 * 2) || FLASH_SECTOR_SIZE_BYTES == (1024 * 4)
        if((sector_addr & (FLASH_SECTOR_SIZE_BYTES - 1)) == 0) flash_sector_erase(sector_addr);
#else
#error "FLASH_SECTOR_SIZE_BYTES not correct!"
#endif
    }
    flash_lock();
    return FuncRetCode::OK;
}

FuncRetCode Write_NoErase(uint32_t addr, const uint8_t* buffer, size_t size)
{
    if(addr < FLASH_USER_START_ADDR || addr > FLASH_USER_START_ADDR + FLASH_USER_AREA_SIZE) return FuncRetCode::ACCESS_VIOLATION;
    if(size % (FLASH_WRITE_GRAN_BITS / 8)) return FuncRetCode::ACCESS_VIOLATION;
    volatile uint32_t write_data = 0;
    volatile uint32_t read_data = 0;
    flash_unlock();
    while(size)
    {
        memcpy((uint32_t*)&write_data, buffer, 4);
        read_data = *(uint32_t*)addr;
        if(write_data != read_data)
        {
            uint8_t retry_time = 0;
            while(retry_time < 3)
            {
                if(flash_word_program(addr, write_data) == FLASH_OPERATE_DONE) break;
                retry_time++;
            }
            read_data = *(uint32_t*)addr;
            if(write_data != read_data || retry_time >= 3)
            {
                flash_lock();
                return FuncRetCode::HARDWARE_ERROR;
            }
        }
        addr += 4;
        buffer += 4;
        if(size >= 4) size -= 4;
        else break;
    }
    flash_lock();
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
