#include "hal_impl.hpp"
#include "main.h"

#if defined(USE_HAL_DRIVER)

#include "stm32_nvm_address.h"

#if defined(STM32G431xx)
#define FLASH_USER_START_ADDR (ADDR_FLASH_PAGE_56)
#define FLASH_USER_END_ADDR   (ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE - 1)
#define FLASH_ADDR_ALIGN (8)
static constexpr uint32_t GetPage(const uint32_t Addr)
{
    return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

namespace iFOC::HAL::NVM
{

FuncRetCode EraseSector(const uint8_t sector)
{
    static constexpr auto page_count = GetPage(FLASH_USER_END_ADDR) - GetPage(FLASH_USER_START_ADDR) + 1;
    if(sector >= page_count) return FuncRetCode::PARAM_OUT_BOUND;
    const auto start_addr = FLASH_USER_START_ADDR + sector * FLASH_PAGE_SIZE;
    const auto first_page = GetPage(start_addr);
    uint32_t ret = 0;
    FLASH_EraseInitTypeDef EraseInitStruct{
            .TypeErase = FLASH_TYPEERASE_PAGES,
            .Banks = FLASH_BANK_1,
            .Page = first_page,
            .NbPages = 1
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

FuncRetCode Write_NoErase(const uint8_t sector, const uint8_t *buffer, uint16_t size)
{
    if(size % FLASH_ADDR_ALIGN) return FuncRetCode::ACCESS_VIOLATION;
    // skipped check of page_count
    auto start_addr = FLASH_USER_START_ADDR + sector * FLASH_PAGE_SIZE;
    const auto end_addr = start_addr + FLASH_PAGE_SIZE - 1;
    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    while(size && start_addr < end_addr)
    {
        volatile uint64_t temp = 0;
        memcpy((uint64_t*)&temp, buffer, 8);
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr, temp) == HAL_OK)
        {
            start_addr += 8;
            buffer += 8;
            size -= 8;
        }
        else
        {
            HAL_FLASH_Lock();
            __enable_irq();
            return FuncRetCode::HARDWARE_ERROR;
        }
    }
    HAL_FLASH_Lock();
    __enable_irq();
    return FuncRetCode::OK;
}

FuncRetCode Read(const uint8_t sector, uint8_t *buffer, uint16_t size)
{
    if(size % FLASH_ADDR_ALIGN) return FuncRetCode::ACCESS_VIOLATION;
    auto start_addr = FLASH_USER_START_ADDR + sector * FLASH_PAGE_SIZE;
    if(start_addr >= FLASH_USER_END_ADDR) return FuncRetCode::PARAM_OUT_BOUND;
    while(size)
    {
        uint32_t data = *(volatile uint32_t *)start_addr;
        memcpy(buffer, &data, 4);
        start_addr += 4;
        size -= 4;
        buffer += 4;
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