#include "hal_const.h"

#ifdef USE_FLASHDB

#include "fal.h"
#include "hal_impl.hpp"

static int read(long offset, uint8_t *buf, size_t size);
static int write(long offset, const uint8_t *buf, size_t size);
static int erase(long offset, size_t size);

const struct fal_flash_dev onchip_flash =
{
    .name       = "o", // short for "onchip"
    .addr       = (FLASH_USER_START_ADDR),
    .len        = (FLASH_USER_AREA_SIZE),
    .blk_size   = (FLASH_SECTOR_SIZE_BYTES),
    .ops        = {NULL, read, write, erase},
    .write_gran = (FLASH_WRITE_GRAN_BITS)
};

static int read(long offset, uint8_t *buf, size_t size)
{
    uint32_t addr = onchip_flash.addr + offset;
    if(iFOC::HAL::NVM::Read(addr, buf, size) != iFOC::FuncRetCode::OK) return 0;
    return size;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
    uint32_t addr = onchip_flash.addr + offset;
    if(iFOC::HAL::NVM::Write_NoErase(addr, buf, size) != iFOC::FuncRetCode::OK) return 0;
    return size;
}

static int erase(long offset, size_t size)
{
    uint32_t addr = onchip_flash.addr + offset;
    if(iFOC::HAL::NVM::Erase(addr, size) != iFOC::FuncRetCode::OK) return 0;
    return size;
}

// #include "ascii_tiny_printf.hpp"
// #include "cpp_classes.hpp"
//
// void fdb_debug_print(const char *format, ...)
// {
//     va_list args;
//
//     /* args point to the first variable parameter */
//     va_start(args, format);
//
//     /* You can add your code under here. */
//     char buffer[128];
//     auto len = (uint16_t)iFOC::vsnprintf_(buffer, sizeof(buffer), format, args);
//     if(uart_1.GetTxAvailable() < len) uart_1.StartTransmit(false);
//     uart_1.WriteBytes((const uint8_t*)buffer, len);
//     uart_1.StartTransmit(false);
//
//     va_end(args);
// }

#endif