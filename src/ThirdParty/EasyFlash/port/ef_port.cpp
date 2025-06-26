/*
 * This file is part of the EasyFlash Library.
 *
 * Copyright (c) 2015-2019, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-01-16
 */

#include <easyflash.h>
#include "hal_impl.hpp"

#include "cpp_classes.hpp"
#include "ascii_tiny_printf.hpp"

#ifdef USE_EASYFLASH

#include <cstdarg>

namespace iFOC::HAL::NVM
{

/* default environment variables set for user */
//static const ef_env default_env_set[] = {
////        {(char *) "x1", (void *) "0"},
//};

/**
* Flash port for hardware initialize.
*
* @param default_env default ENV set for user
* @param default_env_size default ENV size
*
* @return result
*/
EfErrCode ef_port_init(ef_env const **default_env, size_t *default_env_size) {
    EfErrCode result = EF_NO_ERR;

//    *default_env = default_env_set;
//    *default_env_size = sizeof(default_env_set) / sizeof(default_env_set[0]);

    return result;
}

/**
* Read data from flash.
* @note This operation's units is word.
*
* @param addr flash address
* @param buf buffer to store read data
* @param size read bytes size
*
* @return result
*/
EfErrCode ef_port_read(uint32_t addr, uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;

    /* You can add your code under here. */

    auto *buf_8 = (uint8_t*)buf;
    if(Read(addr, buf_8, size) != FuncRetCode::OK) result = EF_READ_ERR;

    return result;
}

/**
* Erase data on flash.
* @note This operation is irreversible.
* @note This operation's units is different which on many chips.
*
* @param addr flash address
* @param size erase bytes size
*
* @return result
*/
EfErrCode ef_port_erase(uint32_t addr, size_t size) {
    EfErrCode result = EF_NO_ERR;

    /* make sure the start address is a multiple of EF_ERASE_MIN_SIZE */
    EF_ASSERT(addr % EF_ERASE_MIN_SIZE == 0);

    /* You can add your code under here. */

    if(Erase(addr, size) != FuncRetCode::OK) result = EF_ERASE_ERR;

    return result;
}

/**
* Write data to flash.
* @note This operation's units is word.
* @note This operation must after erase. @see flash_erase.
*
* @param addr flash address
* @param buf the write data buffer
* @param size write bytes size
*
* @return result
*/
EfErrCode ef_port_write(uint32_t addr, const uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;

    /* You can add your code under here. */

    auto *buf_8 = (uint8_t*)buf;
    if(Write_NoErase(addr, buf_8, size) != FuncRetCode::OK) result = EF_WRITE_ERR;

    return result;
}

SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

/**
* lock the ENV ram cache
*/
void ef_port_env_lock() {

    /* You can add your code under here. */
    xSemaphoreTake(mutex, portMAX_DELAY);
}

/**
* unlock the ENV ram cache
*/
void ef_port_env_unlock() {

    /* You can add your code under here. */
    xSemaphoreGive(mutex);
}


/**
* This function is print flash debug info.
*
* @param file the file which has call this function
* @param line the line number which has call this function
* @param format output format
* @param ... args
*
*/
void ef_log_debug(const char *file, const long line, const char *format, ...) {

#ifdef PRINT_DEBUG

    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);

    /* You can add your code under here. */
    uart_1.Print(false, "[%s:%d]", file, line);

    char buffer[128];
    auto len = (uint16_t)iFOC::vsnprintf_(buffer, sizeof(buffer), format, args);
    if(uart_1.GetTxAvailable() < len) uart_1.StartTransmit(false);
    uart_1.WriteBytes((const uint8_t*)buffer, len);
    uart_1.StartTransmit(false);

    va_end(args);

#endif

}

/**
* This function is print flash routine info.
*
* @param format output format
* @param ... args
*/
void ef_log_info(const char *format, ...) {
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);

    /* You can add your code under here. */
    char buffer[128];
    auto len = (uint16_t)iFOC::vsnprintf_(buffer, sizeof(buffer), format, args);
    if(uart_1.GetTxAvailable() < len) uart_1.StartTransmit(false);
    uart_1.WriteBytes((const uint8_t*)buffer, len);
    uart_1.StartTransmit(false);

    va_end(args);
}

/**
* This function is print flash non-package info.
*
* @param format output format
* @param ... args
*/
void ef_print(const char *format, ...) {
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);

    /* You can add your code under here. */
    char buffer[128];
    auto len = (uint16_t)iFOC::vsnprintf_(buffer, sizeof(buffer), format, args);
    if(uart_1.GetTxAvailable() < len) uart_1.StartTransmit(false);
    uart_1.WriteBytes((const uint8_t*)buffer, len);
    uart_1.StartTransmit(false);

    va_end(args);
}

}

#endif