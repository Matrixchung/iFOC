#pragma once

#include "../Common/foc_types.hpp"
#include "../DataType/Headers/Base/bootloader_msg.h"
#include "hal_const.h"

/*
 * Apart from platform-specific peripherals you would wish to use,
 * the functions here are those you need to implement globally, for
 * correct functionality of iFOC Library.
 *
 * By #include "hal_impl.hpp", you can (and must) implement those functions.
 */

namespace iFOC::HAL
{

/// Calculate sine and cosine value based on given input radian.
/// \param theta input angle [rad], range: [0, 2PI]
/// \param sin floating point representation of sin(theta). range: [-1, 1]
/// \param cos floating point representation of cos(theta). range: [-1, 1]
void sinf_cosf_impl(const real_t theta, real_t& sin, real_t& cos);

/// Initialize delay part.
void DelayInit();

/// Delay a specific length of CPU cycle. DelayUs() and DelayMs() are based on this
/// \param cycle number of CPU cycles to be delayed
void DelayCycle(volatile uint32_t cycle);

/// Get current core frequency, in [Hz]
/// \return core frequency, in [Hz]
uint32_t GetCoreClockHz();

/// Get Serial Number (strongly required unique across all devices). Typically we use 32-bits SN.
/// \return serial number, in uint32_t. Range: [0, 4294967295]
uint32_t GetSerialNumber();

/// Get uptime seconds, in [sec]
/// \return current uptime, in [sec]
uint32_t GetUptimeSeconds();

/// Get current firmware size (without Bootloader), in [bytes] \n
/// This function helps to calculate firmware's CRC64, which is useful in IAP. \n
/// The return value should exactly match the size of compiled binary file. \n
/// If you cannot get accurate compiled size, simply implement it with return 0; \n
/// In GCC, you could get the size by editing linker script \n
/// For example, add __firmware_start = ORIGIN(FLASH); \n
/// and another flag __firmware_end = LOADADDR(.data) + SIZEOF(.data); \n
///                  PROVIDE(__firmware_end = __firmware_end); \n
/// the two rows are in the end of FLASH section (likely after .data section, we need to cover all LMA),
/// under SECTION {} bracket. \n
/// Note that the update of compiled .map (and flag) is expected to happen only when Clean & Rebuild. \n
/// \return current firmware size, in [bytes]
uint32_t GetFirmwareSizeBytes();

/// Get current firmware CRC64 (without Bootloader) \n
/// This function is mandatory in IAP, combined with GetFirmwareSizeBytes(). \n
/// Using CRC64 ECMA-182 Standard, check get_crc64() in foc_math.hpp. \n
/// This function MUST NOT CALCULATE CRC64 AT RUNTIME due to heavy load! \n
/// The best way is calculating the CRC64 during startup, and store it with static variable. \n
/// \return current firmware's CRC64, start from FLASH_BASE to FLASH_BASE + GetFirmwareSizeBytes(). \n
uint64_t GetFirmwareCRC64();

/// This function does what it said.
void SystemReboot();

/// Provides an interface to access the bootloader.
namespace Bootloader
{
    /// This function does what it said. \n
    /// \param msg message that is passed to the bootloader,
    /// and how to deal with the message is up to implementation.
    void JumpToBL(const BootloaderMsg& msg);

    /// \return true if a correct bootloader is presented, false if not.
    bool HasBL();

    /// Get bootloader version described in major.minor.vcs_commit. \n
    /// set to 0.0.00000000 if bootloader is damaged or not presented. \n
    /// \param major reference that will be used to store read out MAJOR value
    /// \param minor reference that will be used to store read out MINOR value
    /// \param vcs_commit reference that will be used to store read out VCS_COMMIT.
    void GetBLVersion(uint8_t& major, uint8_t& minor, uint32_t& vcs_commit);
}

/// Provides an interface to Non-Volatile Memory (NVM) for persistent storage use.
/// Used in: iFOC::DataType::ConfigNVMWrapper
namespace NVM
{
    FuncRetCode Erase(uint32_t addr, size_t size);

    FuncRetCode Write_NoErase(uint32_t addr, const uint8_t *buffer, size_t size);

    FuncRetCode Read(uint32_t addr, uint8_t *buffer, size_t size);
}

/// Provides a General Purpose Timer (or DWT) for performance metrics use.
/// Used in: iFOC::TaskTimer
namespace PerfCounter
{
    /// Max readout value of GetCounter(), typically 0xFFFF (for 32-bit timer).
    extern uint32_t max_counter;

    /// Defines how many counter cycles represent one us, typically GetCoreClockHz() / 1000000.
    /// Leave non-zero value if PerfCounter is not used!!
    extern uint32_t counter_to_us;

    /// Typically max_counter / counter_to_us.
    extern uint32_t max_counter_us;

    /// Initialize and start the timer used for performance counter.
    void InitTimer();

    /// Get the current counter from timer
    /// \return uint32_t type of counter value.
    uint32_t GetCounter();
}

}