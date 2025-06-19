#pragma once

#include "foc_types.hpp"

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

/// Get Serial Number (strongly required unique across all devices). Typically we use 48-bits SN.
/// \return serial number, in uint64_t. Range: [0, 18,446,744,073,709,551,615]
uint64_t GetSerialNumber();

/// This function does what it said.
void SystemReboot();

/// Provides an interface to Non-Volatile Memory (NVM) for persistent storage use.
/// Used in: iFOC::DataType::ConfigNVMWrapper
namespace NVM
{
    FuncRetCode EraseSector(uint8_t sector);

    FuncRetCode Write_NoErase(uint8_t sector, const uint8_t *buffer, uint16_t size);

    FuncRetCode Read(uint8_t sector, uint8_t* buffer, uint16_t size);
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