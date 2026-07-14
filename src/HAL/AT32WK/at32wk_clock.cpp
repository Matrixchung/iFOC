#include "hal_const.h"
#include "hal_impl.hpp"

#if defined(AT32WK_ENV)

#if defined(CoreDebug) && defined(DWT)
#define USE_DWT_COUNTER
#endif

#if defined(AT32F403AxG) || defined(AT32F407xx)
#else
uint32_t uptime_sec = 0;
#endif

namespace iFOC::HAL
{
    namespace PerfCounter
    {
        uint32_t max_counter = 0;

        uint32_t counter_to_us = 1;

        uint32_t max_counter_us = 0;

        void InitTimer()
        {
#ifdef USE_DWT_COUNTER
#endif
        }

        uint32_t GetCounter()
        {
#ifdef USE_DWT_COUNTER
            return DWT->CYCCNT;
#endif
            return 0;
        }
    }

    uint32_t GetCoreClockHz() { return SystemCoreClock; }

#if defined(AT32F403AxG) || defined(AT32F407xx)
    // F403A/407: simple 32-bit RTC counter, 1Hz from HEXT/128/62500
    uint32_t GetUptimeSeconds() { return rtc_counter_get(); }
#else
    uint32_t GetUptimeSeconds()
    {
        return uptime_sec;
    }
#endif

    void DelayInit()
    {
#ifdef USE_DWT_COUNTER
        /* Disable TRC */
        CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
        /* Enable TRC */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
        /* Disable Clock Cycle Counter */
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; // ~0x00000001;
        /* Enable Clock Cycle Counter */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 0x00000001;
        /* Reset the Clock Cycle Counter value */
        DWT->CYCCNT = 0;
        /* 3 NOPs */
        __ASM volatile ("NOP");
        __ASM volatile ("NOP");
        __ASM volatile ("NOP");
        // LL Driver turns off SysTick_Handler() interrupt without SysTick_CTRL_TICKINT_Msk set.
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                        SysTick_CTRL_TICKINT_Msk |
                        SysTick_CTRL_ENABLE_Msk;
        PerfCounter::InitTimer();
        PerfCounter::max_counter = 0xFFFF;
        PerfCounter::counter_to_us = GetCoreClockHz() / 1000000;
        PerfCounter::max_counter_us = PerfCounter::max_counter / PerfCounter::counter_to_us;
#endif
    }

    void DelayCycle(volatile uint32_t cycle)
    {
#ifdef USE_DWT_COUNTER
        uint32_t start_cycle = DWT->CYCCNT;
        while((DWT->CYCCNT - start_cycle) < cycle);
#else
        do{__NOP();} while(cycle--);
#endif
    }

    void SystemReboot()
    {
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
}

#endif
