#include "hal_const.h"

#if defined(GD32_ENV)

#if defined(GD32G5X3)
#include "gd32g5x3.h"

#if defined(CoreDebug) && defined(DWT)
#define USE_DWT_COUNTER

uint32_t uptime_sec = 0;

namespace iFOC::HAL
{
    namespace PerfCounter
    {
        uint32_t counter_to_us = 1;

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

    uint32_t GetUptimeSeconds() { return uptime_sec; }

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
        PerfCounter::counter_to_us = GetCoreClockHz() / 1000000;
#endif
    }

    void DelayCycle(volatile uint32_t cycle)
    {
#ifdef USE_DWT_COUNTER
        const uint32_t start_cycle = DWT->CYCCNT;
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

#endif

#endif