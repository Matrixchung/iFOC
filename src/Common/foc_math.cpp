#include "foc_math.hpp"

namespace iFOC::HAL
{
    void DelayUs(volatile uint32_t us)
    {
        DelayCycle(us * (GetCoreClockHz() / 1000000));
    }
    void DelayMs(uint32_t ms)
    {
        while(ms--) DelayUs(1000);
    }
    void osDelayMs(uint32_t ms)
    {
        vTaskDelay(ms / portTICK_PERIOD_MS);
    }
}