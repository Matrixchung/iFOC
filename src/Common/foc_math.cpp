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

namespace iFOC
{
static unsigned long next = 1;
void ifoc_srand(const unsigned long seed)
{
    next = seed;
}

int ifoc_rand()
{
    next = next * 1103515245 + 12345;
    return ((next / 65536) % 32768);
}
}
