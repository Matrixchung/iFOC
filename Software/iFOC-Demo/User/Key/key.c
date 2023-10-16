#include "key.h"

void key_scan(void)
{
    keyState key1;
    for(;;)
    {
        if(LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin) == 0)
        {
            if(key1.state == 0) key1.dbCnt++;
            if(key1.dbCnt >= KEY_DEBOUNCE_MS / KEY_SCAN_INTERVAL_MS)
            {
                if(key1.state == 0) key1.state = 1; // debounced
                if(key1.state == 1) 
                {
                    key1.lpCnt++; // long press counter starts to add
                    if(key1.lpCnt >= KEY_LONG_PRESS_MS / KEY_SCAN_INTERVAL_MS)
                    {
                        key1.state = 2; // long pressed
                        // KEY1 Long Pressed Callbacks (will be called only once)
                        xEventGroupSetBits(mainEventGroup, BIT_KEY_LONG_PRESSED);
                    }
                }
            }
        }
        else
        {
            key1.dbCnt = 0;
            key1.lpCnt = 0;
            if(key1.state == 1) // debounced
            {
                // KEY1 Pressed Callbacks
                xEventGroupSetBits(mainEventGroup, BIT_KEY_PRESSED);
            }
            else if(key1.state == 2) // long press released
            {
                // KEY1 Long Press Released Callbacks
                xEventGroupSetBits(mainEventGroup, BIT_KEY_LONG_PRESS_RELEASED);
            }
            key1.state = 0;
        }
        osDelay(KEY_SCAN_INTERVAL_MS);
    }
    
}