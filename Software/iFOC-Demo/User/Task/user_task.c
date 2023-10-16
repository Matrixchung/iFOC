#include "user_task.h"

osThreadId uartTaskHandle;
osThreadId keyScanHandle;
osThreadId screenTaskHandle;
osThreadId testTaskHandle;

EventGroupHandle_t mainEventGroup;

float target_angle = 0.0f;
float target_velocity = 0.0f;

void uartTask(void const * argument);
void keyScanTask(void const * argument);
void screenTask(void const * argument);
void testTask(void const * argument);

void uartTask(void const * argument)
{
    uint16_t rxLen = 0;
    char buffer[128] = {0};
    char command_buffer[MAX_COMMAND_WORD_LENGTH] = {0};
    float arg_buffer[MAX_FLOAT_ARG_COUNT] = {0.0f};
    uint8_t arg_buffer_len = 0;
    for(;;)
    {
        if(osSemaphoreWait(uart_rx_finished_sem, 0) == osOK)
        {
            rxLen = UART_GetRxLen();
            if(rxLen > 0)
            {
                if(UART_PeekRxFifo(rxLen-1) == '\r')
                {
                    UART_GetRxFifo(buffer, rxLen);
                    // Process buffer
                    getCommand(buffer, rxLen, command_buffer);
                    // test
                    // splitData_f(buffer, rxLen, arg_buffer, &arg_buffer_len, ' ');
                    // UART_printf("get command '%s' with %d arguments\n", command_buffer, arg_buffer_len);
                    // for(uint8_t i = 0; i < arg_buffer_len; i++) UART_printf("arg%d: %.3f\n", i, arg_buffer[i]);

                    // REMEMBER: strcmp returns 0 when equals!
                    if(strcmp(command_buffer, "target_angle") == 0)
                    {
                        splitData_f(buffer, rxLen, arg_buffer, &arg_buffer_len, ' ');
                        if(arg_buffer_len == 1)
                        {
                            target_angle = arg_buffer[0];
                        }
                    }
                    else if(strcmp(command_buffer, "target_velocity") == 0)
                    {
                        splitData_f(buffer, rxLen, arg_buffer, &arg_buffer_len, ' ');
                        if(arg_buffer_len == 1)
                        {
                            target_velocity = arg_buffer[0];
                        }
                    }
                    // else if(strcmp(command_buffer, "adc") == 0)
                    // {
                    //     UART_printf("CH0: %d, CH1: %d, CH2: %d, CH3: %d\n", 
                    //                 ADC_GetRawValue(0), 
                    //                 ADC_GetRawValue(1), 
                    //                 ADC_GetRawValue(2), 
                    //                 ADC_GetRawValue(3));
                    //     UART_printf("VBAT: %.3f, CH1R: %.3f, CH2R: %.3f\n", 
                    //                 ADC_GetVBAT(), 
                    //                 ADC_GetCalibratedValue(1), 
                    //                 ADC_GetCalibratedValue(2));
                    // }

                    // UART_PutTxFifo(buffer, rxLen);
                    // UART_Transmit(TX_BLOCK);
                    // UART_printf("Received len:%d\n", rxLen);
                }
            }
        }
        else
        {
            // vofa_add_float(3, ADC_GetRawValue(0));
            // vofa_add_float(4, ADC_GetRawValue(1));
            // vofa_add_float(5, ADC_GetRawValue(2));
            // vofa_add_float(6, ADC_GetRawValue(3));
            vofa_add_float(0, ADC_GetMidRefVoltage(1));
            vofa_add_float(1, ADC_GetMidRefVoltage(2));
            vofa_add_float(2, -ADC_GetMidRefVoltage(1)-ADC_GetMidRefVoltage(2));
            vofa_add_float(3, SPI_Encoder_GetRawAngle());
            vofa_add_float(4, FOC_GetElecRad());
            vofa_add_float(5, PWM_Encoder_GetRad());
            vofa_add_float(6, foc_driver.velocity);
            vofa_add_float(7, ADC_GetTemperature());
            vofa_send_data();
            osDelay(1);
        }
    }
}

void screenTask(void const * argument)
{
    EventBits_t uxBits;
    for(;;)
    {
        uxBits = xEventGroupWaitBits(mainEventGroup, 
                                     BIT_KEY_PRESSED | BIT_KEY_LONG_PRESSED | BIT_KEY_LONG_PRESS_RELEASED, 
                                     pdTRUE, 
                                     pdFALSE, 
                                     0);
        if((uxBits & BIT_KEY_PRESSED) == BIT_KEY_PRESSED)
        {
            // UART_printf("Key pressed\n");
            OLED_Clear(OLED_NREFRESH);
            OLED_printf(0, 0, 12, "Key pressed");
            OLED_RefreshGram();
        }
        else if((uxBits & BIT_KEY_LONG_PRESSED) == BIT_KEY_LONG_PRESSED)
        {
            // UART_printf("Key long pressed\n");
            OLED_Clear(OLED_NREFRESH);
            OLED_printf(0, 0, 12, "Long pressed");
            OLED_RefreshGram();
        }
        else if((uxBits & BIT_KEY_LONG_PRESS_RELEASED) == BIT_KEY_LONG_PRESS_RELEASED)
        {
            // UART_printf("Key long press released\n");
            OLED_Clear(OLED_NREFRESH);
            OLED_printf(0, 0, 12, "Long press released");
            OLED_RefreshGram();
        }
        else
        {
            OLED_Clear(OLED_NREFRESH);
            // OLED_printf(0, 0, 12, "Enc_Angle: %.6f", PWM_Encoder_GetAngle());
            // OLED_printf(0, 10, 12, "Encoder_Freq: %.2f", encoder_pwm_freq);
            // OLED_printf(0, 12, 12, "VBAT: %.2f V", ADC_GetVBAT());
            // OLED_printf(0, 24, 12, "VDD: %.2f V", ADC_GetVDD());
            // OLED_printf(0, 36, 12, "I1s: %.3f V", ADC_GetMidRefVoltage(1));
            // OLED_printf(0, 48, 12, "I2s: %.3f V", ADC_GetMidRefVoltage(2));
            OLED_printf(0, 0, 12, "enc_raw: %.6f", PWM_Encoder_GetRawRad());
            OLED_printf(0, 12, 12, "VBAT: %.2f V", ADC_GetVBAT());
            OLED_printf(0, 24, 12, "zero_elec_angle: %.2f", foc_driver.zero_electric_angle);
            OLED_printf(0, 36, 12, "acc._rad: %.2f", PWM_Encoder_GetRad());
            OLED_RefreshGram();
            osDelay(500);
        }

    }
}

void keyScanTask(void const * argument)
{
    key_scan();
}
uint8_t foc_loopstate = 0;
void testTask(void const * argument)
{
    // float t = 0;
    EventBits_t uxBits;
    for(;;)
    {
        // // LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        // // UART_printf("iFOC: heartbeat 0.5s\n");
        // vofa_add_float(0, sin(t));
        // vofa_add_float(1, cos(t));
        // t += 0.02;
        // if(t >= 1000) t = 0;
        uxBits = xEventGroupWaitBits(mainEventGroup, 
                                     BIT_KEY_PRESSED,
                                     pdTRUE, 
                                     pdFALSE, 
                                     0);
        if((uxBits & BIT_KEY_PRESSED) == BIT_KEY_PRESSED) foc_loopstate = !foc_loopstate;
        if(foc_loopstate) LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
        else LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
        // if(foc_loopstate)
        // {
        //     // FOC_Position_Closeloop(&foc_driver, motor_target);
        //     FOC_Velocity_Openloop(&foc_driver, 20);
        // }
        // else FOC_Driver_ResetOutput();
        osDelay(1);
    }
}

void FOC_Loop_IRQHandler_LL(void)
{
    if(LL_TIM_IsActiveFlag_UPDATE(TIM2) != RESET)
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);
        FOC_CalcVelocity(&foc_driver);
        if(foc_loopstate)
        {
            // FOC_Position_Closeloop(&foc_driver, target_angle);
            // FOC_Velocity_Openloop(&foc_driver, target_velocity);
            FOC_Velocity_Closeloop(&foc_driver, target_velocity);
        }
        else FOC_Driver_ResetOutput();
    }
}

void StartUserTask(void)
{
    mainEventGroup = xEventGroupCreate();
    vofa_init();

    // osThreadDef(screen_task, screenTask, osPriorityNormal, 0, 256);
    // screenTaskHandle = osThreadCreate(osThread(screen_task), NULL);
    // UART_printf("iFOC: screen task created\n");

    osThreadDef(uart_task, uartTask, osPriorityAboveNormal, 0, 256);
    uartTaskHandle = osThreadCreate(osThread(uart_task), NULL);
    // UART_printf("iFOC: uart task created\n");

    osThreadDef(key_scan_task, keyScanTask, osPriorityNormal, 0, 128);
    keyScanHandle = osThreadCreate(osThread(key_scan_task), NULL);
    // UART_printf("iFOC: key task created\n");

    osThreadDef(test_task, testTask, osPriorityBelowNormal, 0, 128);
    testTaskHandle = osThreadCreate(osThread(test_task), NULL);
    // UART_printf("iFOC: test task created\n");

    osThreadTerminate(NULL);
}