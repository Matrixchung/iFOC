//
// Created by vip99 on 1/21/2026.
//
#include "cpp_classes.hpp"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uart3->OnIdleIRQ(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uart3->OnTxCpltIRQ(huart);
}

void ADC1_2_IRQHandler(void)
{
    // WARNING: The ADC which is triggering JEOS interrupt MUST have the longest period of conversion
    //          because we will clear all the adc flags in a row.
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
    LL_ADC_ClearFlag_JEOS(ADC3);
    if(LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP)
    {
        LL_ADC_DisableIT_JEOS(ADC1);
        MEASURE_TIME(motor_1->task_times.rt_main_task)
        {
            motor_1->DispatchRTTasks(iFOC::RT_LOOP_TS);
        }
        MEASURE_TIME(motor_1->task_times.rt_waiting_for_remaining)
        {
            while(!LL_ADC_IsActiveFlag_JEOS(ADC1) && !LL_ADC_IsActiveFlag_JEOS(ADC2) && !LL_ADC_IsActiveFlag_JEOS(ADC3));
        }
        LL_ADC_ClearFlag_JEOS(ADC1);
        LL_ADC_ClearFlag_JEOS(ADC2);
        LL_ADC_ClearFlag_JEOS(ADC3);
        MEASURE_TIME(motor_1->task_times.rt_remaining_task)
        {
            motor_1->GetCurrSense()->UpdateRemainingCurrent(iFOC::RT_LOOP_TS);
        }
        LL_ADC_EnableIT_JEOS(ADC1);
    }
}
void TIM1_UP_TIM16_IRQHandler(void)
{
    LL_TIM_ClearFlag_UPDATE(TIM16);
    MEASURE_TIME(motor_1->task_times.mid_interval_task)
    {
        motor_1->DispatchMidTasks(iFOC::MID_LOOP_TS);
        vofa_output();
    }
}

#if defined(configUSE_MALLOC_FAILED_HOOK)
void vApplicationMallocFailedHook()
{
    motor_1->DisarmWithError(iFOC::MotorError::SYSTEM_MEM_ALLOCATION_FAILED);
}
#endif

#if defined(configCHECK_FOR_STACK_OVERFLOW)
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    motor_1->DisarmWithError(iFOC::MotorError::SYSTEM_OS_TASK_STACK_OVERFLOW);
}
#endif