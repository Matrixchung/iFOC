#include "cpp_classes.hpp"

void adc_1_2_irq(void)
{
    // adc_flag_clear(ADC2, ADC_PCCE_FLAG);
    // adc_flag_clear(ADC3, ADC_PCCE_FLAG);
    ADC2->sts = ~ADC_PCCE_FLAG;
    ADC3->sts = ~ADC_PCCE_FLAG;
    // cnt_dir: TMR_CTRL1[6:4], while TWCMSEL: TMR_CTRL1[6:5], OWCDIR: TMR_CTRL1[4] (See Reference Manual)
    if((TMR1->ctrl1_bit.cnt_dir & 0x01) == 0x00000000U) // COUNTERDIRECTION_UP
    {
        // adc_interrupt_enable(ADC2, ADC_PCCE_INT, FALSE);
        ADC2->ctrl1 &= ~ADC_PCCE_INT;
        // ADC_Shunt_Filter();
        MEASURE_TIME(motor_1->task_times.rt_main_task)
        {
            motor_1->DispatchRTTasks(iFOC::RT_LOOP_TS);
        }
        MEASURE_TIME(motor_1->task_times.rt_waiting_for_remaining)
        {
            // while(!adc_flag_get(ADC2, ADC_PCCE_FLAG));
            while(!(ADC2->sts & ADC_PCCE_FLAG));
        }
        // adc_flag_clear(ADC2, ADC_PCCE_FLAG);
        // adc_flag_clear(ADC3, ADC_PCCE_FLAG);
        ADC2->sts = ~ADC_PCCE_FLAG;
        ADC3->sts = ~ADC_PCCE_FLAG;
        // ADC_Shunt_Filter();
        MEASURE_TIME(motor_1->task_times.rt_remaining_task)
        {
            motor_1->GetCurrSense()->UpdateRemainingCurrent(iFOC::RT_LOOP_TS);
        }
        // adc_interrupt_enable(ADC2, ADC_PCCE_INT, TRUE);
        ADC2->ctrl1 |= ADC_PCCE_INT;
    }
}

void tmr2_irq(void)
{
    tmr_flag_clear(TMR2, TMR_OVF_FLAG);
    wdt_counter_reload();
    ADC_Filter();
    MEASURE_TIME(motor_1->task_times.mid_interval_task)
    {
        motor_1->DispatchMidTasks(iFOC::MID_LOOP_TS);
    }
}

void drv_fault_irq(void)
{
    if(EXINT->intsts & EXINT_LINE_3)
    {
        exint_flag_clear(EXINT_LINE_3);
    }
}

void usart1_irq(void)
{
    uart1->OnUARTIRQ();
}

void usart1_rx_dma_irq(void)
{
    uart1->OnRxDMAIRQ();
}

void usart1_tx_dma_irq(void)
{
    uart1->OnTxDMAIRQ();
}

void usart3_irq(void)
{
    uart3->OnUARTIRQ();
}

void usart3_rx_dma_irq(void)
{
    uart3->OnRxDMAIRQ();
}

void usart3_tx_dma_irq(void)
{
    uart3->OnTxDMAIRQ();
}

void usbfs_irq(void)
{
    usbd_irq_handler(&usb_core_dev);
}

void can1_rx0_irq(void)
{
    can1->OnIRQ();
}

void hardfault_irq(void)
{
    iFOC::HAL::Bootloader::OnHardFault();
}

void hid_bytes_process(const uint8_t *data, uint16_t len, uint8_t *reply_buf, uint16_t *reply_len)
{
    // usb_protocol->OnRxPacket(data, len, reply_buf, reply_len);
}

#if defined(configUSE_MALLOC_FAILED_HOOK)
extern "C"
{
    void vApplicationMallocFailedHook()
    {
        motor_1->DisarmWithError(iFOC::MotorError::SYSTEM_MEM_ALLOCATION_FAILED);
    }
}
#endif

#if defined(configCHECK_FOR_STACK_OVERFLOW)
extern "C"
{
    void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
    {
        motor_1->DisarmWithError(iFOC::MotorError::SYSTEM_OS_TASK_STACK_OVERFLOW);
    }
}
#endif