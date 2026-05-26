#include "adc_handler.h"
#include "wk_dma.h"
#include "sliding_filter.hpp"

uint16_t ADC1_Value[4];
uint16_t VREFINT_FILTERED = 0;
uint16_t MCU_TEMP_FILTERED = 0;
uint16_t NTC_IN_1_FILTERED = 0;
uint16_t NTC_IN_2_FILTERED = 0;

uint16_t SHUNT_A_FILTERED = 0;
uint16_t SHUNT_B_FILTERED = 0;

iFOC::Filter::SlidingFilter vrefint_filter(32);
iFOC::Filter::SlidingFilter mcu_temp_filter(8);
iFOC::Filter::SlidingFilter ntc_in_1_filter(8);
iFOC::Filter::SlidingFilter ntc_in_2_filter(8);

// iFOC::Filter::SlidingFilter shunt_a_filter(4);
// iFOC::Filter::SlidingFilter shunt_b_filter(4);

extern "C"
{
void ADC_Start()
{
    // ADC1, Regular Conversions
    wk_adc1_init();
    wk_dma1_channel7_init(); // ADC1 DMA CH
    wk_dma_channel_config(DMA1_CHANNEL7, (uint32_t)&ADC1->odt, (uint32_t)&ADC1_Value[0], 4); // fixed buffer size causing DMA overlap
    adc_interrupt_enable(ADC1, ADC_VMOR_INT | ADC_CCE_INT | ADC_PCCE_INT, FALSE);
    dma_channel_enable(DMA1_CHANNEL7, TRUE);
    adc_ordinary_software_trigger_enable(ADC1, TRUE); // start ADC1

    // ADC2 & 3, Injected Conversions
    wk_adc2_init();
    wk_adc3_init();
    adc_interrupt_enable(ADC2, ADC_VMOR_INT | ADC_CCE_INT, FALSE);
    adc_interrupt_enable(ADC3, ADC_VMOR_INT | ADC_CCE_INT | ADC_PCCE_INT, FALSE);
    adc_interrupt_enable(ADC2, ADC_PCCE_INT, TRUE);
}

// void ADC_Shunt_Filter()
// {
//     SHUNT_A_FILTERED = (uint16_t)shunt_a_filter.GetOutput((real_t)(ADC2->pdt1));
//     SHUNT_B_FILTERED = (uint16_t)shunt_b_filter.GetOutput((real_t)(ADC3->pdt1));
// }

void ADC_Filter()
{
    VREFINT_FILTERED = (uint16_t)vrefint_filter.GetOutput((real_t)VREFINT_RAW);
    MCU_TEMP_FILTERED = (uint16_t)mcu_temp_filter.GetOutput((real_t)MCU_TEMP_RAW);
    NTC_IN_1_FILTERED = (uint16_t)ntc_in_1_filter.GetOutput((real_t)NTC_IN_1_RAW);
    NTC_IN_2_FILTERED = (uint16_t)ntc_in_2_filter.GetOutput((real_t)NTC_IN_2_RAW);
}
}