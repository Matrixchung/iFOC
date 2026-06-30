#include "adc_handler.h"

/*
 * ADC input clock: synchronous clock div by 4, 170/4 = 42.5MHz
 * fSamp = fADC / (Tchannels + 12.5), Tsamp = 1 / fSamp
 *
 * ADC1: IN1(SENSE_C), IN2(SENSE_B), IN3(SENSE_A), IN4(SENSE_BUS), IN11(NTC3), IN12(NTC1)
 *       Regular channels: IN11, IN12, IN4, Tmcu (640.5 cycles, with 4x oversampling)
 *       Injected channels: IN1, IN2, IN3 (24.5 cycles, total 73.5 cycles): Ic, Ib, Ia, Tsamp = 2.02us
 *
 * ADC2: IN3(HALL_A), IN4(HALL_B), IN5(POT), IN6(V_A), IN7(V_B), IN8(V_C), IN11(V_BUS)
 *       Regular channels: IN6, IN7, IN8 (47.5 cycles), IN11, IN5 (92.5 cycles)
 *       Injected channels: IN3, IN4 (24.5 cycles, total 49 cycles): HALL_A/B
 *
 * ADC3: IN12(HALL_C)
 *       Regular channels: Vrefint (640.5 cycles, with 4x oversampling)
 *       Injected channels: IN12 (47.5 cycles): HALL_C
 *
 * ADC1/2/3 injected conversions are all triggered by TIM1 TRGO event (possibly OC4REF in the future, for shifting phase)
 */

volatile uint16_t ADC1_Value[4]{};
volatile uint16_t ADC2_Value[5]{};
volatile uint16_t ADC3_Value = 0;

void ADC_Start_DMA()
{
    HAL_Delay(50);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_Value, 4);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_Value, 5);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC3_Value, 1);
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_RDY | ADC_IT_EOSMP | ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR | ADC_IT_JEOC | ADC_IT_JEOS | ADC_IT_AWD1 | ADC_IT_AWD2 | ADC_IT_AWD3 | ADC_IT_JQOVF);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_RDY | ADC_IT_EOSMP | ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR | ADC_IT_JEOC | ADC_IT_JEOS | ADC_IT_AWD1 | ADC_IT_AWD2 | ADC_IT_AWD3 | ADC_IT_JQOVF);
    __HAL_ADC_DISABLE_IT(&hadc3, ADC_IT_RDY | ADC_IT_EOSMP | ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR | ADC_IT_JEOC | ADC_IT_JEOS | ADC_IT_AWD1 | ADC_IT_AWD2 | ADC_IT_AWD3 | ADC_IT_JQOVF);
    HAL_Delay(10);
}

void ADC_Start_Injected()
{
    /* Case of multimode enabled when multimode feature is available:
     * HAL_ADCEx_InjectedStart() API must be called for ADC slave first, then for ADC master.
     **/
    /* If the durations of the master injected sequence and the slave injected sequence are equal,
     * it is possible for the software to enable only one of the two JEOS interrupts (for example, the
     * master JEOS), and to read both converted data results from the master ADC_JDRx and
     * slave ADC_JDRx registers. -- AN4195
     **/
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart(&hadc3);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}