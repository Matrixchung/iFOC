#include "adc_handler.h"
uint16_t adc_ret_val[5];
// 0 - SENSE_BAT
// 1 - SENSE_1
// 2 - SENSE_3
// 3 - SENSE_VRefint
// 4 - SENSE_Temp

// Using the information in the Adjusting the Output Midpoint With the Reference Pins section, the reference point is
// set to mid-scale by splitting the supply with REF1 connected to ground and REF2 connected to supply. This
// configuration allows for bipolar current measurements.

ADC_HandleTypeDef *gHadc;
void ADC_HandlerInit(ADC_HandleTypeDef *hadc)
{
    gHadc = hadc;
    HAL_ADCEx_Calibration_Start(gHadc);
    HAL_ADC_Start_DMA(gHadc, (uint32_t*)adc_ret_val, 5);
}

// #define ADC_GetRawValue(channel) (adc_ret_val[channel])
// uint16_t ADC_GetRawValue(uint8_t channel)
// {
//     return adc_ret_val[channel];
// }

// #define ADC_GetCalibratedValue(channel) ((float)(1.2f*(float)(adc_ret_val[channel])/(float)(adc_ret_val[3])))
// float ADC_GetCalibratedValue(uint8_t channel)
// {
//     // STM32F103CBT6 Vrefint = 1.20 V (see datasheet)
//     return (float)(1.2f*(float)(adc_ret_val[channel])/(float)(adc_ret_val[3]));
// }

// #define ADC_GetVBAT() ((float)(11.0f*ADC_GetCalibratedValue(0)))
// float ADC_GetVBAT()
// {
//     // 100K Ohm - 10K Ohm voltage divider
//     return (float)(11.0f*ADC_GetCalibratedValue(0));
// }