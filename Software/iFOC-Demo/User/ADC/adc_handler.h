#ifndef _ADC_HANDLER_H
#define _ADC_HANDLER_H

#include "main.h"
#include "adc.h"

extern uint16_t adc_ret_val[5];
void ADC_HandlerInit(ADC_HandleTypeDef *hadc);
// uint16_t ADC_GetRawValue(uint8_t channel);
// float ADC_GetCalibratedValue(uint8_t channel);
// float ADC_GetVBAT();
#define ADC_GetRawValue(channel) (adc_ret_val[channel])
#define ADC_GetCalibratedValue(channel) ((float)(1.2f*(float)(adc_ret_val[channel])/(float)(adc_ret_val[3])))
#define ADC_GetVBAT() ((float)(11.0f*ADC_GetCalibratedValue(0)))

// VDD = 4095 / Vrefint * Vrefint_cal (4914 = 4095 * 1.2)
#define ADC_GetVDD() ((float)(4914.0f/(float)adc_ret_val[3]))
#define ADC_GetMidRefVoltage(channel) ((float)(ADC_GetCalibratedValue(channel) - (float) (0.5f * ADC_GetVDD())))

// I = U / R
#define ADC_GetCurrent(channel, gain, r) ((float)(ADC_GetMidRefVoltage(channel) / (r * gain)))

// Temp = {(V25-Vsense) / Avg_Slope} + 25
// V25 = 1.43V, Avg_Slope = 4.3mV/C
#define ADC_GetTemperature() ((float)((1.43f - ADC_GetCalibratedValue(4)) / 0.0043f) + 25.0f)
#endif