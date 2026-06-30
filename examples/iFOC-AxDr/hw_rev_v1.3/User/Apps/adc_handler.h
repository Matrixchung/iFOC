#ifndef IFOC_AXDR_ADC_HANDLER_H
#define IFOC_AXDR_ADC_HANDLER_H

#include "global_include.h"

#define NTC_IN_2_RAW ADC1_Value[0]
#define NTC_IN_1_RAW ADC1_Value[1]
#define I_BUS_RAW    ADC1_Value[2]
#define MCU_TEMP_RAW ADC1_Value[3]
#define V_A_RAW      ADC2_Value[0]
#define V_B_RAW      ADC2_Value[1]
#define V_C_RAW      ADC2_Value[2]
#define V_BUS_RAW    ADC2_Value[3]
#define V_POT_RAW    ADC2_Value[4]
#define VREFINT_RAW  ADC3_Value

extern volatile uint16_t ADC1_Value[4];
extern volatile uint16_t ADC2_Value[5];
extern volatile uint16_t ADC3_Value;

void ADC_Start_DMA();
void ADC_Start_Injected();

#endif //IFOC_AXDR_ADC_HANDLER_H