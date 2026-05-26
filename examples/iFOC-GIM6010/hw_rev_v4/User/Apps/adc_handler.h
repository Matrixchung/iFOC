//
// Created by vip99 on 9/11/2025.
//

#ifndef IFOC_GIM6010_V3_ADC_HANDLER_H
#define IFOC_GIM6010_V3_ADC_HANDLER_H

#include "wk_adc.h"

#define MCU_TEMP_RAW ADC1_Value[0]
#define VREFINT_RAW ADC1_Value[1]
#define NTC_IN_1_RAW ADC1_Value[3]
#define NTC_IN_2_RAW ADC1_Value[2]

extern uint16_t ADC1_Value[4];
extern uint16_t VREFINT_FILTERED;
extern uint16_t MCU_TEMP_FILTERED;
extern uint16_t NTC_IN_1_FILTERED;
extern uint16_t NTC_IN_2_FILTERED;

// extern volatile uint16_t SHUNT_A_FILTERED;
// extern volatile uint16_t SHUNT_B_FILTERED;

extern "C"
{
    void ADC_Start();
    // void ADC_Shunt_Filter();
    void ADC_Filter();
}


#endif //IFOC_GIM6010_V3_ADC_HANDLER_H