#ifndef _FOC_PORT_STM32_SYNC_TIMER_H
#define _FOC_PORT_STM32_SYNC_TIMER_H

#include "global_include.h"
#include "array"

#pragma GCC push_options
#pragma GCC optimize (3)

template<size_t T, size_t ... Ts>
void _SyncStartTimer_impl(std::array<TIM_TypeDef*, T> timers, std::index_sequence<Ts...>)
{
    for(size_t i = 0; i < T; i++)
    {
        LL_TIM_DisableCounter(timers[i]);
        LL_TIM_DisableIT_UPDATE(timers[i]);
        LL_TIM_GenerateEvent_UPDATE(timers[i]);
        LL_TIM_ClearFlag_UPDATE(timers[i]);
    }
    volatile uint32_t* pReg[T];
    volatile uint32_t reg[T];
    
    // Synchronously initialize TIMs' CNTs.
    for(size_t i = 0; i < T; i++)
    {
        pReg[i] = &timers[i]->CR1;
        reg[i] = timers[i]->CR1 | TIM_CR1_CEN;
    }
    __disable_irq();
    int dummy[T] = {(*pReg[Ts] = reg[Ts], 0)...};
    (void)dummy;
    __enable_irq();

    // Synchronously initialize TIMs' main outputs(MOE)
    for(size_t i = 0; i < T; i++)
    {
        pReg[i] = &timers[i]->BDTR;
        reg[i] = timers[i]->BDTR | TIM_BDTR_MOE;
    }
    __disable_irq();
    int dummy2[T] = {(*pReg[Ts] = reg[Ts], 0)...};
    (void)dummy2;
    __enable_irq();
}

template<typename ... T>
void SyncStartTimer(T&... inst)
{
    _SyncStartTimer_impl(std::array<TIM_TypeDef*, sizeof...(T)>{inst.driver.htim...},  std::make_index_sequence<sizeof...(T)>());
}


#pragma GCC pop_options

#endif

// #pragma GCC push_options
// #pragma GCC optimize (3)
// void SyncStartTimer()
// {
//     LL_TIM_DisableCounter(motor_1.driver.htim);
//     LL_TIM_DisableIT_UPDATE(motor_1.driver.htim);
//     LL_TIM_GenerateEvent_UPDATE(motor_1.driver.htim);
//     LL_TIM_ClearFlag_UPDATE(motor_1.driver.htim);

//     LL_TIM_DisableCounter(motor_2.driver.htim);
//     LL_TIM_DisableIT_UPDATE(motor_2.driver.htim);
//     LL_TIM_GenerateEvent_UPDATE(motor_2.driver.htim);
//     LL_TIM_ClearFlag_UPDATE(motor_2.driver.htim);

//     volatile uint32_t* cr_addr[2];
//     volatile uint32_t cr_value[2];
//     cr_addr[0] = &motor_1.driver.htim->CR1;
//     cr_value[0] = motor_1.driver.htim->CR1 | TIM_CR1_CEN;
//     cr_addr[1] = &motor_2.driver.htim->CR1;
//     cr_value[1] = motor_2.driver.htim->CR1 | TIM_CR1_CEN;

//     __disable_irq();
//     *cr_addr[0] = cr_value[0];
//     *cr_addr[1] = cr_value[1];
//     LL_TIM_EnableAllOutputs(motor_1.driver.htim);
//     LL_TIM_EnableAllOutputs(motor_2.driver.htim);
//     __enable_irq();
// }
// #pragma GCC pop_options