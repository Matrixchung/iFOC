#pragma once

#include "main.h"

namespace iFOC
{
#if defined(STM32G4)
static void EnablePVD()
{
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR_PVDTypeDef sConfigPVD{
        .PVDLevel = PWR_PVDLEVEL_6,
        .Mode = PWR_PVD_MODE_IT_RISING
    };
    // MAXIMUM PRIORITY
    HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
    HAL_PWR_ConfigPVD(&sConfigPVD);
    HAL_PWR_EnablePVD();
}
#endif
}