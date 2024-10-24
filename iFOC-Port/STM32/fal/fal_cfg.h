#include "fal_def.h"
#if MCU == F7
#ifndef _FAL_CFG_STM32F7_H_
#define _FAL_CFG_STM32F7_H_

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev stm32f7_onchip_flash;
#define FAL_PART_HAS_TABLE_CFG
/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &stm32f7_onchip_flash,                                           \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                               \
{                                                                                    \
    {FAL_PART_MAGIC_WORD,       "app",     "stm32_onchip",   0,  128*1024, 0},       \
    {FAL_PART_MAGIC_WORD,  "nvm_storage",  "stm32_onchip",   128*1024,  256*1024, 0},       \
}
#endif
#endif
#endif