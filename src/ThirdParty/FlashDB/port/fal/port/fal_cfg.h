/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include "fal_def.h"
#include "hal_const.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FAL_PART_HAS_TABLE_CFG

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev onchip_flash;

void fdb_debug_print(const char *format, ...);

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &onchip_flash,                                                   \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                               \
{                                                                                    \
    {FAL_PART_MAGIC_WORD, "s", "o", 0x00, (FLASH_USER_AREA_SIZE), 0}, \
}
#endif /* FAL_PART_HAS_TABLE_CFG */

#ifdef __cplusplus
}
#endif

#endif /* _FAL_CFG_H_ */
