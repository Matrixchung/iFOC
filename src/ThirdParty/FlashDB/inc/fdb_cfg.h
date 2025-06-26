/*
 * Copyright (c) 2020, Armink, <armink.ztl@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief configuration template file. You need to rename the file to "fbd_cfg.h" and modify the configuration items in it to suit your use.
 */

#ifndef _FDB_CFG_H_
#define _FDB_CFG_H_

#include "hal_const.h"
#include "fal_cfg.h"

/* using KVDB feature */
#define FDB_USING_KVDB

#ifdef FDB_USING_KVDB
/* Auto update KV to latest default when current KVDB version number is changed. @see fdb_kvdb.ver_num */
/* #define FDB_KV_AUTO_UPDATE */
#endif

/* using TSDB (Time series database) feature */
//#define FDB_USING_TSDB

/* Using FAL storage mode */
#define FDB_USING_FAL_MODE

#ifdef FDB_USING_FAL_MODE
/* the flash write granularity, unit: bit
 * only support 1(nor flash)/ 8(stm32f2/f4)/ 32(stm32f1)/ 64(stm32f7)/ 128(stm32h5) */
#define FDB_WRITE_GRAN        (FLASH_WRITE_GRAN_BITS)        /* @note you must define it for a value */
#endif

/* Using file storage mode by LIBC file API, like fopen/fread/fwrte/fclose */
/* #define FDB_USING_FILE_LIBC_MODE */

/* Using file storage mode by POSIX file API, like open/read/write/close */
/* #define FDB_USING_FILE_POSIX_MODE */

/* MCU Endian Configuration, default is Little Endian Order. */
/* #define FDB_BIG_ENDIAN */

#define FDB_SILENT

/* print debug information */
//#define FDB_DEBUG_ENABLE

#ifndef FDB_SILENT
/* log print macro. default EF_PRINT macro is printf() */
#define FDB_PRINT(...)              fdb_debug_print(__VA_ARGS__)
#else
#ifdef FDB_DEBUG_ENABLE
#undef FDB_DEBUG_ENABLE
#endif
#endif

#define FDB_REUSE_CRC16


#endif /* _FDB_CFG_H_ */
