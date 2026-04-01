#pragma once

#ifdef GD32_ENV

#include "../hal_const.h"

#ifdef GD32G5X3 // GD32G553CEU7, 256K Flash (1/2Bank), 128K RAM
#include "gd32g5x3_fmc.h"

#define ADDR_FLASH_PAGE_0   ((uint32_t)0x8000000) /* Base @ of Page 0  , 2 Kbytes */
#define ADDR_FLASH_PAGE_1   ((uint32_t)0x8000800) /* Base @ of Page 1  , 2 Kbytes */
#define ADDR_FLASH_PAGE_2   ((uint32_t)0x8001000) /* Base @ of Page 2  , 2 Kbytes */
#define ADDR_FLASH_PAGE_3   ((uint32_t)0x8001800) /* Base @ of Page 3  , 2 Kbytes */
#define ADDR_FLASH_PAGE_4   ((uint32_t)0x8002000) /* Base @ of Page 4  , 2 Kbytes */
#define ADDR_FLASH_PAGE_5   ((uint32_t)0x8002800) /* Base @ of Page 5  , 2 Kbytes */
#define ADDR_FLASH_PAGE_6   ((uint32_t)0x8003000) /* Base @ of Page 6  , 2 Kbytes */
#define ADDR_FLASH_PAGE_7   ((uint32_t)0x8003800) /* Base @ of Page 7  , 2 Kbytes */
#define ADDR_FLASH_PAGE_8   ((uint32_t)0x8004000) /* Base @ of Page 8  , 2 Kbytes */
#define ADDR_FLASH_PAGE_9   ((uint32_t)0x8004800) /* Base @ of Page 9  , 2 Kbytes */
#define ADDR_FLASH_PAGE_10  ((uint32_t)0x8005000) /* Base @ of Page 10 , 2 Kbytes */
#define ADDR_FLASH_PAGE_11  ((uint32_t)0x8005800) /* Base @ of Page 11 , 2 Kbytes */
#define ADDR_FLASH_PAGE_12  ((uint32_t)0x8006000) /* Base @ of Page 12 , 2 Kbytes */
#define ADDR_FLASH_PAGE_13  ((uint32_t)0x8006800) /* Base @ of Page 13 , 2 Kbytes */
#define ADDR_FLASH_PAGE_14  ((uint32_t)0x8007000) /* Base @ of Page 14 , 2 Kbytes */
#define ADDR_FLASH_PAGE_15  ((uint32_t)0x8007800) /* Base @ of Page 15 , 2 Kbytes */
#define ADDR_FLASH_PAGE_16  ((uint32_t)0x8008000) /* Base @ of Page 16 , 2 Kbytes */
#define ADDR_FLASH_PAGE_17  ((uint32_t)0x8008800) /* Base @ of Page 17 , 2 Kbytes */
#define ADDR_FLASH_PAGE_18  ((uint32_t)0x8009000) /* Base @ of Page 18 , 2 Kbytes */
#define ADDR_FLASH_PAGE_19  ((uint32_t)0x8009800) /* Base @ of Page 19 , 2 Kbytes */
#define ADDR_FLASH_PAGE_20  ((uint32_t)0x800a000) /* Base @ of Page 20 , 2 Kbytes */
#define ADDR_FLASH_PAGE_21  ((uint32_t)0x800a800) /* Base @ of Page 21 , 2 Kbytes */
#define ADDR_FLASH_PAGE_22  ((uint32_t)0x800b000) /* Base @ of Page 22 , 2 Kbytes */
#define ADDR_FLASH_PAGE_23  ((uint32_t)0x800b800) /* Base @ of Page 23 , 2 Kbytes */
#define ADDR_FLASH_PAGE_24  ((uint32_t)0x800c000) /* Base @ of Page 24 , 2 Kbytes */
#define ADDR_FLASH_PAGE_25  ((uint32_t)0x800c800) /* Base @ of Page 25 , 2 Kbytes */
#define ADDR_FLASH_PAGE_26  ((uint32_t)0x800d000) /* Base @ of Page 26 , 2 Kbytes */
#define ADDR_FLASH_PAGE_27  ((uint32_t)0x800d800) /* Base @ of Page 27 , 2 Kbytes */
#define ADDR_FLASH_PAGE_28  ((uint32_t)0x800e000) /* Base @ of Page 28 , 2 Kbytes */
#define ADDR_FLASH_PAGE_29  ((uint32_t)0x800e800) /* Base @ of Page 29 , 2 Kbytes */
#define ADDR_FLASH_PAGE_30  ((uint32_t)0x800f000) /* Base @ of Page 30 , 2 Kbytes */
#define ADDR_FLASH_PAGE_31  ((uint32_t)0x800f800) /* Base @ of Page 31 , 2 Kbytes */
#define ADDR_FLASH_PAGE_32  ((uint32_t)0x8010000) /* Base @ of Page 32 , 2 Kbytes */
#define ADDR_FLASH_PAGE_33  ((uint32_t)0x8010800) /* Base @ of Page 33 , 2 Kbytes */
#define ADDR_FLASH_PAGE_34  ((uint32_t)0x8011000) /* Base @ of Page 34 , 2 Kbytes */
#define ADDR_FLASH_PAGE_35  ((uint32_t)0x8011800) /* Base @ of Page 35 , 2 Kbytes */
#define ADDR_FLASH_PAGE_36  ((uint32_t)0x8012000) /* Base @ of Page 36 , 2 Kbytes */
#define ADDR_FLASH_PAGE_37  ((uint32_t)0x8012800) /* Base @ of Page 37 , 2 Kbytes */
#define ADDR_FLASH_PAGE_38  ((uint32_t)0x8013000) /* Base @ of Page 38 , 2 Kbytes */
#define ADDR_FLASH_PAGE_39  ((uint32_t)0x8013800) /* Base @ of Page 39 , 2 Kbytes */
#define ADDR_FLASH_PAGE_40  ((uint32_t)0x8014000) /* Base @ of Page 40 , 2 Kbytes */
#define ADDR_FLASH_PAGE_41  ((uint32_t)0x8014800) /* Base @ of Page 41 , 2 Kbytes */
#define ADDR_FLASH_PAGE_42  ((uint32_t)0x8015000) /* Base @ of Page 42 , 2 Kbytes */
#define ADDR_FLASH_PAGE_43  ((uint32_t)0x8015800) /* Base @ of Page 43 , 2 Kbytes */
#define ADDR_FLASH_PAGE_44  ((uint32_t)0x8016000) /* Base @ of Page 44 , 2 Kbytes */
#define ADDR_FLASH_PAGE_45  ((uint32_t)0x8016800) /* Base @ of Page 45 , 2 Kbytes */
#define ADDR_FLASH_PAGE_46  ((uint32_t)0x8017000) /* Base @ of Page 46 , 2 Kbytes */
#define ADDR_FLASH_PAGE_47  ((uint32_t)0x8017800) /* Base @ of Page 47 , 2 Kbytes */
#define ADDR_FLASH_PAGE_48  ((uint32_t)0x8018000) /* Base @ of Page 48 , 2 Kbytes */
#define ADDR_FLASH_PAGE_49  ((uint32_t)0x8018800) /* Base @ of Page 49 , 2 Kbytes */
#define ADDR_FLASH_PAGE_50  ((uint32_t)0x8019000) /* Base @ of Page 50 , 2 Kbytes */
#define ADDR_FLASH_PAGE_51  ((uint32_t)0x8019800) /* Base @ of Page 51 , 2 Kbytes */
#define ADDR_FLASH_PAGE_52  ((uint32_t)0x801a000) /* Base @ of Page 52 , 2 Kbytes */
#define ADDR_FLASH_PAGE_53  ((uint32_t)0x801a800) /* Base @ of Page 53 , 2 Kbytes */
#define ADDR_FLASH_PAGE_54  ((uint32_t)0x801b000) /* Base @ of Page 54 , 2 Kbytes */
#define ADDR_FLASH_PAGE_55  ((uint32_t)0x801b800) /* Base @ of Page 55 , 2 Kbytes */
#define ADDR_FLASH_PAGE_56  ((uint32_t)0x801c000) /* Base @ of Page 56 , 2 Kbytes */
#define ADDR_FLASH_PAGE_57  ((uint32_t)0x801c800) /* Base @ of Page 57 , 2 Kbytes */
#define ADDR_FLASH_PAGE_58  ((uint32_t)0x801d000) /* Base @ of Page 58 , 2 Kbytes */
#define ADDR_FLASH_PAGE_59  ((uint32_t)0x801d800) /* Base @ of Page 59 , 2 Kbytes */
#define ADDR_FLASH_PAGE_60  ((uint32_t)0x801e000) /* Base @ of Page 60 , 2 Kbytes */
#define ADDR_FLASH_PAGE_61  ((uint32_t)0x801e800) /* Base @ of Page 61 , 2 Kbytes */
#define ADDR_FLASH_PAGE_62  ((uint32_t)0x801f000) /* Base @ of Page 62 , 2 Kbytes */
#define ADDR_FLASH_PAGE_63  ((uint32_t)0x801f800) /* Base @ of Page 63 , 2 Kbytes */

#define FLASH_WRITE_GRAN_BITS   (64) // DOUBLE WORD
#define FLASH_SECTOR_SIZE_BYTES (0x800U)
#define FLASH_USER_START_ADDR   (ADDR_FLASH_PAGE_56)
#define FLASH_BL_START_ADDR     (ADDR_FLASH_PAGE_0)
#define FLASH_APP_START_ADDR    (ADDR_FLASH_PAGE_16) // 32K Bootloader
#define FLASH_USER_AREA_SIZE    (8 * FLASH_SECTOR_SIZE_BYTES)
#define USE_FLASHDB

#endif

#endif