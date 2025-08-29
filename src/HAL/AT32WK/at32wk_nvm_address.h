#pragma once

#include "wk_gpio.h"

#if defined(AT32F403AxG) // 1024K, Double Bank (User Data uses Bank2)
#define ADDR_FLASH_SECTOR_0   ((uint32_t)0x8000000) /* Base @ of Sector 0  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_1   ((uint32_t)0x8000800) /* Base @ of Sector 1  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_2   ((uint32_t)0x8001000) /* Base @ of Sector 2  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_3   ((uint32_t)0x8001800) /* Base @ of Sector 3  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_4   ((uint32_t)0x8002000) /* Base @ of Sector 4  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_5   ((uint32_t)0x8002800) /* Base @ of Sector 5  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_6   ((uint32_t)0x8003000) /* Base @ of Sector 6  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_7   ((uint32_t)0x8003800) /* Base @ of Sector 7  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_8   ((uint32_t)0x8004000) /* Base @ of Sector 8  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_9   ((uint32_t)0x8004800) /* Base @ of Sector 9  , 2 Kbytes */
#define ADDR_FLASH_SECTOR_10  ((uint32_t)0x8005000) /* Base @ of Sector 10 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_11  ((uint32_t)0x8005800) /* Base @ of Sector 11 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_12  ((uint32_t)0x8006000) /* Base @ of Sector 12 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_13  ((uint32_t)0x8006800) /* Base @ of Sector 13 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_14  ((uint32_t)0x8007000) /* Base @ of Sector 14 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_15  ((uint32_t)0x8007800) /* Base @ of Sector 15 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_16  ((uint32_t)0x8008000) /* Base @ of Sector 16 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_17  ((uint32_t)0x8008800) /* Base @ of Sector 17 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_18  ((uint32_t)0x8009000) /* Base @ of Sector 18 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_19  ((uint32_t)0x8009800) /* Base @ of Sector 19 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_20  ((uint32_t)0x800a000) /* Base @ of Sector 20 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_21  ((uint32_t)0x800a800) /* Base @ of Sector 21 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_22  ((uint32_t)0x800b000) /* Base @ of Sector 22 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_23  ((uint32_t)0x800b800) /* Base @ of Sector 23 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_24  ((uint32_t)0x800c000) /* Base @ of Sector 24 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_25  ((uint32_t)0x800c800) /* Base @ of Sector 25 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_26  ((uint32_t)0x800d000) /* Base @ of Sector 26 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_27  ((uint32_t)0x800d800) /* Base @ of Sector 27 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_28  ((uint32_t)0x800e000) /* Base @ of Sector 28 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_29  ((uint32_t)0x800e800) /* Base @ of Sector 29 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_30  ((uint32_t)0x800f000) /* Base @ of Sector 30 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_31  ((uint32_t)0x800f800) /* Base @ of Sector 31 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_32  ((uint32_t)0x8010000) /* Base @ of Sector 32 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_33  ((uint32_t)0x8010800) /* Base @ of Sector 33 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_34  ((uint32_t)0x8011000) /* Base @ of Sector 34 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_35  ((uint32_t)0x8011800) /* Base @ of Sector 35 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_36  ((uint32_t)0x8012000) /* Base @ of Sector 36 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_37  ((uint32_t)0x8012800) /* Base @ of Sector 37 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_38  ((uint32_t)0x8013000) /* Base @ of Sector 38 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_39  ((uint32_t)0x8013800) /* Base @ of Sector 39 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_40  ((uint32_t)0x8014000) /* Base @ of Sector 40 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_41  ((uint32_t)0x8014800) /* Base @ of Sector 41 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_42  ((uint32_t)0x8015000) /* Base @ of Sector 42 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_43  ((uint32_t)0x8015800) /* Base @ of Sector 43 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_44  ((uint32_t)0x8016000) /* Base @ of Sector 44 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_45  ((uint32_t)0x8016800) /* Base @ of Sector 45 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_46  ((uint32_t)0x8017000) /* Base @ of Sector 46 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_47  ((uint32_t)0x8017800) /* Base @ of Sector 47 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_48  ((uint32_t)0x8018000) /* Base @ of Sector 48 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_49  ((uint32_t)0x8018800) /* Base @ of Sector 49 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_50  ((uint32_t)0x8019000) /* Base @ of Sector 50 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_51  ((uint32_t)0x8019800) /* Base @ of Sector 51 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_52  ((uint32_t)0x801a000) /* Base @ of Sector 52 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_53  ((uint32_t)0x801a800) /* Base @ of Sector 53 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_54  ((uint32_t)0x801b000) /* Base @ of Sector 54 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_55  ((uint32_t)0x801b800) /* Base @ of Sector 55 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_56  ((uint32_t)0x801c000) /* Base @ of Sector 56 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_57  ((uint32_t)0x801c800) /* Base @ of Sector 57 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_58  ((uint32_t)0x801d000) /* Base @ of Sector 58 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_59  ((uint32_t)0x801d800) /* Base @ of Sector 59 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_60  ((uint32_t)0x801e000) /* Base @ of Sector 60 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_61  ((uint32_t)0x801e800) /* Base @ of Sector 61 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_62  ((uint32_t)0x801f000) /* Base @ of Sector 62 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_63  ((uint32_t)0x801f800) /* Base @ of Sector 63 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_64  ((uint32_t)0x8020000) /* Base @ of Sector 64 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_65  ((uint32_t)0x8020800) /* Base @ of Sector 65 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_66  ((uint32_t)0x8021000) /* Base @ of Sector 66 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_67  ((uint32_t)0x8021800) /* Base @ of Sector 67 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_68  ((uint32_t)0x8022000) /* Base @ of Sector 68 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_69  ((uint32_t)0x8022800) /* Base @ of Sector 69 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_70  ((uint32_t)0x8023000) /* Base @ of Sector 70 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_71  ((uint32_t)0x8023800) /* Base @ of Sector 71 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_72  ((uint32_t)0x8024000) /* Base @ of Sector 72 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_73  ((uint32_t)0x8024800) /* Base @ of Sector 73 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_74  ((uint32_t)0x8025000) /* Base @ of Sector 74 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_75  ((uint32_t)0x8025800) /* Base @ of Sector 75 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_76  ((uint32_t)0x8026000) /* Base @ of Sector 76 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_77  ((uint32_t)0x8026800) /* Base @ of Sector 77 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_78  ((uint32_t)0x8027000) /* Base @ of Sector 78 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_79  ((uint32_t)0x8027800) /* Base @ of Sector 79 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_80  ((uint32_t)0x8028000) /* Base @ of Sector 80 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_81  ((uint32_t)0x8028800) /* Base @ of Sector 81 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_82  ((uint32_t)0x8029000) /* Base @ of Sector 82 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_83  ((uint32_t)0x8029800) /* Base @ of Sector 83 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_84  ((uint32_t)0x802a000) /* Base @ of Sector 84 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_85  ((uint32_t)0x802a800) /* Base @ of Sector 85 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_86  ((uint32_t)0x802b000) /* Base @ of Sector 86 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_87  ((uint32_t)0x802b800) /* Base @ of Sector 87 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_88  ((uint32_t)0x802c000) /* Base @ of Sector 88 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_89  ((uint32_t)0x802c800) /* Base @ of Sector 89 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_90  ((uint32_t)0x802d000) /* Base @ of Sector 90 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_91  ((uint32_t)0x802d800) /* Base @ of Sector 91 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_92  ((uint32_t)0x802e000) /* Base @ of Sector 92 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_93  ((uint32_t)0x802e800) /* Base @ of Sector 93 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_94  ((uint32_t)0x802f000) /* Base @ of Sector 94 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_95  ((uint32_t)0x802f800) /* Base @ of Sector 95 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_96  ((uint32_t)0x8030000) /* Base @ of Sector 96 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_97  ((uint32_t)0x8030800) /* Base @ of Sector 97 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_98  ((uint32_t)0x8031000) /* Base @ of Sector 98 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_99  ((uint32_t)0x8031800) /* Base @ of Sector 99 , 2 Kbytes */
#define ADDR_FLASH_SECTOR_100 ((uint32_t)0x8032000) /* Base @ of Sector 100, 2 Kbytes */
#define ADDR_FLASH_SECTOR_101 ((uint32_t)0x8032800) /* Base @ of Sector 101, 2 Kbytes */
#define ADDR_FLASH_SECTOR_102 ((uint32_t)0x8033000) /* Base @ of Sector 102, 2 Kbytes */
#define ADDR_FLASH_SECTOR_103 ((uint32_t)0x8033800) /* Base @ of Sector 103, 2 Kbytes */
#define ADDR_FLASH_SECTOR_104 ((uint32_t)0x8034000) /* Base @ of Sector 104, 2 Kbytes */
#define ADDR_FLASH_SECTOR_105 ((uint32_t)0x8034800) /* Base @ of Sector 105, 2 Kbytes */
#define ADDR_FLASH_SECTOR_106 ((uint32_t)0x8035000) /* Base @ of Sector 106, 2 Kbytes */
#define ADDR_FLASH_SECTOR_107 ((uint32_t)0x8035800) /* Base @ of Sector 107, 2 Kbytes */
#define ADDR_FLASH_SECTOR_108 ((uint32_t)0x8036000) /* Base @ of Sector 108, 2 Kbytes */
#define ADDR_FLASH_SECTOR_109 ((uint32_t)0x8036800) /* Base @ of Sector 109, 2 Kbytes */
#define ADDR_FLASH_SECTOR_110 ((uint32_t)0x8037000) /* Base @ of Sector 110, 2 Kbytes */
#define ADDR_FLASH_SECTOR_111 ((uint32_t)0x8037800) /* Base @ of Sector 111, 2 Kbytes */
#define ADDR_FLASH_SECTOR_112 ((uint32_t)0x8038000) /* Base @ of Sector 112, 2 Kbytes */
#define ADDR_FLASH_SECTOR_113 ((uint32_t)0x8038800) /* Base @ of Sector 113, 2 Kbytes */
#define ADDR_FLASH_SECTOR_114 ((uint32_t)0x8039000) /* Base @ of Sector 114, 2 Kbytes */
#define ADDR_FLASH_SECTOR_115 ((uint32_t)0x8039800) /* Base @ of Sector 115, 2 Kbytes */
#define ADDR_FLASH_SECTOR_116 ((uint32_t)0x803a000) /* Base @ of Sector 116, 2 Kbytes */
#define ADDR_FLASH_SECTOR_117 ((uint32_t)0x803a800) /* Base @ of Sector 117, 2 Kbytes */
#define ADDR_FLASH_SECTOR_118 ((uint32_t)0x803b000) /* Base @ of Sector 118, 2 Kbytes */
#define ADDR_FLASH_SECTOR_119 ((uint32_t)0x803b800) /* Base @ of Sector 119, 2 Kbytes */
#define ADDR_FLASH_SECTOR_120 ((uint32_t)0x803c000) /* Base @ of Sector 120, 2 Kbytes */
#define ADDR_FLASH_SECTOR_121 ((uint32_t)0x803c800) /* Base @ of Sector 121, 2 Kbytes */
#define ADDR_FLASH_SECTOR_122 ((uint32_t)0x803d000) /* Base @ of Sector 122, 2 Kbytes */
#define ADDR_FLASH_SECTOR_123 ((uint32_t)0x803d800) /* Base @ of Sector 123, 2 Kbytes */
#define ADDR_FLASH_SECTOR_124 ((uint32_t)0x803e000) /* Base @ of Sector 124, 2 Kbytes */
#define ADDR_FLASH_SECTOR_125 ((uint32_t)0x803e800) /* Base @ of Sector 125, 2 Kbytes */
#define ADDR_FLASH_SECTOR_126 ((uint32_t)0x803f000) /* Base @ of Sector 126, 2 Kbytes */
#define ADDR_FLASH_SECTOR_127 ((uint32_t)0x803f800) /* Base @ of Sector 127, 2 Kbytes */
#define ADDR_FLASH_SECTOR_128 ((uint32_t)0x8040000) /* Base @ of Sector 128, 2 Kbytes */
#define ADDR_FLASH_SECTOR_129 ((uint32_t)0x8040800) /* Base @ of Sector 129, 2 Kbytes */
#define ADDR_FLASH_SECTOR_130 ((uint32_t)0x8041000) /* Base @ of Sector 130, 2 Kbytes */
#define ADDR_FLASH_SECTOR_131 ((uint32_t)0x8041800) /* Base @ of Sector 131, 2 Kbytes */
#define ADDR_FLASH_SECTOR_132 ((uint32_t)0x8042000) /* Base @ of Sector 132, 2 Kbytes */
#define ADDR_FLASH_SECTOR_133 ((uint32_t)0x8042800) /* Base @ of Sector 133, 2 Kbytes */
#define ADDR_FLASH_SECTOR_134 ((uint32_t)0x8043000) /* Base @ of Sector 134, 2 Kbytes */
#define ADDR_FLASH_SECTOR_135 ((uint32_t)0x8043800) /* Base @ of Sector 135, 2 Kbytes */
#define ADDR_FLASH_SECTOR_136 ((uint32_t)0x8044000) /* Base @ of Sector 136, 2 Kbytes */
#define ADDR_FLASH_SECTOR_137 ((uint32_t)0x8044800) /* Base @ of Sector 137, 2 Kbytes */
#define ADDR_FLASH_SECTOR_138 ((uint32_t)0x8045000) /* Base @ of Sector 138, 2 Kbytes */
#define ADDR_FLASH_SECTOR_139 ((uint32_t)0x8045800) /* Base @ of Sector 139, 2 Kbytes */
#define ADDR_FLASH_SECTOR_140 ((uint32_t)0x8046000) /* Base @ of Sector 140, 2 Kbytes */
#define ADDR_FLASH_SECTOR_141 ((uint32_t)0x8046800) /* Base @ of Sector 141, 2 Kbytes */
#define ADDR_FLASH_SECTOR_142 ((uint32_t)0x8047000) /* Base @ of Sector 142, 2 Kbytes */
#define ADDR_FLASH_SECTOR_143 ((uint32_t)0x8047800) /* Base @ of Sector 143, 2 Kbytes */
#define ADDR_FLASH_SECTOR_144 ((uint32_t)0x8048000) /* Base @ of Sector 144, 2 Kbytes */
#define ADDR_FLASH_SECTOR_145 ((uint32_t)0x8048800) /* Base @ of Sector 145, 2 Kbytes */
#define ADDR_FLASH_SECTOR_146 ((uint32_t)0x8049000) /* Base @ of Sector 146, 2 Kbytes */
#define ADDR_FLASH_SECTOR_147 ((uint32_t)0x8049800) /* Base @ of Sector 147, 2 Kbytes */
#define ADDR_FLASH_SECTOR_148 ((uint32_t)0x804a000) /* Base @ of Sector 148, 2 Kbytes */
#define ADDR_FLASH_SECTOR_149 ((uint32_t)0x804a800) /* Base @ of Sector 149, 2 Kbytes */
#define ADDR_FLASH_SECTOR_150 ((uint32_t)0x804b000) /* Base @ of Sector 150, 2 Kbytes */
#define ADDR_FLASH_SECTOR_151 ((uint32_t)0x804b800) /* Base @ of Sector 151, 2 Kbytes */
#define ADDR_FLASH_SECTOR_152 ((uint32_t)0x804c000) /* Base @ of Sector 152, 2 Kbytes */
#define ADDR_FLASH_SECTOR_153 ((uint32_t)0x804c800) /* Base @ of Sector 153, 2 Kbytes */
#define ADDR_FLASH_SECTOR_154 ((uint32_t)0x804d000) /* Base @ of Sector 154, 2 Kbytes */
#define ADDR_FLASH_SECTOR_155 ((uint32_t)0x804d800) /* Base @ of Sector 155, 2 Kbytes */
#define ADDR_FLASH_SECTOR_156 ((uint32_t)0x804e000) /* Base @ of Sector 156, 2 Kbytes */
#define ADDR_FLASH_SECTOR_157 ((uint32_t)0x804e800) /* Base @ of Sector 157, 2 Kbytes */
#define ADDR_FLASH_SECTOR_158 ((uint32_t)0x804f000) /* Base @ of Sector 158, 2 Kbytes */
#define ADDR_FLASH_SECTOR_159 ((uint32_t)0x804f800) /* Base @ of Sector 159, 2 Kbytes */
#define ADDR_FLASH_SECTOR_160 ((uint32_t)0x8050000) /* Base @ of Sector 160, 2 Kbytes */
#define ADDR_FLASH_SECTOR_161 ((uint32_t)0x8050800) /* Base @ of Sector 161, 2 Kbytes */
#define ADDR_FLASH_SECTOR_162 ((uint32_t)0x8051000) /* Base @ of Sector 162, 2 Kbytes */
#define ADDR_FLASH_SECTOR_163 ((uint32_t)0x8051800) /* Base @ of Sector 163, 2 Kbytes */
#define ADDR_FLASH_SECTOR_164 ((uint32_t)0x8052000) /* Base @ of Sector 164, 2 Kbytes */
#define ADDR_FLASH_SECTOR_165 ((uint32_t)0x8052800) /* Base @ of Sector 165, 2 Kbytes */
#define ADDR_FLASH_SECTOR_166 ((uint32_t)0x8053000) /* Base @ of Sector 166, 2 Kbytes */
#define ADDR_FLASH_SECTOR_167 ((uint32_t)0x8053800) /* Base @ of Sector 167, 2 Kbytes */
#define ADDR_FLASH_SECTOR_168 ((uint32_t)0x8054000) /* Base @ of Sector 168, 2 Kbytes */
#define ADDR_FLASH_SECTOR_169 ((uint32_t)0x8054800) /* Base @ of Sector 169, 2 Kbytes */
#define ADDR_FLASH_SECTOR_170 ((uint32_t)0x8055000) /* Base @ of Sector 170, 2 Kbytes */
#define ADDR_FLASH_SECTOR_171 ((uint32_t)0x8055800) /* Base @ of Sector 171, 2 Kbytes */
#define ADDR_FLASH_SECTOR_172 ((uint32_t)0x8056000) /* Base @ of Sector 172, 2 Kbytes */
#define ADDR_FLASH_SECTOR_173 ((uint32_t)0x8056800) /* Base @ of Sector 173, 2 Kbytes */
#define ADDR_FLASH_SECTOR_174 ((uint32_t)0x8057000) /* Base @ of Sector 174, 2 Kbytes */
#define ADDR_FLASH_SECTOR_175 ((uint32_t)0x8057800) /* Base @ of Sector 175, 2 Kbytes */
#define ADDR_FLASH_SECTOR_176 ((uint32_t)0x8058000) /* Base @ of Sector 176, 2 Kbytes */
#define ADDR_FLASH_SECTOR_177 ((uint32_t)0x8058800) /* Base @ of Sector 177, 2 Kbytes */
#define ADDR_FLASH_SECTOR_178 ((uint32_t)0x8059000) /* Base @ of Sector 178, 2 Kbytes */
#define ADDR_FLASH_SECTOR_179 ((uint32_t)0x8059800) /* Base @ of Sector 179, 2 Kbytes */
#define ADDR_FLASH_SECTOR_180 ((uint32_t)0x805a000) /* Base @ of Sector 180, 2 Kbytes */
#define ADDR_FLASH_SECTOR_181 ((uint32_t)0x805a800) /* Base @ of Sector 181, 2 Kbytes */
#define ADDR_FLASH_SECTOR_182 ((uint32_t)0x805b000) /* Base @ of Sector 182, 2 Kbytes */
#define ADDR_FLASH_SECTOR_183 ((uint32_t)0x805b800) /* Base @ of Sector 183, 2 Kbytes */
#define ADDR_FLASH_SECTOR_184 ((uint32_t)0x805c000) /* Base @ of Sector 184, 2 Kbytes */
#define ADDR_FLASH_SECTOR_185 ((uint32_t)0x805c800) /* Base @ of Sector 185, 2 Kbytes */
#define ADDR_FLASH_SECTOR_186 ((uint32_t)0x805d000) /* Base @ of Sector 186, 2 Kbytes */
#define ADDR_FLASH_SECTOR_187 ((uint32_t)0x805d800) /* Base @ of Sector 187, 2 Kbytes */
#define ADDR_FLASH_SECTOR_188 ((uint32_t)0x805e000) /* Base @ of Sector 188, 2 Kbytes */
#define ADDR_FLASH_SECTOR_189 ((uint32_t)0x805e800) /* Base @ of Sector 189, 2 Kbytes */
#define ADDR_FLASH_SECTOR_190 ((uint32_t)0x805f000) /* Base @ of Sector 190, 2 Kbytes */
#define ADDR_FLASH_SECTOR_191 ((uint32_t)0x805f800) /* Base @ of Sector 191, 2 Kbytes */
#define ADDR_FLASH_SECTOR_192 ((uint32_t)0x8060000) /* Base @ of Sector 192, 2 Kbytes */
#define ADDR_FLASH_SECTOR_193 ((uint32_t)0x8060800) /* Base @ of Sector 193, 2 Kbytes */
#define ADDR_FLASH_SECTOR_194 ((uint32_t)0x8061000) /* Base @ of Sector 194, 2 Kbytes */
#define ADDR_FLASH_SECTOR_195 ((uint32_t)0x8061800) /* Base @ of Sector 195, 2 Kbytes */
#define ADDR_FLASH_SECTOR_196 ((uint32_t)0x8062000) /* Base @ of Sector 196, 2 Kbytes */
#define ADDR_FLASH_SECTOR_197 ((uint32_t)0x8062800) /* Base @ of Sector 197, 2 Kbytes */
#define ADDR_FLASH_SECTOR_198 ((uint32_t)0x8063000) /* Base @ of Sector 198, 2 Kbytes */
#define ADDR_FLASH_SECTOR_199 ((uint32_t)0x8063800) /* Base @ of Sector 199, 2 Kbytes */
#define ADDR_FLASH_SECTOR_200 ((uint32_t)0x8064000) /* Base @ of Sector 200, 2 Kbytes */
#define ADDR_FLASH_SECTOR_201 ((uint32_t)0x8064800) /* Base @ of Sector 201, 2 Kbytes */
#define ADDR_FLASH_SECTOR_202 ((uint32_t)0x8065000) /* Base @ of Sector 202, 2 Kbytes */
#define ADDR_FLASH_SECTOR_203 ((uint32_t)0x8065800) /* Base @ of Sector 203, 2 Kbytes */
#define ADDR_FLASH_SECTOR_204 ((uint32_t)0x8066000) /* Base @ of Sector 204, 2 Kbytes */
#define ADDR_FLASH_SECTOR_205 ((uint32_t)0x8066800) /* Base @ of Sector 205, 2 Kbytes */
#define ADDR_FLASH_SECTOR_206 ((uint32_t)0x8067000) /* Base @ of Sector 206, 2 Kbytes */
#define ADDR_FLASH_SECTOR_207 ((uint32_t)0x8067800) /* Base @ of Sector 207, 2 Kbytes */
#define ADDR_FLASH_SECTOR_208 ((uint32_t)0x8068000) /* Base @ of Sector 208, 2 Kbytes */
#define ADDR_FLASH_SECTOR_209 ((uint32_t)0x8068800) /* Base @ of Sector 209, 2 Kbytes */
#define ADDR_FLASH_SECTOR_210 ((uint32_t)0x8069000) /* Base @ of Sector 210, 2 Kbytes */
#define ADDR_FLASH_SECTOR_211 ((uint32_t)0x8069800) /* Base @ of Sector 211, 2 Kbytes */
#define ADDR_FLASH_SECTOR_212 ((uint32_t)0x806a000) /* Base @ of Sector 212, 2 Kbytes */
#define ADDR_FLASH_SECTOR_213 ((uint32_t)0x806a800) /* Base @ of Sector 213, 2 Kbytes */
#define ADDR_FLASH_SECTOR_214 ((uint32_t)0x806b000) /* Base @ of Sector 214, 2 Kbytes */
#define ADDR_FLASH_SECTOR_215 ((uint32_t)0x806b800) /* Base @ of Sector 215, 2 Kbytes */
#define ADDR_FLASH_SECTOR_216 ((uint32_t)0x806c000) /* Base @ of Sector 216, 2 Kbytes */
#define ADDR_FLASH_SECTOR_217 ((uint32_t)0x806c800) /* Base @ of Sector 217, 2 Kbytes */
#define ADDR_FLASH_SECTOR_218 ((uint32_t)0x806d000) /* Base @ of Sector 218, 2 Kbytes */
#define ADDR_FLASH_SECTOR_219 ((uint32_t)0x806d800) /* Base @ of Sector 219, 2 Kbytes */
#define ADDR_FLASH_SECTOR_220 ((uint32_t)0x806e000) /* Base @ of Sector 220, 2 Kbytes */
#define ADDR_FLASH_SECTOR_221 ((uint32_t)0x806e800) /* Base @ of Sector 221, 2 Kbytes */
#define ADDR_FLASH_SECTOR_222 ((uint32_t)0x806f000) /* Base @ of Sector 222, 2 Kbytes */
#define ADDR_FLASH_SECTOR_223 ((uint32_t)0x806f800) /* Base @ of Sector 223, 2 Kbytes */
#define ADDR_FLASH_SECTOR_224 ((uint32_t)0x8070000) /* Base @ of Sector 224, 2 Kbytes */
#define ADDR_FLASH_SECTOR_225 ((uint32_t)0x8070800) /* Base @ of Sector 225, 2 Kbytes */
#define ADDR_FLASH_SECTOR_226 ((uint32_t)0x8071000) /* Base @ of Sector 226, 2 Kbytes */
#define ADDR_FLASH_SECTOR_227 ((uint32_t)0x8071800) /* Base @ of Sector 227, 2 Kbytes */
#define ADDR_FLASH_SECTOR_228 ((uint32_t)0x8072000) /* Base @ of Sector 228, 2 Kbytes */
#define ADDR_FLASH_SECTOR_229 ((uint32_t)0x8072800) /* Base @ of Sector 229, 2 Kbytes */
#define ADDR_FLASH_SECTOR_230 ((uint32_t)0x8073000) /* Base @ of Sector 230, 2 Kbytes */
#define ADDR_FLASH_SECTOR_231 ((uint32_t)0x8073800) /* Base @ of Sector 231, 2 Kbytes */
#define ADDR_FLASH_SECTOR_232 ((uint32_t)0x8074000) /* Base @ of Sector 232, 2 Kbytes */
#define ADDR_FLASH_SECTOR_233 ((uint32_t)0x8074800) /* Base @ of Sector 233, 2 Kbytes */
#define ADDR_FLASH_SECTOR_234 ((uint32_t)0x8075000) /* Base @ of Sector 234, 2 Kbytes */
#define ADDR_FLASH_SECTOR_235 ((uint32_t)0x8075800) /* Base @ of Sector 235, 2 Kbytes */
#define ADDR_FLASH_SECTOR_236 ((uint32_t)0x8076000) /* Base @ of Sector 236, 2 Kbytes */
#define ADDR_FLASH_SECTOR_237 ((uint32_t)0x8076800) /* Base @ of Sector 237, 2 Kbytes */
#define ADDR_FLASH_SECTOR_238 ((uint32_t)0x8077000) /* Base @ of Sector 238, 2 Kbytes */
#define ADDR_FLASH_SECTOR_239 ((uint32_t)0x8077800) /* Base @ of Sector 239, 2 Kbytes */
#define ADDR_FLASH_SECTOR_240 ((uint32_t)0x8078000) /* Base @ of Sector 240, 2 Kbytes */
#define ADDR_FLASH_SECTOR_241 ((uint32_t)0x8078800) /* Base @ of Sector 241, 2 Kbytes */
#define ADDR_FLASH_SECTOR_242 ((uint32_t)0x8079000) /* Base @ of Sector 242, 2 Kbytes */
#define ADDR_FLASH_SECTOR_243 ((uint32_t)0x8079800) /* Base @ of Sector 243, 2 Kbytes */
#define ADDR_FLASH_SECTOR_244 ((uint32_t)0x807a000) /* Base @ of Sector 244, 2 Kbytes */
#define ADDR_FLASH_SECTOR_245 ((uint32_t)0x807a800) /* Base @ of Sector 245, 2 Kbytes */
#define ADDR_FLASH_SECTOR_246 ((uint32_t)0x807b000) /* Base @ of Sector 246, 2 Kbytes */
#define ADDR_FLASH_SECTOR_247 ((uint32_t)0x807b800) /* Base @ of Sector 247, 2 Kbytes */
#define ADDR_FLASH_SECTOR_248 ((uint32_t)0x807c000) /* Base @ of Sector 248, 2 Kbytes */
#define ADDR_FLASH_SECTOR_249 ((uint32_t)0x807c800) /* Base @ of Sector 249, 2 Kbytes */
#define ADDR_FLASH_SECTOR_250 ((uint32_t)0x807d000) /* Base @ of Sector 250, 2 Kbytes */
#define ADDR_FLASH_SECTOR_251 ((uint32_t)0x807d800) /* Base @ of Sector 251, 2 Kbytes */
#define ADDR_FLASH_SECTOR_252 ((uint32_t)0x807e000) /* Base @ of Sector 252, 2 Kbytes */
#define ADDR_FLASH_SECTOR_253 ((uint32_t)0x807e800) /* Base @ of Sector 253, 2 Kbytes */
#define ADDR_FLASH_SECTOR_254 ((uint32_t)0x807f000) /* Base @ of Sector 254, 2 Kbytes */
#define ADDR_FLASH_SECTOR_255 ((uint32_t)0x807f800) /* Base @ of Sector 255, 2 Kbytes */
#define ADDR_FLASH_SECTOR_256 ((uint32_t)0x8080000) /* Base @ of Sector 256, 2 Kbytes */
#define ADDR_FLASH_SECTOR_257 ((uint32_t)0x8080800) /* Base @ of Sector 257, 2 Kbytes */
#define ADDR_FLASH_SECTOR_258 ((uint32_t)0x8081000) /* Base @ of Sector 258, 2 Kbytes */
#define ADDR_FLASH_SECTOR_259 ((uint32_t)0x8081800) /* Base @ of Sector 259, 2 Kbytes */
#define ADDR_FLASH_SECTOR_260 ((uint32_t)0x8082000) /* Base @ of Sector 260, 2 Kbytes */
#define ADDR_FLASH_SECTOR_261 ((uint32_t)0x8082800) /* Base @ of Sector 261, 2 Kbytes */
#define ADDR_FLASH_SECTOR_262 ((uint32_t)0x8083000) /* Base @ of Sector 262, 2 Kbytes */
#define ADDR_FLASH_SECTOR_263 ((uint32_t)0x8083800) /* Base @ of Sector 263, 2 Kbytes */
#define ADDR_FLASH_SECTOR_264 ((uint32_t)0x8084000) /* Base @ of Sector 264, 2 Kbytes */
#define ADDR_FLASH_SECTOR_265 ((uint32_t)0x8084800) /* Base @ of Sector 265, 2 Kbytes */
#define ADDR_FLASH_SECTOR_266 ((uint32_t)0x8085000) /* Base @ of Sector 266, 2 Kbytes */
#define ADDR_FLASH_SECTOR_267 ((uint32_t)0x8085800) /* Base @ of Sector 267, 2 Kbytes */
#define ADDR_FLASH_SECTOR_268 ((uint32_t)0x8086000) /* Base @ of Sector 268, 2 Kbytes */
#define ADDR_FLASH_SECTOR_269 ((uint32_t)0x8086800) /* Base @ of Sector 269, 2 Kbytes */
#define ADDR_FLASH_SECTOR_270 ((uint32_t)0x8087000) /* Base @ of Sector 270, 2 Kbytes */
#define ADDR_FLASH_SECTOR_271 ((uint32_t)0x8087800) /* Base @ of Sector 271, 2 Kbytes */
#define ADDR_FLASH_SECTOR_272 ((uint32_t)0x8088000) /* Base @ of Sector 272, 2 Kbytes */
#define ADDR_FLASH_SECTOR_273 ((uint32_t)0x8088800) /* Base @ of Sector 273, 2 Kbytes */
#define ADDR_FLASH_SECTOR_274 ((uint32_t)0x8089000) /* Base @ of Sector 274, 2 Kbytes */
#define ADDR_FLASH_SECTOR_275 ((uint32_t)0x8089800) /* Base @ of Sector 275, 2 Kbytes */
#define ADDR_FLASH_SECTOR_276 ((uint32_t)0x808a000) /* Base @ of Sector 276, 2 Kbytes */
#define ADDR_FLASH_SECTOR_277 ((uint32_t)0x808a800) /* Base @ of Sector 277, 2 Kbytes */
#define ADDR_FLASH_SECTOR_278 ((uint32_t)0x808b000) /* Base @ of Sector 278, 2 Kbytes */
#define ADDR_FLASH_SECTOR_279 ((uint32_t)0x808b800) /* Base @ of Sector 279, 2 Kbytes */
#define ADDR_FLASH_SECTOR_280 ((uint32_t)0x808c000) /* Base @ of Sector 280, 2 Kbytes */
#define ADDR_FLASH_SECTOR_281 ((uint32_t)0x808c800) /* Base @ of Sector 281, 2 Kbytes */
#define ADDR_FLASH_SECTOR_282 ((uint32_t)0x808d000) /* Base @ of Sector 282, 2 Kbytes */
#define ADDR_FLASH_SECTOR_283 ((uint32_t)0x808d800) /* Base @ of Sector 283, 2 Kbytes */
#define ADDR_FLASH_SECTOR_284 ((uint32_t)0x808e000) /* Base @ of Sector 284, 2 Kbytes */
#define ADDR_FLASH_SECTOR_285 ((uint32_t)0x808e800) /* Base @ of Sector 285, 2 Kbytes */
#define ADDR_FLASH_SECTOR_286 ((uint32_t)0x808f000) /* Base @ of Sector 286, 2 Kbytes */
#define ADDR_FLASH_SECTOR_287 ((uint32_t)0x808f800) /* Base @ of Sector 287, 2 Kbytes */
#define ADDR_FLASH_SECTOR_288 ((uint32_t)0x8090000) /* Base @ of Sector 288, 2 Kbytes */
#define ADDR_FLASH_SECTOR_289 ((uint32_t)0x8090800) /* Base @ of Sector 289, 2 Kbytes */
#define ADDR_FLASH_SECTOR_290 ((uint32_t)0x8091000) /* Base @ of Sector 290, 2 Kbytes */
#define ADDR_FLASH_SECTOR_291 ((uint32_t)0x8091800) /* Base @ of Sector 291, 2 Kbytes */
#define ADDR_FLASH_SECTOR_292 ((uint32_t)0x8092000) /* Base @ of Sector 292, 2 Kbytes */
#define ADDR_FLASH_SECTOR_293 ((uint32_t)0x8092800) /* Base @ of Sector 293, 2 Kbytes */
#define ADDR_FLASH_SECTOR_294 ((uint32_t)0x8093000) /* Base @ of Sector 294, 2 Kbytes */
#define ADDR_FLASH_SECTOR_295 ((uint32_t)0x8093800) /* Base @ of Sector 295, 2 Kbytes */
#define ADDR_FLASH_SECTOR_296 ((uint32_t)0x8094000) /* Base @ of Sector 296, 2 Kbytes */
#define ADDR_FLASH_SECTOR_297 ((uint32_t)0x8094800) /* Base @ of Sector 297, 2 Kbytes */
#define ADDR_FLASH_SECTOR_298 ((uint32_t)0x8095000) /* Base @ of Sector 298, 2 Kbytes */
#define ADDR_FLASH_SECTOR_299 ((uint32_t)0x8095800) /* Base @ of Sector 299, 2 Kbytes */
#define ADDR_FLASH_SECTOR_300 ((uint32_t)0x8096000) /* Base @ of Sector 300, 2 Kbytes */
#define ADDR_FLASH_SECTOR_301 ((uint32_t)0x8096800) /* Base @ of Sector 301, 2 Kbytes */
#define ADDR_FLASH_SECTOR_302 ((uint32_t)0x8097000) /* Base @ of Sector 302, 2 Kbytes */
#define ADDR_FLASH_SECTOR_303 ((uint32_t)0x8097800) /* Base @ of Sector 303, 2 Kbytes */
#define ADDR_FLASH_SECTOR_304 ((uint32_t)0x8098000) /* Base @ of Sector 304, 2 Kbytes */
#define ADDR_FLASH_SECTOR_305 ((uint32_t)0x8098800) /* Base @ of Sector 305, 2 Kbytes */
#define ADDR_FLASH_SECTOR_306 ((uint32_t)0x8099000) /* Base @ of Sector 306, 2 Kbytes */
#define ADDR_FLASH_SECTOR_307 ((uint32_t)0x8099800) /* Base @ of Sector 307, 2 Kbytes */
#define ADDR_FLASH_SECTOR_308 ((uint32_t)0x809a000) /* Base @ of Sector 308, 2 Kbytes */
#define ADDR_FLASH_SECTOR_309 ((uint32_t)0x809a800) /* Base @ of Sector 309, 2 Kbytes */
#define ADDR_FLASH_SECTOR_310 ((uint32_t)0x809b000) /* Base @ of Sector 310, 2 Kbytes */
#define ADDR_FLASH_SECTOR_311 ((uint32_t)0x809b800) /* Base @ of Sector 311, 2 Kbytes */
#define ADDR_FLASH_SECTOR_312 ((uint32_t)0x809c000) /* Base @ of Sector 312, 2 Kbytes */
#define ADDR_FLASH_SECTOR_313 ((uint32_t)0x809c800) /* Base @ of Sector 313, 2 Kbytes */
#define ADDR_FLASH_SECTOR_314 ((uint32_t)0x809d000) /* Base @ of Sector 314, 2 Kbytes */
#define ADDR_FLASH_SECTOR_315 ((uint32_t)0x809d800) /* Base @ of Sector 315, 2 Kbytes */
#define ADDR_FLASH_SECTOR_316 ((uint32_t)0x809e000) /* Base @ of Sector 316, 2 Kbytes */
#define ADDR_FLASH_SECTOR_317 ((uint32_t)0x809e800) /* Base @ of Sector 317, 2 Kbytes */
#define ADDR_FLASH_SECTOR_318 ((uint32_t)0x809f000) /* Base @ of Sector 318, 2 Kbytes */
#define ADDR_FLASH_SECTOR_319 ((uint32_t)0x809f800) /* Base @ of Sector 319, 2 Kbytes */
#define ADDR_FLASH_SECTOR_320 ((uint32_t)0x80a0000) /* Base @ of Sector 320, 2 Kbytes */
#define ADDR_FLASH_SECTOR_321 ((uint32_t)0x80a0800) /* Base @ of Sector 321, 2 Kbytes */
#define ADDR_FLASH_SECTOR_322 ((uint32_t)0x80a1000) /* Base @ of Sector 322, 2 Kbytes */
#define ADDR_FLASH_SECTOR_323 ((uint32_t)0x80a1800) /* Base @ of Sector 323, 2 Kbytes */
#define ADDR_FLASH_SECTOR_324 ((uint32_t)0x80a2000) /* Base @ of Sector 324, 2 Kbytes */
#define ADDR_FLASH_SECTOR_325 ((uint32_t)0x80a2800) /* Base @ of Sector 325, 2 Kbytes */
#define ADDR_FLASH_SECTOR_326 ((uint32_t)0x80a3000) /* Base @ of Sector 326, 2 Kbytes */
#define ADDR_FLASH_SECTOR_327 ((uint32_t)0x80a3800) /* Base @ of Sector 327, 2 Kbytes */
#define ADDR_FLASH_SECTOR_328 ((uint32_t)0x80a4000) /* Base @ of Sector 328, 2 Kbytes */
#define ADDR_FLASH_SECTOR_329 ((uint32_t)0x80a4800) /* Base @ of Sector 329, 2 Kbytes */
#define ADDR_FLASH_SECTOR_330 ((uint32_t)0x80a5000) /* Base @ of Sector 330, 2 Kbytes */
#define ADDR_FLASH_SECTOR_331 ((uint32_t)0x80a5800) /* Base @ of Sector 331, 2 Kbytes */
#define ADDR_FLASH_SECTOR_332 ((uint32_t)0x80a6000) /* Base @ of Sector 332, 2 Kbytes */
#define ADDR_FLASH_SECTOR_333 ((uint32_t)0x80a6800) /* Base @ of Sector 333, 2 Kbytes */
#define ADDR_FLASH_SECTOR_334 ((uint32_t)0x80a7000) /* Base @ of Sector 334, 2 Kbytes */
#define ADDR_FLASH_SECTOR_335 ((uint32_t)0x80a7800) /* Base @ of Sector 335, 2 Kbytes */
#define ADDR_FLASH_SECTOR_336 ((uint32_t)0x80a8000) /* Base @ of Sector 336, 2 Kbytes */
#define ADDR_FLASH_SECTOR_337 ((uint32_t)0x80a8800) /* Base @ of Sector 337, 2 Kbytes */
#define ADDR_FLASH_SECTOR_338 ((uint32_t)0x80a9000) /* Base @ of Sector 338, 2 Kbytes */
#define ADDR_FLASH_SECTOR_339 ((uint32_t)0x80a9800) /* Base @ of Sector 339, 2 Kbytes */
#define ADDR_FLASH_SECTOR_340 ((uint32_t)0x80aa000) /* Base @ of Sector 340, 2 Kbytes */
#define ADDR_FLASH_SECTOR_341 ((uint32_t)0x80aa800) /* Base @ of Sector 341, 2 Kbytes */
#define ADDR_FLASH_SECTOR_342 ((uint32_t)0x80ab000) /* Base @ of Sector 342, 2 Kbytes */
#define ADDR_FLASH_SECTOR_343 ((uint32_t)0x80ab800) /* Base @ of Sector 343, 2 Kbytes */
#define ADDR_FLASH_SECTOR_344 ((uint32_t)0x80ac000) /* Base @ of Sector 344, 2 Kbytes */
#define ADDR_FLASH_SECTOR_345 ((uint32_t)0x80ac800) /* Base @ of Sector 345, 2 Kbytes */
#define ADDR_FLASH_SECTOR_346 ((uint32_t)0x80ad000) /* Base @ of Sector 346, 2 Kbytes */
#define ADDR_FLASH_SECTOR_347 ((uint32_t)0x80ad800) /* Base @ of Sector 347, 2 Kbytes */
#define ADDR_FLASH_SECTOR_348 ((uint32_t)0x80ae000) /* Base @ of Sector 348, 2 Kbytes */
#define ADDR_FLASH_SECTOR_349 ((uint32_t)0x80ae800) /* Base @ of Sector 349, 2 Kbytes */
#define ADDR_FLASH_SECTOR_350 ((uint32_t)0x80af000) /* Base @ of Sector 350, 2 Kbytes */
#define ADDR_FLASH_SECTOR_351 ((uint32_t)0x80af800) /* Base @ of Sector 351, 2 Kbytes */
#define ADDR_FLASH_SECTOR_352 ((uint32_t)0x80b0000) /* Base @ of Sector 352, 2 Kbytes */
#define ADDR_FLASH_SECTOR_353 ((uint32_t)0x80b0800) /* Base @ of Sector 353, 2 Kbytes */
#define ADDR_FLASH_SECTOR_354 ((uint32_t)0x80b1000) /* Base @ of Sector 354, 2 Kbytes */
#define ADDR_FLASH_SECTOR_355 ((uint32_t)0x80b1800) /* Base @ of Sector 355, 2 Kbytes */
#define ADDR_FLASH_SECTOR_356 ((uint32_t)0x80b2000) /* Base @ of Sector 356, 2 Kbytes */
#define ADDR_FLASH_SECTOR_357 ((uint32_t)0x80b2800) /* Base @ of Sector 357, 2 Kbytes */
#define ADDR_FLASH_SECTOR_358 ((uint32_t)0x80b3000) /* Base @ of Sector 358, 2 Kbytes */
#define ADDR_FLASH_SECTOR_359 ((uint32_t)0x80b3800) /* Base @ of Sector 359, 2 Kbytes */
#define ADDR_FLASH_SECTOR_360 ((uint32_t)0x80b4000) /* Base @ of Sector 360, 2 Kbytes */
#define ADDR_FLASH_SECTOR_361 ((uint32_t)0x80b4800) /* Base @ of Sector 361, 2 Kbytes */
#define ADDR_FLASH_SECTOR_362 ((uint32_t)0x80b5000) /* Base @ of Sector 362, 2 Kbytes */
#define ADDR_FLASH_SECTOR_363 ((uint32_t)0x80b5800) /* Base @ of Sector 363, 2 Kbytes */
#define ADDR_FLASH_SECTOR_364 ((uint32_t)0x80b6000) /* Base @ of Sector 364, 2 Kbytes */
#define ADDR_FLASH_SECTOR_365 ((uint32_t)0x80b6800) /* Base @ of Sector 365, 2 Kbytes */
#define ADDR_FLASH_SECTOR_366 ((uint32_t)0x80b7000) /* Base @ of Sector 366, 2 Kbytes */
#define ADDR_FLASH_SECTOR_367 ((uint32_t)0x80b7800) /* Base @ of Sector 367, 2 Kbytes */
#define ADDR_FLASH_SECTOR_368 ((uint32_t)0x80b8000) /* Base @ of Sector 368, 2 Kbytes */
#define ADDR_FLASH_SECTOR_369 ((uint32_t)0x80b8800) /* Base @ of Sector 369, 2 Kbytes */
#define ADDR_FLASH_SECTOR_370 ((uint32_t)0x80b9000) /* Base @ of Sector 370, 2 Kbytes */
#define ADDR_FLASH_SECTOR_371 ((uint32_t)0x80b9800) /* Base @ of Sector 371, 2 Kbytes */
#define ADDR_FLASH_SECTOR_372 ((uint32_t)0x80ba000) /* Base @ of Sector 372, 2 Kbytes */
#define ADDR_FLASH_SECTOR_373 ((uint32_t)0x80ba800) /* Base @ of Sector 373, 2 Kbytes */
#define ADDR_FLASH_SECTOR_374 ((uint32_t)0x80bb000) /* Base @ of Sector 374, 2 Kbytes */
#define ADDR_FLASH_SECTOR_375 ((uint32_t)0x80bb800) /* Base @ of Sector 375, 2 Kbytes */
#define ADDR_FLASH_SECTOR_376 ((uint32_t)0x80bc000) /* Base @ of Sector 376, 2 Kbytes */
#define ADDR_FLASH_SECTOR_377 ((uint32_t)0x80bc800) /* Base @ of Sector 377, 2 Kbytes */
#define ADDR_FLASH_SECTOR_378 ((uint32_t)0x80bd000) /* Base @ of Sector 378, 2 Kbytes */
#define ADDR_FLASH_SECTOR_379 ((uint32_t)0x80bd800) /* Base @ of Sector 379, 2 Kbytes */
#define ADDR_FLASH_SECTOR_380 ((uint32_t)0x80be000) /* Base @ of Sector 380, 2 Kbytes */
#define ADDR_FLASH_SECTOR_381 ((uint32_t)0x80be800) /* Base @ of Sector 381, 2 Kbytes */
#define ADDR_FLASH_SECTOR_382 ((uint32_t)0x80bf000) /* Base @ of Sector 382, 2 Kbytes */
#define ADDR_FLASH_SECTOR_383 ((uint32_t)0x80bf800) /* Base @ of Sector 383, 2 Kbytes */
#define ADDR_FLASH_SECTOR_384 ((uint32_t)0x80c0000) /* Base @ of Sector 384, 2 Kbytes */
#define ADDR_FLASH_SECTOR_385 ((uint32_t)0x80c0800) /* Base @ of Sector 385, 2 Kbytes */
#define ADDR_FLASH_SECTOR_386 ((uint32_t)0x80c1000) /* Base @ of Sector 386, 2 Kbytes */
#define ADDR_FLASH_SECTOR_387 ((uint32_t)0x80c1800) /* Base @ of Sector 387, 2 Kbytes */
#define ADDR_FLASH_SECTOR_388 ((uint32_t)0x80c2000) /* Base @ of Sector 388, 2 Kbytes */
#define ADDR_FLASH_SECTOR_389 ((uint32_t)0x80c2800) /* Base @ of Sector 389, 2 Kbytes */
#define ADDR_FLASH_SECTOR_390 ((uint32_t)0x80c3000) /* Base @ of Sector 390, 2 Kbytes */
#define ADDR_FLASH_SECTOR_391 ((uint32_t)0x80c3800) /* Base @ of Sector 391, 2 Kbytes */
#define ADDR_FLASH_SECTOR_392 ((uint32_t)0x80c4000) /* Base @ of Sector 392, 2 Kbytes */
#define ADDR_FLASH_SECTOR_393 ((uint32_t)0x80c4800) /* Base @ of Sector 393, 2 Kbytes */
#define ADDR_FLASH_SECTOR_394 ((uint32_t)0x80c5000) /* Base @ of Sector 394, 2 Kbytes */
#define ADDR_FLASH_SECTOR_395 ((uint32_t)0x80c5800) /* Base @ of Sector 395, 2 Kbytes */
#define ADDR_FLASH_SECTOR_396 ((uint32_t)0x80c6000) /* Base @ of Sector 396, 2 Kbytes */
#define ADDR_FLASH_SECTOR_397 ((uint32_t)0x80c6800) /* Base @ of Sector 397, 2 Kbytes */
#define ADDR_FLASH_SECTOR_398 ((uint32_t)0x80c7000) /* Base @ of Sector 398, 2 Kbytes */
#define ADDR_FLASH_SECTOR_399 ((uint32_t)0x80c7800) /* Base @ of Sector 399, 2 Kbytes */
#define ADDR_FLASH_SECTOR_400 ((uint32_t)0x80c8000) /* Base @ of Sector 400, 2 Kbytes */
#define ADDR_FLASH_SECTOR_401 ((uint32_t)0x80c8800) /* Base @ of Sector 401, 2 Kbytes */
#define ADDR_FLASH_SECTOR_402 ((uint32_t)0x80c9000) /* Base @ of Sector 402, 2 Kbytes */
#define ADDR_FLASH_SECTOR_403 ((uint32_t)0x80c9800) /* Base @ of Sector 403, 2 Kbytes */
#define ADDR_FLASH_SECTOR_404 ((uint32_t)0x80ca000) /* Base @ of Sector 404, 2 Kbytes */
#define ADDR_FLASH_SECTOR_405 ((uint32_t)0x80ca800) /* Base @ of Sector 405, 2 Kbytes */
#define ADDR_FLASH_SECTOR_406 ((uint32_t)0x80cb000) /* Base @ of Sector 406, 2 Kbytes */
#define ADDR_FLASH_SECTOR_407 ((uint32_t)0x80cb800) /* Base @ of Sector 407, 2 Kbytes */
#define ADDR_FLASH_SECTOR_408 ((uint32_t)0x80cc000) /* Base @ of Sector 408, 2 Kbytes */
#define ADDR_FLASH_SECTOR_409 ((uint32_t)0x80cc800) /* Base @ of Sector 409, 2 Kbytes */
#define ADDR_FLASH_SECTOR_410 ((uint32_t)0x80cd000) /* Base @ of Sector 410, 2 Kbytes */
#define ADDR_FLASH_SECTOR_411 ((uint32_t)0x80cd800) /* Base @ of Sector 411, 2 Kbytes */
#define ADDR_FLASH_SECTOR_412 ((uint32_t)0x80ce000) /* Base @ of Sector 412, 2 Kbytes */
#define ADDR_FLASH_SECTOR_413 ((uint32_t)0x80ce800) /* Base @ of Sector 413, 2 Kbytes */
#define ADDR_FLASH_SECTOR_414 ((uint32_t)0x80cf000) /* Base @ of Sector 414, 2 Kbytes */
#define ADDR_FLASH_SECTOR_415 ((uint32_t)0x80cf800) /* Base @ of Sector 415, 2 Kbytes */
#define ADDR_FLASH_SECTOR_416 ((uint32_t)0x80d0000) /* Base @ of Sector 416, 2 Kbytes */
#define ADDR_FLASH_SECTOR_417 ((uint32_t)0x80d0800) /* Base @ of Sector 417, 2 Kbytes */
#define ADDR_FLASH_SECTOR_418 ((uint32_t)0x80d1000) /* Base @ of Sector 418, 2 Kbytes */
#define ADDR_FLASH_SECTOR_419 ((uint32_t)0x80d1800) /* Base @ of Sector 419, 2 Kbytes */
#define ADDR_FLASH_SECTOR_420 ((uint32_t)0x80d2000) /* Base @ of Sector 420, 2 Kbytes */
#define ADDR_FLASH_SECTOR_421 ((uint32_t)0x80d2800) /* Base @ of Sector 421, 2 Kbytes */
#define ADDR_FLASH_SECTOR_422 ((uint32_t)0x80d3000) /* Base @ of Sector 422, 2 Kbytes */
#define ADDR_FLASH_SECTOR_423 ((uint32_t)0x80d3800) /* Base @ of Sector 423, 2 Kbytes */
#define ADDR_FLASH_SECTOR_424 ((uint32_t)0x80d4000) /* Base @ of Sector 424, 2 Kbytes */
#define ADDR_FLASH_SECTOR_425 ((uint32_t)0x80d4800) /* Base @ of Sector 425, 2 Kbytes */
#define ADDR_FLASH_SECTOR_426 ((uint32_t)0x80d5000) /* Base @ of Sector 426, 2 Kbytes */
#define ADDR_FLASH_SECTOR_427 ((uint32_t)0x80d5800) /* Base @ of Sector 427, 2 Kbytes */
#define ADDR_FLASH_SECTOR_428 ((uint32_t)0x80d6000) /* Base @ of Sector 428, 2 Kbytes */
#define ADDR_FLASH_SECTOR_429 ((uint32_t)0x80d6800) /* Base @ of Sector 429, 2 Kbytes */
#define ADDR_FLASH_SECTOR_430 ((uint32_t)0x80d7000) /* Base @ of Sector 430, 2 Kbytes */
#define ADDR_FLASH_SECTOR_431 ((uint32_t)0x80d7800) /* Base @ of Sector 431, 2 Kbytes */
#define ADDR_FLASH_SECTOR_432 ((uint32_t)0x80d8000) /* Base @ of Sector 432, 2 Kbytes */
#define ADDR_FLASH_SECTOR_433 ((uint32_t)0x80d8800) /* Base @ of Sector 433, 2 Kbytes */
#define ADDR_FLASH_SECTOR_434 ((uint32_t)0x80d9000) /* Base @ of Sector 434, 2 Kbytes */
#define ADDR_FLASH_SECTOR_435 ((uint32_t)0x80d9800) /* Base @ of Sector 435, 2 Kbytes */
#define ADDR_FLASH_SECTOR_436 ((uint32_t)0x80da000) /* Base @ of Sector 436, 2 Kbytes */
#define ADDR_FLASH_SECTOR_437 ((uint32_t)0x80da800) /* Base @ of Sector 437, 2 Kbytes */
#define ADDR_FLASH_SECTOR_438 ((uint32_t)0x80db000) /* Base @ of Sector 438, 2 Kbytes */
#define ADDR_FLASH_SECTOR_439 ((uint32_t)0x80db800) /* Base @ of Sector 439, 2 Kbytes */
#define ADDR_FLASH_SECTOR_440 ((uint32_t)0x80dc000) /* Base @ of Sector 440, 2 Kbytes */
#define ADDR_FLASH_SECTOR_441 ((uint32_t)0x80dc800) /* Base @ of Sector 441, 2 Kbytes */
#define ADDR_FLASH_SECTOR_442 ((uint32_t)0x80dd000) /* Base @ of Sector 442, 2 Kbytes */
#define ADDR_FLASH_SECTOR_443 ((uint32_t)0x80dd800) /* Base @ of Sector 443, 2 Kbytes */
#define ADDR_FLASH_SECTOR_444 ((uint32_t)0x80de000) /* Base @ of Sector 444, 2 Kbytes */
#define ADDR_FLASH_SECTOR_445 ((uint32_t)0x80de800) /* Base @ of Sector 445, 2 Kbytes */
#define ADDR_FLASH_SECTOR_446 ((uint32_t)0x80df000) /* Base @ of Sector 446, 2 Kbytes */
#define ADDR_FLASH_SECTOR_447 ((uint32_t)0x80df800) /* Base @ of Sector 447, 2 Kbytes */
#define ADDR_FLASH_SECTOR_448 ((uint32_t)0x80e0000) /* Base @ of Sector 448, 2 Kbytes */
#define ADDR_FLASH_SECTOR_449 ((uint32_t)0x80e0800) /* Base @ of Sector 449, 2 Kbytes */
#define ADDR_FLASH_SECTOR_450 ((uint32_t)0x80e1000) /* Base @ of Sector 450, 2 Kbytes */
#define ADDR_FLASH_SECTOR_451 ((uint32_t)0x80e1800) /* Base @ of Sector 451, 2 Kbytes */
#define ADDR_FLASH_SECTOR_452 ((uint32_t)0x80e2000) /* Base @ of Sector 452, 2 Kbytes */
#define ADDR_FLASH_SECTOR_453 ((uint32_t)0x80e2800) /* Base @ of Sector 453, 2 Kbytes */
#define ADDR_FLASH_SECTOR_454 ((uint32_t)0x80e3000) /* Base @ of Sector 454, 2 Kbytes */
#define ADDR_FLASH_SECTOR_455 ((uint32_t)0x80e3800) /* Base @ of Sector 455, 2 Kbytes */
#define ADDR_FLASH_SECTOR_456 ((uint32_t)0x80e4000) /* Base @ of Sector 456, 2 Kbytes */
#define ADDR_FLASH_SECTOR_457 ((uint32_t)0x80e4800) /* Base @ of Sector 457, 2 Kbytes */
#define ADDR_FLASH_SECTOR_458 ((uint32_t)0x80e5000) /* Base @ of Sector 458, 2 Kbytes */
#define ADDR_FLASH_SECTOR_459 ((uint32_t)0x80e5800) /* Base @ of Sector 459, 2 Kbytes */
#define ADDR_FLASH_SECTOR_460 ((uint32_t)0x80e6000) /* Base @ of Sector 460, 2 Kbytes */
#define ADDR_FLASH_SECTOR_461 ((uint32_t)0x80e6800) /* Base @ of Sector 461, 2 Kbytes */
#define ADDR_FLASH_SECTOR_462 ((uint32_t)0x80e7000) /* Base @ of Sector 462, 2 Kbytes */
#define ADDR_FLASH_SECTOR_463 ((uint32_t)0x80e7800) /* Base @ of Sector 463, 2 Kbytes */
#define ADDR_FLASH_SECTOR_464 ((uint32_t)0x80e8000) /* Base @ of Sector 464, 2 Kbytes */
#define ADDR_FLASH_SECTOR_465 ((uint32_t)0x80e8800) /* Base @ of Sector 465, 2 Kbytes */
#define ADDR_FLASH_SECTOR_466 ((uint32_t)0x80e9000) /* Base @ of Sector 466, 2 Kbytes */
#define ADDR_FLASH_SECTOR_467 ((uint32_t)0x80e9800) /* Base @ of Sector 467, 2 Kbytes */
#define ADDR_FLASH_SECTOR_468 ((uint32_t)0x80ea000) /* Base @ of Sector 468, 2 Kbytes */
#define ADDR_FLASH_SECTOR_469 ((uint32_t)0x80ea800) /* Base @ of Sector 469, 2 Kbytes */
#define ADDR_FLASH_SECTOR_470 ((uint32_t)0x80eb000) /* Base @ of Sector 470, 2 Kbytes */
#define ADDR_FLASH_SECTOR_471 ((uint32_t)0x80eb800) /* Base @ of Sector 471, 2 Kbytes */
#define ADDR_FLASH_SECTOR_472 ((uint32_t)0x80ec000) /* Base @ of Sector 472, 2 Kbytes */
#define ADDR_FLASH_SECTOR_473 ((uint32_t)0x80ec800) /* Base @ of Sector 473, 2 Kbytes */
#define ADDR_FLASH_SECTOR_474 ((uint32_t)0x80ed000) /* Base @ of Sector 474, 2 Kbytes */
#define ADDR_FLASH_SECTOR_475 ((uint32_t)0x80ed800) /* Base @ of Sector 475, 2 Kbytes */
#define ADDR_FLASH_SECTOR_476 ((uint32_t)0x80ee000) /* Base @ of Sector 476, 2 Kbytes */
#define ADDR_FLASH_SECTOR_477 ((uint32_t)0x80ee800) /* Base @ of Sector 477, 2 Kbytes */
#define ADDR_FLASH_SECTOR_478 ((uint32_t)0x80ef000) /* Base @ of Sector 478, 2 Kbytes */
#define ADDR_FLASH_SECTOR_479 ((uint32_t)0x80ef800) /* Base @ of Sector 479, 2 Kbytes */
#define ADDR_FLASH_SECTOR_480 ((uint32_t)0x80f0000) /* Base @ of Sector 480, 2 Kbytes */
#define ADDR_FLASH_SECTOR_481 ((uint32_t)0x80f0800) /* Base @ of Sector 481, 2 Kbytes */
#define ADDR_FLASH_SECTOR_482 ((uint32_t)0x80f1000) /* Base @ of Sector 482, 2 Kbytes */
#define ADDR_FLASH_SECTOR_483 ((uint32_t)0x80f1800) /* Base @ of Sector 483, 2 Kbytes */
#define ADDR_FLASH_SECTOR_484 ((uint32_t)0x80f2000) /* Base @ of Sector 484, 2 Kbytes */
#define ADDR_FLASH_SECTOR_485 ((uint32_t)0x80f2800) /* Base @ of Sector 485, 2 Kbytes */
#define ADDR_FLASH_SECTOR_486 ((uint32_t)0x80f3000) /* Base @ of Sector 486, 2 Kbytes */
#define ADDR_FLASH_SECTOR_487 ((uint32_t)0x80f3800) /* Base @ of Sector 487, 2 Kbytes */
#define ADDR_FLASH_SECTOR_488 ((uint32_t)0x80f4000) /* Base @ of Sector 488, 2 Kbytes */
#define ADDR_FLASH_SECTOR_489 ((uint32_t)0x80f4800) /* Base @ of Sector 489, 2 Kbytes */
#define ADDR_FLASH_SECTOR_490 ((uint32_t)0x80f5000) /* Base @ of Sector 490, 2 Kbytes */
#define ADDR_FLASH_SECTOR_491 ((uint32_t)0x80f5800) /* Base @ of Sector 491, 2 Kbytes */
#define ADDR_FLASH_SECTOR_492 ((uint32_t)0x80f6000) /* Base @ of Sector 492, 2 Kbytes */
#define ADDR_FLASH_SECTOR_493 ((uint32_t)0x80f6800) /* Base @ of Sector 493, 2 Kbytes */
#define ADDR_FLASH_SECTOR_494 ((uint32_t)0x80f7000) /* Base @ of Sector 494, 2 Kbytes */
#define ADDR_FLASH_SECTOR_495 ((uint32_t)0x80f7800) /* Base @ of Sector 495, 2 Kbytes */
#define ADDR_FLASH_SECTOR_496 ((uint32_t)0x80f8000) /* Base @ of Sector 496, 2 Kbytes */
#define ADDR_FLASH_SECTOR_497 ((uint32_t)0x80f8800) /* Base @ of Sector 497, 2 Kbytes */
#define ADDR_FLASH_SECTOR_498 ((uint32_t)0x80f9000) /* Base @ of Sector 498, 2 Kbytes */
#define ADDR_FLASH_SECTOR_499 ((uint32_t)0x80f9800) /* Base @ of Sector 499, 2 Kbytes */
#define ADDR_FLASH_SECTOR_500 ((uint32_t)0x80fa000) /* Base @ of Sector 500, 2 Kbytes */
#define ADDR_FLASH_SECTOR_501 ((uint32_t)0x80fa800) /* Base @ of Sector 501, 2 Kbytes */
#define ADDR_FLASH_SECTOR_502 ((uint32_t)0x80fb000) /* Base @ of Sector 502, 2 Kbytes */
#define ADDR_FLASH_SECTOR_503 ((uint32_t)0x80fb800) /* Base @ of Sector 503, 2 Kbytes */
#define ADDR_FLASH_SECTOR_504 ((uint32_t)0x80fc000) /* Base @ of Sector 504, 2 Kbytes */
#define ADDR_FLASH_SECTOR_505 ((uint32_t)0x80fc800) /* Base @ of Sector 505, 2 Kbytes */
#define ADDR_FLASH_SECTOR_506 ((uint32_t)0x80fd000) /* Base @ of Sector 506, 2 Kbytes */
#define ADDR_FLASH_SECTOR_507 ((uint32_t)0x80fd800) /* Base @ of Sector 507, 2 Kbytes */
#define ADDR_FLASH_SECTOR_508 ((uint32_t)0x80fe000) /* Base @ of Sector 508, 2 Kbytes */
#define ADDR_FLASH_SECTOR_509 ((uint32_t)0x80fe800) /* Base @ of Sector 509, 2 Kbytes */
#define ADDR_FLASH_SECTOR_510 ((uint32_t)0x80ff000) /* Base @ of Sector 510, 2 Kbytes */
#define ADDR_FLASH_SECTOR_511 ((uint32_t)0x80ff800) /* Base @ of Sector 511, 2 Kbytes */

#define FLASH_WRITE_GRAN_BITS   (32) // WORD
#define FLASH_SECTOR_SIZE_BYTES (1024 * 2)
#define FLASH_USER_START_ADDR   (ADDR_FLASH_SECTOR_504)
#define FLASH_USER_AREA_SIZE    (8 * FLASH_SECTOR_SIZE_BYTES)
#define USE_FLASHDB
#endif