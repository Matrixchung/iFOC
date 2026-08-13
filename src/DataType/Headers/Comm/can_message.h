#pragma once

namespace iFOC::DataType::Comm
{
struct CANMessage
{
    uint32_t cob_id = 0;
    bool is_ext = false;
    bool is_rtr = false;
    uint8_t len = 0;
    uint8_t data[8]{};
};

typedef enum canfd_dlc : uint8_t
{
    DLC_0_BYTES = 0,
    DLC_1_BYTES = 1,
    DLC_2_BYTES = 2,
    DLC_3_BYTES = 3,
    DLC_4_BYTES = 4,
    DLC_5_BYTES = 5,
    DLC_6_BYTES = 6,
    DLC_7_BYTES = 7,
    DLC_8_BYTES = 8,
    DLC_12_BYTES = 12,
    DLC_16_BYTES = 16,
    DLC_20_BYTES = 20,
    DLC_24_BYTES = 24,
    DLC_32_BYTES = 32,
    DLC_48_BYTES = 48,
    DLC_64_BYTES = 64,
}canfd_dlc;

struct CANFDMessage
{
    // uint32_t cob_id = 0;
    uint8_t cob_id[4]{}; // LSB id (uint32_t)
    bool is_ext = false;
    canfd_dlc dlc = DLC_0_BYTES;
    uint8_t data[64]{};
};
}