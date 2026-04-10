#include "gd32_bkp.hpp"
#include "at32wk_bkp_struct.h"
#include "../../Common/foc_math.hpp"

#if defined(GD32_ENV)

// GD32G553 BKP size: 32 * 4 = 128 bytes, double word r/w

bool read_bkp(bkp_struct_t* dst);
void write_bkp(bkp_struct_t* src);

#include "../../DataType/board_config.hpp"

namespace iFOC::HAL::Bootloader
{
    void JumpToBL(const BootloaderMsg& msg)
    {
        static_assert(sizeof(bkp_struct_t::dronecan_image_path) == sizeof(BootloaderMsg::update_image_path));
        bkp_struct_t bkp;
        if(!read_bkp(&bkp)) return;
        if(!bkp.flags.bit.bootloader_presented) return;
        bkp.flags.bit.update_requested = true;
        bkp.flags.bit.app_init_success = true;
        bkp.flags.bit.controllable_hardfault = false;
        bkp.flags.bit.enable_can_resistor = BoardConfig().GetConfig().enable_can_terminal_resistor();
        bkp.file_server_node_id = msg.server_node_id;
        memcpy(bkp.dronecan_image_path, msg.update_image_path, sizeof(bkp.dronecan_image_path));
        write_bkp(&bkp);
        // do nvic_systemreboot()
        SystemReboot();
    }

    bool HasBL()
    {
        bkp_struct_t bkp;
        if(!read_bkp(&bkp)) return false;
        if(!bkp.flags.bit.bootloader_presented) return false;
        return true;
    }

    void GetBLVersion(uint8_t& major, uint8_t& minor, uint32_t& vcs)
    {
        major = 0;
        minor = 0;
        vcs = 0;
        bkp_struct_t bkp;
        if(!read_bkp(&bkp)) return;
        if(!bkp.flags.bit.bootloader_presented) return;
        major = bkp.version.major;
        minor = bkp.version.minor;
        memcpy(&vcs, bkp.version.vcs_commit, sizeof(bkp.version.vcs_commit));
    }

    void SetAppInitSuccessFlag()
    {
        bkp_struct_t bkp;
        if(!read_bkp(&bkp)) return;
        bkp.flags.bit.update_requested = false;
        bkp.flags.bit.app_init_success = true;
        write_bkp(&bkp);
    }

    void OnHardFault()
    {
        bkp_struct_t bkp;
        if(!read_bkp(&bkp)) return;
        bkp.flags.bit.controllable_hardfault = true;
        if(bkp.controllable_hardfault_count < 255) bkp.controllable_hardfault_count++; // prevent overflow
        write_bkp(&bkp);
    }
#ifdef GD32_TEST_BKP
    void TestBKP(bkp_struct_t* bkp)
    {
        read_bkp(bkp);
        bkp->reserved++;
        write_bkp(bkp);
        // read back
        read_bkp(bkp);
    }
#endif
}

bool read_bkp(bkp_struct_t* dst)
{
    static_assert(sizeof(bkp_struct_t) == 18);

    memset(dst, 0, sizeof(bkp_struct_t));
    uint8_t buffer[sizeof(bkp_struct_t)];

    uint32_t temp = 0;

    temp = RTC_BKP0;
    memcpy(buffer, &temp, 4);

    temp = RTC_BKP1;
    memcpy(buffer + 4, &temp, 4);

    temp = RTC_BKP2;
    memcpy(buffer + 8, &temp, 4);

    temp = RTC_BKP3;
    memcpy(buffer + 12, &temp, 4);

    temp = RTC_BKP4;
    memcpy(buffer + 16, &temp, 2);

    const uint8_t calc_crc8 = iFOC::get_crc8(buffer, sizeof(buffer) - 1);
    if(calc_crc8 == buffer[sizeof(buffer) - 1]) // crc8 matched
    {
        memcpy(dst, buffer, sizeof(bkp_struct_t));
        return true;
    }
    return false;
}

void write_bkp(bkp_struct_t* src)
{
    static_assert(sizeof(bkp_struct_t) == 18);

    uint32_t temp = 0;

    auto* ptr = (const uint8_t*)src;

    src->crc8 = iFOC::get_crc8(ptr, sizeof(bkp_struct_t) - 1);

    // Unlock BKP
    pmu_backup_write_enable();

    memcpy(&temp, ptr, 4);
    RTC_BKP0 = temp;
    ptr += 4;
    temp = 0;

    memcpy(&temp, ptr, 4);
    RTC_BKP1 = temp;
    ptr += 4;
    temp = 0;

    memcpy(&temp, ptr, 4);
    RTC_BKP2 = temp;
    ptr += 4;
    temp = 0;

    memcpy(&temp, ptr, 4);
    RTC_BKP3 = temp;
    ptr += 4;
    temp = 0;

    memcpy(&temp, ptr, 2);
    RTC_BKP4 = temp;
}

#endif
