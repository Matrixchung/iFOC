#include "at32wk_bkp.hpp"
#include "at32wk_bkp_struct.h"
#include "../../Common/foc_math.hpp"

#if defined(AT32WK_ENV) && defined(PWC_MODULE_ENABLED) && defined(CRM_MODULE_ENABLED)

#include "../../DataType/board_config.hpp"

/* Using backup register to exchange information between App & Bootloader
 * See: RM_AT32F403A_407_407A_CH_V2.07, Page 285 (BPR)
 * AT32F403A BPR size: 42 * 2 = 84 bytes
 */

// automatically handle crc stuffs
bool read_bpr(bkp_struct_t* dst);
void write_bpr(bkp_struct_t* src);

namespace iFOC::HAL::Bootloader
{
    void JumpToBL(const BootloaderMsg& msg)
    {
        static_assert(sizeof(bkp_struct_t::dronecan_image_path) == sizeof(BootloaderMsg::update_image_path));
        bkp_struct_t bkp{};
        if(!read_bpr(&bkp)) return;
        if(!bkp.flags.bit.bootloader_presented) return;
        bkp.flags.bit.update_requested = true;
        bkp.flags.bit.app_init_success = true;
        bkp.flags.bit.controllable_hardfault = false;
        bkp.flags.bit.enable_can_resistor = BoardConfig().GetConfig().enable_can_terminal_resistor();
        bkp.file_server_node_id = msg.server_node_id;
        memcpy(bkp.dronecan_image_path, msg.update_image_path, sizeof(bkp.dronecan_image_path));
        write_bpr(&bkp);
        // do nvic_systemreboot()
        SystemReboot();
    }

    bool HasBL()
    {
        bkp_struct_t bkp{};
        if(!read_bpr(&bkp)) return false;
        if(!bkp.flags.bit.bootloader_presented) return false;
        return true;
    }

    void GetBLVersion(uint8_t& major, uint8_t& minor, uint32_t& vcs)
    {
        major = 0;
        minor = 0;
        vcs = 0;
        bkp_struct_t bkp{};
        if(!read_bpr(&bkp)) return;
        if(!bkp.flags.bit.bootloader_presented) return;
        major = bkp.version.major;
        minor = bkp.version.minor;
        memcpy(&vcs, bkp.version.vcs_commit, sizeof(bkp.version.vcs_commit));
    }

    void SetAppInitSuccessFlag()
    {
        bkp_struct_t bkp{};
        if(!read_bpr(&bkp)) return;
        bkp.flags.bit.update_requested = false;
        bkp.flags.bit.app_init_success = true;
        write_bpr(&bkp);
    }

    void OnHardFault()
    {
        bkp_struct_t bkp{};
        if(!read_bpr(&bkp)) return;
        bkp.flags.bit.controllable_hardfault = true;
        if(bkp.controllable_hardfault_count < 255) bkp.controllable_hardfault_count++; // prevent overflow
        write_bpr(&bkp);
    }
#ifdef AT32WK_TEST_BKP
    void TestBKP(bkp_struct_t* bkp)
    {
        read_bpr(bkp);
        bkp->reserved++;
        write_bpr(bkp);
        // read back
        read_bpr(bkp);
    }
#endif
}

// bkp_struct_t size: 18, BPR_DATA1 -> BPR_DATA9, little-endian
bool read_bpr(bkp_struct_t* dst)
{
    static_assert(sizeof(bkp_struct_t) == 18);

    memset(dst, 0, sizeof(bkp_struct_t));
    uint8_t buffer[sizeof(bkp_struct_t)];
    volatile uint32_t temp = 0;

    temp = bpr_data_read(BPR_DATA1);
    buffer[0] = (uint8_t)(temp & 0xFF);
    buffer[1] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA2);
    buffer[2] = (uint8_t)(temp & 0xFF);
    buffer[3] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA3);
    buffer[4] = (uint8_t)(temp & 0xFF);
    buffer[5] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA4);
    buffer[6] = (uint8_t)(temp & 0xFF);
    buffer[7] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA5);
    buffer[8] = (uint8_t)(temp & 0xFF);
    buffer[9] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA6);
    buffer[10] = (uint8_t)(temp & 0xFF);
    buffer[11] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA7);
    buffer[12] = (uint8_t)(temp & 0xFF);
    buffer[13] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA8);
    buffer[14] = (uint8_t)(temp & 0xFF);
    buffer[15] = (uint8_t)(temp >> 8);
    temp = bpr_data_read(BPR_DATA9);
    buffer[16] = (uint8_t)(temp & 0xFF);
    buffer[17] = (uint8_t)(temp >> 8);

    // crc calculation
    const uint8_t calc_crc8 = iFOC::get_crc8(buffer, sizeof(buffer) - 1);
    if(calc_crc8 == buffer[sizeof(buffer) - 1]) // crc8 matched
    {
        memcpy(dst, buffer, sizeof(bkp_struct_t));
        return true;
    }
    return false;
}

void write_bpr(bkp_struct_t* src)
{
    static_assert(sizeof(bkp_struct_t) == 18);

    volatile uint32_t temp = 0;

    const uint8_t* ptr = (const uint8_t*)src;

    src->crc8 = iFOC::get_crc8(ptr, sizeof(bkp_struct_t) - 1);

    // Unlock BPR first
    CRM->apb1en_bit.bpren = 1;
    CRM->apb1en_bit.pwcen = 1;
    PWC->ctrl_bit.bpwen = 1;

    temp = (uint32_t)((uint8_t)*(ptr + 0)) | (uint32_t)((uint8_t)*(ptr + 1)) << 8;
    bpr_data_write(BPR_DATA1, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 2)) | (uint32_t)((uint8_t)*(ptr + 3)) << 8;
    bpr_data_write(BPR_DATA2, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 4)) | (uint32_t)((uint8_t)*(ptr + 5)) << 8;
    bpr_data_write(BPR_DATA3, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 6)) | (uint32_t)((uint8_t)*(ptr + 7)) << 8;
    bpr_data_write(BPR_DATA4, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 8)) | (uint32_t)((uint8_t)*(ptr + 9)) << 8;
    bpr_data_write(BPR_DATA5, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 10)) | (uint32_t)((uint8_t)*(ptr + 11)) << 8;
    bpr_data_write(BPR_DATA6, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 12)) | (uint32_t)((uint8_t)*(ptr + 13)) << 8;
    bpr_data_write(BPR_DATA7, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 14)) | (uint32_t)((uint8_t)*(ptr + 15)) << 8;
    bpr_data_write(BPR_DATA8, temp);

    temp = (uint32_t)((uint8_t)*(ptr + 16)) | (uint32_t)((uint8_t)*(ptr + 17)) << 8;
    bpr_data_write(BPR_DATA9, temp);

    // PWC->ctrl_bit.bpwen = 0; // lock BPR // lock BPR cause lock of RTC
}

#endif