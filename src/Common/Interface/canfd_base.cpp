#include "canfd_base.hpp"

namespace iFOC::HAL
{
    void CANFDBase::ProcessIncomingMsg(const DataType::Comm::CANFDMessage& msg) const
    {
        for(const auto& cb : cb_list) if(cb(msg)) break;
    }

    void CANFDBase::RegisterRxHandler(const EventCallback& cb)
    {
        cb_list.push_back(cb);
    }

    DataType::Comm::canfd_dlc CANFDBase::GetDLCFromLength(const uint8_t data_length)
    {
        if(data_length > 64) return DataType::Comm::canfd_dlc::DLC_0_BYTES;
        constexpr static uint8_t DATA_LEN_TO_DLC[65] = {
            0,  1,  2,  3,  4,  5,  6,  7,  8, // 0~8, DLC_0-8_BYTES
            12, 12, 12, 12,                    // 9~12, DLC_12_BYTES
            16, 16, 16, 16,                    // 13~16, DLC_16_BYTES
            20, 20, 20, 20,                    // 17~20, DLC_20_BYTES
            24, 24, 24, 24,                    // 21~24, DLC_24_BYTES
            32, 32, 32, 32, 32, 32, 32, 32,    // 25~32, DLC_32_BYTES
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, // 33~48, DLC_48_BYTES
            64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64  // 49~64, DLC_64_BYTES
        };
        return static_cast<DataType::Comm::canfd_dlc>(DATA_LEN_TO_DLC[data_length]);
    }
}
