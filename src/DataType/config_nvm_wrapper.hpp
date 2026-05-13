#pragma once

#include "proto_wrapper.hpp"
#include "blob_nvm_storage.hpp"

namespace iFOC::DataType
{
template<NanopbMessage msg_t>
class ConfigNVMWrapper
{
    OVERRIDE_NEW();
private:
    ProtoWrapper<msg_t>* wrapper = nullptr;
    ProtoHeader header;
#if defined(USE_EASYFLASH) || defined(USE_FLASHDB)
    char db_key[2]{};
public:
    static size_t GetNVMUsedSize() { return BlobNVMStorage().GetNVMUsedSize(); }
    static size_t GetNVMTotalSize() { return BlobNVMStorage().GetNVMTotalSize(); }
#else
    uint8_t nvm_sector;
#endif
public:
#if defined(USE_EASYFLASH) || defined(USE_FLASHDB)
    ConfigNVMWrapper(ProtoHeader h, uint8_t s) : header(h)
#else
    ConfigNVMWrapper(ProtoHeader h, uint8_t s) : header(h), nvm_sector(s)
#endif
    {
        wrapper = new ProtoWrapper<msg_t>(header);
#if defined(USE_EASYFLASH) || defined(USE_FLASHDB)
        // Use (uint16_t)header as key
        db_key[0] = (char)(to_underlying(header));
        db_key[1] = (char)(to_underlying(header) >> 8);
#endif
    };
    msg_t& GetConfig();
    FuncRetCode ReadNVMConfig();
    FuncRetCode SaveNVMConfig();
    FuncRetCode ClearNVMConfig();
    uint8_t* GetBuffer() { return wrapper->GetBuffer(); };
    decltype(wrapper) GetWrapper() { return wrapper; }
};

template<NanopbMessage msg_t>
msg_t& ConfigNVMWrapper<msg_t>::GetConfig()
{
    return wrapper->payload();
}

template<NanopbMessage msg_t>
FuncRetCode ConfigNVMWrapper<msg_t>::ReadNVMConfig()
{
    // if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
    uint16_t buffer_in_out_size = wrapper->GetBufferSize();
#if defined(USE_EASYFLASH)
    auto ret = FuncRetCode::NOT_SUPPORTED;
#elif defined(USE_FLASHDB)
    auto ret = FuncRetCode::PARAM_NOT_EXIST;
    ret = BlobNVMStorage().ReadNVM(db_key, wrapper->GetBuffer(), &buffer_in_out_size);
#else
    // calculate sector address
    uint32_t addr = FLASH_USER_START_ADDR + nvm_sector * FLASH_SECTOR_SIZE_BYTES;
    auto ret = HAL::NVM::Read(addr,
                            wrapper->GetBuffer(),
                            ALIGN_TO(wrapper->GetBufferSize(), _const::NVM_ALIGN_BYTES));
#endif
    if(ret != FuncRetCode::OK) return ret;
    auto result = wrapper->Deserialize(buffer_in_out_size);
    if(result == FuncRetCode::OK) return FuncRetCode::OK;
    if(result == FuncRetCode::BUFFER_FULL) return FuncRetCode::BUFFER_FULL;
    if(result == FuncRetCode::CRC_MISMATCH) return FuncRetCode::CRC_MISMATCH;
    return FuncRetCode::INVALID_RESULT;
}

template<NanopbMessage msg_t>
FuncRetCode ConfigNVMWrapper<msg_t>::SaveNVMConfig()
{
    // if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
    size_t len = 0;
    auto result = wrapper->Serialize(len);
    if(result == FuncRetCode::OK)
    {
#if defined(USE_EASYFLASH)

#elif defined(USE_FLASHDB)
        return BlobNVMStorage().SaveNVM(db_key, wrapper->GetBuffer(), len);
#else
        return Write(nvm_sector,
                     wrapper->GetBuffer(),
                     ALIGN_TO(len, _const::NVM_ALIGN_BYTES));
#endif
    }
    if(result == FuncRetCode::BUFFER_FULL) return FuncRetCode::BUFFER_FULL;
    return FuncRetCode::INVALID_INPUT;
}

template<NanopbMessage msg_t>
FuncRetCode ConfigNVMWrapper<msg_t>::ClearNVMConfig()
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
#if defined(USE_EASYFLASH)
    return FuncRetCode::NOT_SUPPORTED;
#elif defined(USE_FLASHDB)
    return BlobNVMStorage().ClearNVM(db_key);
#else
    uint32_t addr = FLASH_USER_START_ADDR + nvm_sector * FLASH_SECTOR_SIZE_BYTES;
    return HAL::NVM::Erase(addr, FLASH_SECTOR_SIZE_BYTES);
#endif
}
}