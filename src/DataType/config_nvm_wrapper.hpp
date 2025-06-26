#pragma once

#include "proto_wrapper.hpp"
#include "hal_impl.hpp"
#include "func_ret_code.h"
#include "foc_types.hpp"

#if defined (USE_EASYFLASH)
#include "easyflash.h"
#elif defined (USE_FLASHDB)
#include "flashdb.h"
#endif

namespace iFOC::DataType
{
#if !defined(USE_EASYFLASH) && !defined(USE_FLASHDB)
namespace _const
{
    static constexpr uint8_t NVM_ALIGN_BYTES = FLASH_WRITE_GRAN_BITS / 8;
}
static FuncRetCode Write(uint8_t sector, const uint8_t *buffer, uint16_t size)
{
    uint32_t addr = FLASH_USER_START_ADDR + sector * FLASH_SECTOR_SIZE_BYTES;
    auto ret = iFOC::HAL::NVM::Erase(addr, size);
    if(ret != FuncRetCode::OK) return ret;
    return iFOC::HAL::NVM::Write_NoErase(addr, buffer, size);
}
#elif defined(USE_FLASHDB)
namespace _internal
{
    extern fdb_kvdb config_kvdb;
    fdb_err_t kvdb_init();
    size_t get_kvdb_used_size();
    size_t get_kvdb_total_size();
}
#endif

template<ProtoMessage msg_t>
class ConfigNVMWrapper
{
    OVERRIDE_NEW();
private:
    DataType::ProtoWrapper<msg_t>* wrapper = nullptr;
    Base::ProtoHeader header;
    uint8_t nvm_sector;
#if defined(USE_EASYFLASH) || defined(USE_FLASHDB)
    char db_key[2]{};
public:
    static size_t GetNVMUsedSize();
    static size_t GetNVMTotalSize();
#endif
public:
    ConfigNVMWrapper(Base::ProtoHeader h, uint8_t s) : header(h), nvm_sector(s)
    {
        wrapper = new DataType::ProtoWrapper<msg_t>(header);
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

template<ProtoMessage msg_t>
msg_t& ConfigNVMWrapper<msg_t>::GetConfig()
{
    return wrapper->payload();
}

template<ProtoMessage msg_t>
FuncRetCode ConfigNVMWrapper<msg_t>::ReadNVMConfig()
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
#if defined(USE_EASYFLASH)
    auto ret = FuncRetCode::NOT_SUPPORTED;
#elif defined(USE_FLASHDB)
    auto ret = FuncRetCode::PARAM_NOT_EXIST;
    auto flashdb_ret = _internal::kvdb_init();
    if(flashdb_ret != fdb_err_t::FDB_NO_ERR) return FuncRetCode::INVALID_RESULT;
    fdb_blob blob{};
    fdb_kv_get_blob(&_internal::config_kvdb, db_key, fdb_blob_make(&blob, wrapper->GetBuffer(), wrapper->GetBufferSize()));
    if(blob.saved.len > 0) ret = FuncRetCode::OK;
    else ret = FuncRetCode::PARAM_NOT_EXIST;
#else
    // calculate sector address
    uint32_t addr = FLASH_USER_START_ADDR + nvm_sector * FLASH_SECTOR_SIZE_BYTES;
    auto ret = HAL::NVM::Read(addr,
                            wrapper->GetBuffer(),
                            ALIGN_TO(wrapper->GetBufferSize(), _const::NVM_ALIGN_BYTES));
#endif
    if(ret != FuncRetCode::OK) return ret;
    auto result = wrapper->Deserialize(wrapper->GetBufferSize());
    if(result == EmbeddedProto::Error::NO_ERRORS) return FuncRetCode::OK;
    if(result == EmbeddedProto::Error::END_OF_BUFFER) return FuncRetCode::BUFFER_FULL;
    if(result == EmbeddedProto::Error::INVALID_FIELD_ID) return FuncRetCode::CRC_MISMATCH;
    return FuncRetCode::INVALID_RESULT;
}

template<ProtoMessage msg_t>
FuncRetCode ConfigNVMWrapper<msg_t>::SaveNVMConfig()
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
    size_t len = 0;
    auto result = wrapper->Serialize(len);
    if(result == EmbeddedProto::Error::NO_ERRORS)
    {
#if defined(USE_EASYFLASH)

#elif defined(USE_FLASHDB)
        auto flashdb_ret = _internal::kvdb_init();
        if(flashdb_ret != fdb_err_t::FDB_NO_ERR) return FuncRetCode::INVALID_RESULT;
        fdb_blob blob{};
        flashdb_ret = fdb_kv_set_blob(&_internal::config_kvdb, db_key, fdb_blob_make(&blob, wrapper->GetBuffer(), len));
        if(flashdb_ret == fdb_err_t::FDB_NO_ERR) return FuncRetCode::OK;
        return FuncRetCode::HARDWARE_ERROR;
#else
        return Write(nvm_sector,
                     wrapper->GetBuffer(),
                     ALIGN_TO(len, _const::NVM_ALIGN_BYTES));
#endif
    }
    if(result == EmbeddedProto::Error::BUFFER_FULL) return FuncRetCode::BUFFER_FULL;
    return FuncRetCode::INVALID_INPUT;
}

template<ProtoMessage msg_t>
FuncRetCode ConfigNVMWrapper<msg_t>::ClearNVMConfig()
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
#if defined(USE_EASYFLASH)
    return FuncRetCode::NOT_SUPPORTED;
#elif defined(USE_FLASHDB)
    auto flashdb_ret = _internal::kvdb_init();
    if(flashdb_ret != fdb_err_t::FDB_NO_ERR) return FuncRetCode::HARDWARE_ERROR;
    flashdb_ret = fdb_kv_del(&_internal::config_kvdb, db_key);
    if(flashdb_ret != fdb_err_t::FDB_NO_ERR) return FuncRetCode::INVALID_RESULT;
    return FuncRetCode::OK;
#else
    uint32_t addr = FLASH_USER_START_ADDR + nvm_sector * FLASH_SECTOR_SIZE_BYTES;
    return HAL::NVM::Erase(addr, FLASH_SECTOR_SIZE_BYTES);
#endif
}
}