#pragma once

#include "proto_wrapper.hpp"
#include "../Common/Interface/nvm_storage_base.hpp"
#include "func_ret_code.h"
#include "foc_types.hpp"

namespace iFOC::DataType
{
namespace _const
{
    static constexpr uint8_t NVM_ALIGN_BYTES = 8;
}
template<ProtoMessage msg_t>
class ConfigNVMWrapper
{
    OVERRIDE_NEW();
private:
    DataType::ProtoWrapper<msg_t>* wrapper = nullptr;
    Base::ProtoHeader header;
    uint8_t nvm_sector;
public:
    ConfigNVMWrapper(Base::ProtoHeader h, uint8_t s) : header(h), nvm_sector(s)
    {
        wrapper = new DataType::ProtoWrapper<msg_t>(header);
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
    auto ret = HAL::NVM::Read(nvm_sector,
                            wrapper->GetBuffer(),
                            ALIGN_TO(wrapper->GetBufferSize(), _const::NVM_ALIGN_BYTES));
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
    size_t len = 0;
    auto result = wrapper->Serialize(len);
    if(result == EmbeddedProto::Error::NO_ERRORS)
    {
        return HAL::NVM::Write(nvm_sector,
                             wrapper->GetBuffer(),
                             ALIGN_TO(len, _const::NVM_ALIGN_BYTES));
    }
    if(result == EmbeddedProto::Error::BUFFER_FULL) return FuncRetCode::BUFFER_FULL;
    return FuncRetCode::INVALID_INPUT;
}

template<ProtoMessage msg_t>
FuncRetCode ConfigNVMWrapper<msg_t>::ClearNVMConfig()
{
    return HAL::NVM::EraseSector(nvm_sector);
}
}