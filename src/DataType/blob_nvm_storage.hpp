#pragma once

#include "../HAL/hal_impl.hpp"

#if defined(USE_FLASHDB)
#include "../ThirdParty/FlashDB/inc/flashdb.h"
#endif

namespace iFOC::DataType
{
class BlobNVMStorage
{
    OVERRIDE_NEW();
public:
    static size_t GetNVMUsedSize();
    static size_t GetNVMTotalSize();
    static FuncRetCode ReadNVM(const char* key, uint8_t* buffer, uint16_t* in_out_buf_len);
    static FuncRetCode SaveNVM(const char* key, uint8_t* buffer, uint16_t buf_len);
    static FuncRetCode ClearNVM(const char* key);
    static uint16_t GetKVSize(const char* key);
};
}

namespace iFOC
{
DataType::BlobNVMStorage& BlobNVMStorage();
}