#include "blob_nvm_storage.hpp"

static constexpr uint32_t KVDB_SECTOR_SIZE = 8192; // Max size of a single KV value

fdb_kvdb blob_kvdb{};
SemaphoreHandle_t mutex;
static void lock(fdb_db_t db)
{
    (void)db;
    if(!xPortIsInsideInterrupt()) xSemaphoreTake(mutex, portMAX_DELAY);
}
static void unlock(fdb_db_t db)
{
    (void)db;
    if(!xPortIsInsideInterrupt()) xSemaphoreGive(mutex);
}
static fdb_err_t kvdb_init()
{
    static fdb_err_t result = FDB_INIT_FAILED;
    static bool is_init_called = false;
    if(is_init_called) return result;
    mutex = xSemaphoreCreateMutex();
    uint32_t temp = KVDB_SECTOR_SIZE;
    iFOC::HAL::NVM::fdb_kvdb_control(&blob_kvdb, FDB_KVDB_CTRL_SET_LOCK, (void*)lock);
    iFOC::HAL::NVM::fdb_kvdb_control(&blob_kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void*)unlock);
    iFOC::HAL::NVM::fdb_kvdb_control(&blob_kvdb, FDB_KVDB_CTRL_SET_SEC_SIZE, &temp);
    result = iFOC::HAL::NVM::fdb_kvdb_init(&blob_kvdb, "c", "s", nullptr, nullptr);
    is_init_called = true;
    return result;
}
static bool get_kv_used_cb(const fdb_kv_t kv, void *arg1, void *arg2)
{
    (void)arg2;
    auto *using_size = static_cast<size_t *>(arg1);
    if(kv->crc_is_ok)
    {
        *using_size += kv->len;
    }
    return false;
}
static size_t get_kvdb_used_size()
{
    if(!blob_kvdb.parent.init_ok) return 0;
    size_t ret = 0;
    fdb_kv kv{};
    if(blob_kvdb.parent.lock) blob_kvdb.parent.lock(&blob_kvdb.parent);
    iFOC::HAL::NVM::kv_iterator(&blob_kvdb, &kv, &ret, &blob_kvdb, get_kv_used_cb);
    if(blob_kvdb.parent.unlock) blob_kvdb.parent.unlock(&blob_kvdb.parent);
    return ret;
}
static size_t get_kvdb_total_size()
{
    if(!blob_kvdb.parent.init_ok) return 0;
    // return blob_kvdb.parent.max_size - blob_kvdb.parent.sec_size;
    return blob_kvdb.parent.max_size;
}

namespace iFOC::DataType
{
size_t BlobNVMStorage::GetNVMUsedSize()
{
    return get_kvdb_used_size();
}

size_t BlobNVMStorage::GetNVMTotalSize()
{
    return get_kvdb_total_size();
}

FuncRetCode BlobNVMStorage::ReadNVM(const char* key, uint8_t* buffer, uint16_t* in_out_buf_len)
{
    const auto init_ret = kvdb_init();
    if(init_ret != FDB_NO_ERR) return FuncRetCode::HARDWARE_ERROR;
    fdb_blob blob
    {
        .buf = buffer,
        .size = *in_out_buf_len,
        .saved = {}
    };
    HAL::NVM::fdb_kv_get_blob(&blob_kvdb, key, &blob);
    *in_out_buf_len = 0;
    if(blob.saved.len > 0)
    {
        *in_out_buf_len = blob.saved.len;
        return FuncRetCode::OK;
    }
    return FuncRetCode::PARAM_NOT_EXIST;
}

FuncRetCode BlobNVMStorage::SaveNVM(const char* key, uint8_t* buffer, uint16_t buf_len)
{
    auto init_ret = kvdb_init();
    if(init_ret != FDB_NO_ERR) return FuncRetCode::HARDWARE_ERROR;
    fdb_blob blob
    {
        .buf = buffer,
        .size = buf_len,
        .saved = {}
    };
    init_ret = HAL::NVM::fdb_kv_set_blob(&blob_kvdb, key, &blob);
    if(init_ret != FDB_NO_ERR) return FuncRetCode::ACCESS_VIOLATION;
    return FuncRetCode::OK;
}

FuncRetCode BlobNVMStorage::ClearNVM(const char* key)
{
    auto init_ret = kvdb_init();
    if(init_ret != FDB_NO_ERR) return FuncRetCode::HARDWARE_ERROR;
    init_ret = HAL::NVM::fdb_kv_del(&blob_kvdb, key);
    if(init_ret != FDB_NO_ERR) return FuncRetCode::INVALID_RESULT;
    return FuncRetCode::OK;
}

uint16_t BlobNVMStorage::GetKVSize(const char* key)
{
    if(kvdb_init() != FDB_NO_ERR) return 0;
    return HAL::NVM::fdb_kv_get_blob_len(&blob_kvdb, key);
}
}

namespace iFOC
{
DataType::BlobNVMStorage& BlobNVMStorage()
{
    static DataType::BlobNVMStorage storage{};
    return storage;
}
}
