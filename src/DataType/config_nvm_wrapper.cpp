#include "config_nvm_wrapper.hpp"

#if defined(USE_FLASHDB)

#include "semphr.h"

namespace iFOC::DataType::_internal
{
    fdb_kvdb config_kvdb{};
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    static void lock(fdb_db_t db)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
    }
    static void unlock(fdb_db_t db)
    {
        xSemaphoreGive(mutex);
    }
    fdb_err_t kvdb_init()
    {
        static fdb_err_t result = FDB_INIT_FAILED;
        static bool is_init_called = false;
        if(is_init_called) return result;
        HAL::NVM::fdb_kvdb_control(&config_kvdb, FDB_KVDB_CTRL_SET_LOCK, (void*)lock);
        HAL::NVM::fdb_kvdb_control(&config_kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void*)unlock);
        result = HAL::NVM::fdb_kvdb_init(&config_kvdb, "cfg", "s", nullptr, nullptr);
        is_init_called = true;
        return result;
    }
    static bool get_kv_used_cb(fdb_kv_t kv, void *arg1, void *arg2)
    {
        size_t *using_size = static_cast<size_t *>(arg1);
        if(kv->crc_is_ok)
        {
            *using_size += kv->len;
        }
        return false;
    }
    size_t get_kvdb_used_size()
    {
        if(!config_kvdb.parent.init_ok) return 0;
        size_t ret = 0;
        fdb_kv kv{};
        if(config_kvdb.parent.lock) config_kvdb.parent.lock(&config_kvdb.parent);
        HAL::NVM::kv_iterator(&config_kvdb, &kv, &ret, &config_kvdb, get_kv_used_cb);
        if(config_kvdb.parent.unlock) config_kvdb.parent.unlock(&config_kvdb.parent);
        return ret;
    }
    size_t get_kvdb_total_size()
    {
        if(!config_kvdb.parent.init_ok) return 0;
        return config_kvdb.parent.max_size - config_kvdb.parent.sec_size;
    }
}

#endif