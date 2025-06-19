#pragma once

#include <cstring>
#include <cstdint>
#include "foc_types.hpp"
#include "func_ret_code.h"
#include "foc_math.hpp"

namespace iFOC::DataType::Ringbuf
{
struct kfifo_t
{
    DELETE_COPY_CONSTRUCTOR(kfifo_t);
    OVERRIDE_NEW();
public:
    kfifo_t() = default;
    ~kfifo_t();
    FuncRetCode init(uint32_t s);
    uint32_t put(const uint8_t *p, uint32_t len);
    uint32_t peek(uint8_t *p, uint32_t len);
    uint32_t get(uint8_t *p, uint32_t len);
    void wipe_n(uint32_t len);
    __fast_inline uint32_t used() { return in - out; };
    __fast_inline uint32_t available() { return size - (in - out); };
    __fast_inline void flush() { in = out = 0; };
private:
    uint32_t size = 0;
    uint32_t in = 0;
    uint32_t out = 0;
    uint8_t *buffer = nullptr;
};

}