#include "kfifo.hpp"

#define MEMALLOC(size) pvPortMalloc(size)
#define MEMFREE(ptr) vPortFree(ptr)
#define uint32_min(x, y) ({       \
    uint32_t _min1 = (x);         \
    uint32_t _min2 = (y);         \
    (void) (&_min1 == &_min2);    \
    _min1 < _min2 ? _min1 : _min2;})

namespace iFOC::DataType::Ringbuf
{
FuncRetCode kfifo_t::init(uint32_t s)
{
    if(s & (s - 1)) s = roundup_pow2(s);
    buffer = static_cast<uint8_t*>(MEMALLOC(s));
    if(!buffer) return FuncRetCode::BUFFER_FULL;
    size = s;
    in = 0;
    out = 0;
    return FuncRetCode::OK;
}

uint32_t kfifo_t::put(const uint8_t *p, uint32_t len)
{
    uint32_t l;
    len = uint32_min(len, (size - in + out));
    l = uint32_min(len, (size - (in & (size - 1))));
    memcpy(buffer + (in & (size - 1)), p, l);
    memcpy(buffer, p + l, len - l);
    in += len;
    return len;
}

uint32_t kfifo_t::peek(uint8_t *p, uint32_t len)
{
    uint32_t l;
    len = uint32_min(len, (in - out));
    l = uint32_min(len, (size - (out & (size - 1))));
    memcpy(p, buffer + (out & (size - 1)), l);
    memcpy(p + l, buffer, len - l);
    return len;
}

uint32_t kfifo_t::get(uint8_t *p, uint32_t len)
{
    len = peek(p, len);
    out += len;
    return len;
}

void kfifo_t::wipe_n(uint32_t len)
{
    out += len;
}

kfifo_t::~kfifo_t()
{
    MEMFREE(buffer);
}

}
