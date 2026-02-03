#pragma once

#include "kfifo.hpp"

namespace iFOC::DataType::Ringbuf
{
template<typename T> requires (sizeof(T) >= 1)
struct obj_kfifo_t
{
    DELETE_COPY_CONSTRUCTOR(obj_kfifo_t);
    OVERRIDE_NEW();
public:
    obj_kfifo_t() = default;
    FuncRetCode init(uint32_t s);
    uint32_t put(const T* p, uint32_t s);
    uint32_t peek(T* p, uint32_t s);
    uint32_t get(T* p, uint32_t s);
    void wipe_n(uint32_t len);
    [[nodiscard]] uint32_t used() const;
    [[nodiscard]] uint32_t available() const;
    void flush();
private:
    kfifo_t kfifo;
};

template <typename T> requires (sizeof(T) >= 1)
FuncRetCode obj_kfifo_t<T>::init(uint32_t s)
{
    return kfifo.init(s * sizeof(T));
}

template <typename T> requires (sizeof(T) >= 1)
uint32_t obj_kfifo_t<T>::put(const T* p, uint32_t s)
{
    return kfifo.put((const uint8_t*)p, s * sizeof(T)) / sizeof(T);
}

template <typename T> requires (sizeof(T) >= 1)
uint32_t obj_kfifo_t<T>::peek(T* p, uint32_t s)
{
    return kfifo.peek((uint8_t*)p, s * sizeof(T)) / sizeof(T);
}

template <typename T> requires (sizeof(T) >= 1)
uint32_t obj_kfifo_t<T>::get(T* p, uint32_t s)
{
    return kfifo.get((uint8_t*)p, s * sizeof(T)) / sizeof(T);
}

template <typename T> requires (sizeof(T) >= 1)
void obj_kfifo_t<T>::wipe_n(uint32_t len)
{
    kfifo.wipe_n(len * sizeof(T));
}

template <typename T> requires (sizeof(T) >= 1)
uint32_t obj_kfifo_t<T>::used() const
{
    return kfifo.used() / sizeof(T);
}

template <typename T> requires (sizeof(T) >= 1)
uint32_t obj_kfifo_t<T>::available() const
{
    return kfifo.available() / sizeof(T);
}

template <typename T> requires (sizeof(T) >= 1)
void obj_kfifo_t<T>::flush()
{
    kfifo.flush();
}
}
