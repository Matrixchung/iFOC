#pragma once

#include <deque>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <unordered_map>

#include "portable.h"

/// \brief Custom memory allocator for STL containers using FreeRTOS Heap
/// \ref https://zhuanlan.zhihu.com/p/185611161

namespace iFOC
{
template<typename T>
class Allocator
{
public:
    static void * operator new(std::size_t size) {
        void* ptr = pvPortMalloc(size);
        memset(ptr, 0, size);
        return ptr;
    }
    static void operator delete(void* ptr) noexcept
    {
        vPortFree(ptr);
    }

    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    // using reference = T&;
    // using const_reference = const T&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template<typename U>
    struct rebind
    {
        using other = Allocator<U>;
    };

    Allocator() noexcept = default;
    template<typename U>
    explicit Allocator(const Allocator<U>&) noexcept {}

    T* allocate(std::size_t n)
    {
        return static_cast<T*>(operator new(n * sizeof(T)));
    }
    void deallocate(T* p, std::size_t n)
    {
        operator delete(p);
    }

    bool operator==(const Allocator&) const noexcept { return true; }
    bool operator!=(const Allocator& other) const noexcept { return !(*this == other); };
};

using String = std::basic_string<char, std::char_traits<char>, Allocator<char>>;

template<typename T>
using Vector = std::vector<T, Allocator<T>>;

template<typename T>
using List = std::list<T, Allocator<T>>;

template<typename T>
using Deque = std::deque<T, Allocator<T>>;

template<typename T1, typename T2>
using Map = std::map<T1, T2, std::less<T1>, Allocator<std::pair<const T1, T2>>>;

template<typename T1, typename T2>
using HashMap = std::unordered_map<T1, T2, std::hash<T1>, std::equal_to<T1>, Allocator<std::pair<const T1, T2>>>;

}