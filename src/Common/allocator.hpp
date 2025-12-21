#pragma once

#include <deque>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>

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
        configASSERT(ptr != nullptr);
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
using Queue = std::queue<T, Allocator<T>>;

template<typename T>
using Deque = std::deque<T, Allocator<T>>;

template<typename Key, typename Value, typename Compare = std::less<Key>>
using Map = std::map<Key, Value, Compare, Allocator<std::pair<const Key, Value>>>;

template<typename Key, typename Value, typename Hash = std::hash<Key>, typename Pred = std::equal_to<Key>>
using HashMap = std::unordered_map<Key, Value, Hash, Pred, Allocator<std::pair<const Key, Value>>>;

template<typename T, typename Hash = std::hash<T>, typename Pred = std::equal_to<T>>
using HashSet = std::unordered_set<T, Hash, Pred, Allocator<T>>;

}