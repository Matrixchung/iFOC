#include <cstddef>
#include <cstdint>
#include <new>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
#include "portable.h"
#endif
#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook(void);
#endif
}

static inline void freertos_malloc_fail() {
#if (configUSE_MALLOC_FAILED_HOOK == 1)
    vApplicationMallocFailedHook();
#endif
    taskDISABLE_INTERRUPTS();
    for(;;) { }
}

static inline void* freertos_malloc(std::size_t size) {
    void* p = pvPortMalloc(size);
    if(p == nullptr) {
        freertos_malloc_fail();
    }
    return p;
}

static inline void freertos_free(void* ptr) {
    if(ptr) vPortFree(ptr);
}

// 1) standard scalar new/delete
void* operator new(std::size_t size) {
    return freertos_malloc(size);
}

void operator delete(void* ptr) noexcept {
    freertos_free(ptr);
}

// 2) standard array new[]/delete[]
void* operator new[](std::size_t size) {
    return freertos_malloc(size);
}

void operator delete[](void* ptr) noexcept {
    freertos_free(ptr);
}

// 3) nothrow
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    return pvPortMalloc(size);
}

void operator delete(void* ptr, const std::nothrow_t&) noexcept {
    freertos_free(ptr);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    return pvPortMalloc(size);
}

void operator delete[](void* ptr, const std::nothrow_t&) noexcept {
    freertos_free(ptr);
}

// 4) C++14 sized delete
#if defined(__cpp_sized_deallocation) && (__cpp_sized_deallocation >= 201309)
void operator delete(void* ptr, std::size_t) noexcept {
    freertos_free(ptr);
}
void operator delete[](void* ptr, std::size_t) noexcept {
    freertos_free(ptr);
}
#endif

// 5) C++17 aligned new/delete（with nothrow & array）
#if defined(__cpp_aligned_new) && (__cpp_aligned_new >= 201606)
static inline void* freertos_aligned_malloc(std::size_t size, std::size_t alignment) {
#if defined(portBYTE_ALIGNMENT)
    if(alignment <= portBYTE_ALIGNMENT) {
        return freertos_malloc(size);
    }
#endif
    const std::size_t total = size + alignment + sizeof(void*);
    void* raw = pvPortMalloc(total);
    if(raw == nullptr) {
        freertos_malloc_fail();
    }
    uintptr_t raw_addr = (uintptr_t)raw + sizeof(void*);
    uintptr_t aligned_addr = (raw_addr + (alignment - 1)) & ~(alignment - 1);
    void** store = (void**)(aligned_addr - sizeof(void*));
    *store = raw;
    return (void*)aligned_addr;
}

static inline void freertos_aligned_free(void* ptr, std::size_t alignment) noexcept {
#if defined(portBYTE_ALIGNMENT)
    if(ptr == nullptr) return;
    if(alignment <= portBYTE_ALIGNMENT) {
        vPortFree(ptr);
        return;
    }
#endif
    if(ptr == nullptr) return;
    void** store = (void**)((uintptr_t)ptr - sizeof(void*));
    vPortFree(*store);
}

// scalar alignment
void* operator new(std::size_t size, std::align_val_t al) {
    return freertos_aligned_malloc(size, static_cast<std::size_t>(al));
}
void operator delete(void* ptr, std::align_val_t al) noexcept {
    freertos_aligned_free(ptr, static_cast<std::size_t>(al));
}

// array alignment
void* operator new[](std::size_t size, std::align_val_t al) {
    return freertos_aligned_malloc(size, static_cast<std::size_t>(al));
}
void operator delete[](void* ptr, std::align_val_t al) noexcept {
    freertos_aligned_free(ptr, static_cast<std::size_t>(al));
}

// nothrow alignment
void* operator new(std::size_t size, std::align_val_t al, const std::nothrow_t&) noexcept {
#if defined(portBYTE_ALIGNMENT)
    if(static_cast<std::size_t>(al) <= portBYTE_ALIGNMENT) {
        return pvPortMalloc(size);
    }
#endif
    const std::size_t alignment = static_cast<std::size_t>(al);
    const std::size_t total = size + alignment + sizeof(void*);
    void* raw = pvPortMalloc(total);
    if(raw == nullptr) return nullptr;
    uintptr_t raw_addr = (uintptr_t)raw + sizeof(void*);
    uintptr_t aligned_addr = (raw_addr + (alignment - 1)) & ~(alignment - 1);
    void** store = (void**)(aligned_addr - sizeof(void*));
    *store = raw;
    return (void*)aligned_addr;
}
void operator delete(void* ptr, std::align_val_t al, const std::nothrow_t&) noexcept {
    freertos_aligned_free(ptr, static_cast<std::size_t>(al));
}
void* operator new[](std::size_t size, std::align_val_t al, const std::nothrow_t&) noexcept {
#if defined(portBYTE_ALIGNMENT)
    if(static_cast<std::size_t>(al) <= portBYTE_ALIGNMENT) {
        return pvPortMalloc(size);
    }
#endif
    const std::size_t alignment = static_cast<std::size_t>(al);
    const std::size_t total = size + alignment + sizeof(void*);
    void* raw = pvPortMalloc(total);
    if(raw == nullptr) return nullptr;
    uintptr_t raw_addr = (uintptr_t)raw + sizeof(void*);
    uintptr_t aligned_addr = (raw_addr + (alignment - 1)) & ~(alignment - 1);
    void** store = (void**)(aligned_addr - sizeof(void*));
    *store = raw;
    return (void*)aligned_addr;
}
void operator delete[](void* ptr, std::align_val_t al, const std::nothrow_t&) noexcept {
    freertos_aligned_free(ptr, static_cast<std::size_t>(al));
}

// C++17 sized aligned delete
#if defined(__cpp_sized_deallocation) && (__cpp_sized_deallocation >= 201309)
void operator delete(void* ptr, std::size_t, std::align_val_t al) noexcept {
    freertos_aligned_free(ptr, static_cast<std::size_t>(al));
}
void operator delete[](void* ptr, std::size_t, std::align_val_t al) noexcept {
    freertos_aligned_free(ptr, static_cast<std::size_t>(al));
}
#endif
#endif // __cpp_aligned_new