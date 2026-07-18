#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>
#include <cmath>

#include "Math/float16.hpp"
#include "Math/real_t.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "../DataType/Headers/Base/func_ret_code.h"
#include "allocator.hpp"

namespace iFOC
{
using namespace DataType::Base;
struct qd_t
{
    real_t q = 0;
    real_t d = 0;
};

struct alphabeta_t
{
    real_t alpha = 0;
    real_t beta = 0;
};

struct U {};
struct V {};
struct W {};

extern volatile float RT_LOOP_TS;
extern volatile float MID_LOOP_TS;
extern volatile uint8_t SYSTEM_MOTOR_NUM;

#define EXECUTE(func, ...) do{if(func) func(__VA_ARGS__);}while(0)

#define DELETE_COPY_CONSTRUCTOR(Class) \
public:                                \
    Class(const Class&) = delete;      \
    Class(Class&&) = delete;           \

#define MAKE_SINGLETON(Class) \
public: \
    static Class& Inst() { \
        static Class instance; \
        return instance; \
    } \
    Class(const Class&) = delete; \
    Class(Class&&) = delete; \
    void operator=(const Class&) = delete; \
private: \
    Class() = default;

#define OVERRIDE_NEW() \
public:                \
static void * operator new(std::size_t size) { \
     void* ptr = pvPortMalloc(size);           \
     configASSERT(ptr != nullptr);             \
     return ptr; \
}                      \
static void operator delete(void* ptr) noexcept\
{                      \
    vPortFree(ptr); \
}

#define xSemaphoreTakeAuto(xSemaphore, xBlockTime) (xPortIsInsideInterrupt() ? (xSemaphoreTakeFromISR((xSemaphore), nullptr)) : (xSemaphoreTake((xSemaphore), (xBlockTime))))
#define xSemaphoreGiveAuto(xSemaphore) (xPortIsInsideInterrupt() ? xSemaphoreGiveFromISR((xSemaphore), nullptr) : xSemaphoreGive((xSemaphore)))
}
