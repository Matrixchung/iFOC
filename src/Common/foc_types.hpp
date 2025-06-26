#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <optional>

#include "Math/real_t.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "func_ret_code.h"
#include "allocator.hpp"
#include "../DataType/Headers/Base/misconfigured_area.h"

namespace iFOC
{
using namespace DataType::Base;
struct qd_t
{
    real_t q = 0;
    real_t d = 0;
};

// struct ab_t
// {
//     real_t a = 0;
//     real_t b = 0;
// };
//
// struct abc_t
// {
//     real_t a = 0;
//     real_t b = 0;
//     real_t c = 0;
// };

struct alphabeta_t
{
    real_t alpha = 0;
    real_t beta = 0;
};

extern std::underlying_type_t<DataType::Base::MisconfiguredArea> misconfigured_area;
extern float RT_LOOP_TS;
extern float MID_LOOP_TS;
extern uint8_t SYSTEM_MOTOR_NUM;

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
     memset(ptr, 0, size);                     \
     return ptr; \
}                      \
static void operator delete(void* ptr) noexcept\
{                      \
    vPortFree(ptr); \
}
}