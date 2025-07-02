#pragma once

#include "foc_types.hpp"
#include "math_concepts.hpp"
#include <cstring>
#include <map>

namespace iFOC::Reflection
{
struct CharPtrHash
{
    size_t operator()(const char* s) const
    {
        return std::hash<std::string_view>{}(s);
    }
};

struct CharPtrEqual
{
    bool operator()(const char* a, const char* b) const
    {
        return strcmp(a, b) == 0;
    }
};

struct CharPtrCompare
{
    bool operator()(const char* a, const char* b) const
    {
        return strcmp(a, b) < 0;
    }
};

enum class ProtoFieldType : uint8_t
{
    DOUBLE = 0,
    FLOAT = 1,
    INT32 = 2,
    INT64 = 3,
    UINT32 = 4,
    UINT64 = 5,
    BOOL = 6,
    UNKNOWN = 7
};

template<typename T>
constexpr ProtoFieldType GetFieldType()
{
    if constexpr (floating_double<T>) return ProtoFieldType::DOUBLE;
    else if constexpr (floating_float<T>) return ProtoFieldType::FLOAT;
    else if constexpr (integral_s32<T>) return ProtoFieldType::INT32;
    else if constexpr (integral_s64<T>) return ProtoFieldType::INT64;
    else if constexpr (integral_u32<T>) return ProtoFieldType::UINT32;
    else if constexpr (integral_u64<T>) return ProtoFieldType::UINT64;
    else if constexpr (integral_8<T>) return ProtoFieldType::BOOL;
    return ProtoFieldType::UNKNOWN;
}

constexpr size_t GetFieldSize(const ProtoFieldType field)
{
    switch(field)
    {
        case ProtoFieldType::DOUBLE:
        case ProtoFieldType::INT64:
        case ProtoFieldType::UINT64:
            return 8;
        case ProtoFieldType::FLOAT:
        case ProtoFieldType::INT32:
        case ProtoFieldType::UINT32:
            return 4;
        case ProtoFieldType::BOOL:
            return 1;
        default: return 0;
    }
}

constexpr const char* GetFieldName(const ProtoFieldType field)
{
    switch(field)
    {
        case ProtoFieldType::DOUBLE: return "double";
        case ProtoFieldType::FLOAT: return "float";
        case ProtoFieldType::INT32: return "int32";
        case ProtoFieldType::INT64: return "int64";
        case ProtoFieldType::UINT32: return "uint32";
        case ProtoFieldType::UINT64: return "uint64";
        case ProtoFieldType::BOOL: return "bool";
        default: return "unknown";
    }
}

template<typename T>
concept HasFieldType = requires { typename T::TYPE; };

template<typename T>
struct ExtractFieldType
{
    using raw_type = typename T::TYPE;
    using type = std::conditional_t<
            std::is_enum_v<raw_type>,
            std::underlying_type_t<raw_type>,
            raw_type
    >;
};

template<typename T>
requires (!HasFieldType<T>)
struct ExtractFieldType<T>
{
    using type = std::conditional_t<
            std::is_enum_v<T>,
            std::underlying_type_t<T>,
            T
    >;
};

template<typename T>
requires (HasFieldType<T> && !std::is_enum_v<typename T::TYPE>)
struct ExtractFieldType<T>
{
    using type = typename T::TYPE;
};
}

namespace iFOC
{
using MemberInfo = std::pair<Reflection::ProtoFieldType, size_t>; // {ProtoFieldType, offsetof()}
// using ReflectMap = std::unordered_map<
//         const char*,
//         MemberInfo,
//         Reflection::CharPtrHash,
//         Reflection::CharPtrEqual,
//         Allocator<std::pair<const char* const, MemberInfo>>
// >;
using ReflectMap = std::map<
        const char*,
        MemberInfo,
        Reflection::CharPtrCompare,
        Allocator<std::pair<const char* const, MemberInfo>> // stored in memory? is it necessary?
>;
}

#define REFLECT(...) \
    static iFOC::ReflectMap GetReflectMap() { \
        static const iFOC::ReflectMap members{ __VA_ARGS__ }; \
        return members;                                \
    }

#define MEMBER_SIZE_OFFSET(class, name) {#name, {iFOC::Reflection::GetFieldType< \
            iFOC::Reflection::ExtractFieldType<decltype(class::name)>::type >(), \
            offsetof(class, name)}}