#pragma once

#include <type_traits>
#include <iterator>

template<typename T>
concept arithmetic = std::is_arithmetic_v<T>;

template<typename T>
concept floating = std::is_floating_point_v<T>;

template<typename T>
concept floating_float = floating<T> and sizeof(T) == 4;

template<typename T>
concept floating_double = floating<T> and sizeof(T) > 4;

template<typename T>
concept integral = std::is_integral_v<T>;

template<typename T>
concept uintegral = std::is_integral_v<T> and std::is_unsigned_v<T>;

template<typename T>
concept sintegral = std::is_integral_v<T> and std::is_signed_v<T>;

template<typename T>
concept integral_8 = std::is_integral_v<T> && sizeof(T) == 1;

template<typename T>
concept integral_s8 = integral_8<T> && std::is_signed_v<T>;

template<typename T>
concept integral_u8 = integral_8<T> && std::is_unsigned_v<T>;

template<typename T>
concept integral_16 = std::is_integral_v<T> && sizeof(T) == 2;

template<typename T>
concept integral_s16 = integral_16<T> && std::is_signed_v<T>;

template<typename T>
concept integral_u16 = integral_16<T> && std::is_unsigned_v<T>;

template<typename T>
concept integral_32 = std::is_integral_v<T> && sizeof(T) == 4;

template<typename T>
concept integral_s32 = integral_32<T> && std::is_signed_v<T>;

template<typename T>
concept integral_u32 = integral_32<T> && std::is_unsigned_v<T>;

template<typename T>
concept integral_64 = std::is_integral_v<T> && sizeof(T) == 8;

template<typename T>
concept integral_s64 = integral_64<T> && std::is_signed_v<T>;

template<typename T>
concept integral_u64 = integral_64<T> && std::is_unsigned_v<T>;

template<typename T>
concept iterable = requires(T t) {
    { std::next(t) } -> std::same_as<T>;
    { std::prev(t) } -> std::same_as<T>;
};

template<typename T>
concept is_stdlayout = std::is_standard_layout<T>::value;