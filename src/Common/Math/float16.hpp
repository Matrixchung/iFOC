#pragma once

#include <bit>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace iFOC
{
class float16 final
{
public:
    constexpr float16() noexcept = default;
    constexpr float16(const float value) noexcept : bits_(float32_to_float16_bits(value)) {}

    constexpr float16& operator=(const float value) noexcept
    {
        bits_ = float32_to_float16_bits(value);
        return *this;
    }

    static constexpr float16 from_bits(const uint16_t bits) noexcept
    {
        float16 value;
        value.bits_ = bits;
        return value;
    }

    static constexpr float16 from_float(float value) noexcept { return float16(value); }
    static constexpr float16 positive_zero() noexcept { return from_bits(0x0000U); }
    static constexpr float16 negative_zero() noexcept { return from_bits(0x8000U); }
    static constexpr float16 positive_infinity() noexcept { return from_bits(0x7C00U); }
    static constexpr float16 negative_infinity() noexcept { return from_bits(0xFC00U); }
    static constexpr bool native_conversion_available() noexcept { return has_native_float16_conversion(); }

    [[nodiscard]] constexpr uint16_t bits() const noexcept { return bits_; }
    [[nodiscard]] constexpr bool sign_bit() const noexcept { return (bits_ & 0x8000U) != 0; }
    [[nodiscard]] constexpr uint8_t exponent_bits() const noexcept { return (uint8_t)((bits_ >> 10) & 0x1FU); }
    [[nodiscard]] constexpr uint16_t mantissa_bits() const noexcept { return bits_ & 0x03FFU; }
    [[nodiscard]] constexpr bool is_zero() const noexcept { return (bits_ & 0x7FFFU) == 0; }
    [[nodiscard]] constexpr bool is_inf() const noexcept { return (bits_ & 0x7FFFU) == 0x7C00U; }
    [[nodiscard]] constexpr bool is_nan() const noexcept { return ((bits_ & 0x7C00U) == 0x7C00U) && ((bits_ & 0x03FFU) != 0); }
    [[nodiscard]] constexpr bool is_finite() const noexcept { return (bits_ & 0x7C00U) != 0x7C00U; }
    [[nodiscard]] constexpr bool bit_equal(const float16 other) const noexcept { return bits_ == other.bits_; }
    [[nodiscard]] constexpr float to_float() const noexcept { return float16_bits_to_float32(bits_); }

    explicit constexpr operator float() const noexcept
    {
        return to_float();
    }

    [[nodiscard]] constexpr float16 operator+() const noexcept
    {
        return *this;
    }

    [[nodiscard]] constexpr float16 operator-() const noexcept
    {
        return from_bits(bits_ ^ 0x8000U);
    }

    constexpr float16& operator+=(const float16 rhs) noexcept
    {
        *this = *this + rhs;
        return *this;
    }

    constexpr float16& operator-=(const float16 rhs) noexcept
    {
        *this = *this - rhs;
        return *this;
    }

    constexpr float16& operator*=(const float16 rhs) noexcept
    {
        *this = *this * rhs;
        return *this;
    }

    constexpr float16& operator/=(const float16 rhs) noexcept
    {
        *this = *this / rhs;
        return *this;
    }

    friend constexpr bool operator==(const float16 lhs, const float16 rhs) noexcept
    {
        if(lhs.is_nan() || rhs.is_nan()) return false;
        if(lhs.is_zero() && rhs.is_zero()) return true;
        return lhs.bits_ == rhs.bits_;
    }

    friend constexpr bool operator!=(const float16 lhs, const float16 rhs) noexcept
    {
        return !(lhs == rhs);
    }

    friend constexpr bool operator<(const float16 lhs, const float16 rhs) noexcept
    {
        return lhs.to_float() < rhs.to_float();
    }

    friend constexpr bool operator<=(const float16 lhs, const float16 rhs) noexcept
    {
        return lhs.to_float() <= rhs.to_float();
    }

    friend constexpr bool operator>(const float16 lhs, const float16 rhs) noexcept
    {
        return lhs.to_float() > rhs.to_float();
    }

    friend constexpr bool operator>=(const float16 lhs, const float16 rhs) noexcept
    {
        return lhs.to_float() >= rhs.to_float();
    }

    friend constexpr float16 operator+(const float16 lhs, const float16 rhs) noexcept
    {
        return float16(lhs.to_float() + rhs.to_float());
    }

    friend constexpr float16 operator-(const float16 lhs, const float16 rhs) noexcept
    {
        return float16(lhs.to_float() - rhs.to_float());
    }

    friend constexpr float16 operator*(const float16 lhs, const float16 rhs) noexcept
    {
        return float16(lhs.to_float() * rhs.to_float());
    }

    friend constexpr float16 operator/(const float16 lhs, const float16 rhs) noexcept
    {
        return float16(lhs.to_float() / rhs.to_float());
    }

private:
    static constexpr bool has_native_float16_conversion() noexcept
    {
#if (defined(__arm__) || defined(__aarch64__)) && defined(__ARM_FP16_FORMAT_IEEE) && defined(__ARM_FEATURE_FP16_SCALAR_ARITHMETIC)
        return true;
#else
        return false;
#endif
    }

#if (defined(__arm__) || defined(__aarch64__)) && defined(__ARM_FP16_FORMAT_IEEE) && defined(__ARM_FEATURE_FP16_SCALAR_ARITHMETIC)
    static uint16_t native_float32_to_float16_bits(const float value) noexcept
    {
        const __fp16 half = value;
        return std::bit_cast<uint16_t>(half);
    }

    static float native_float16_bits_to_float32(const uint16_t bits) noexcept
    {
        const __fp16 half = std::bit_cast<__fp16>(bits);
        return (float)half;
    }
#endif

    static constexpr uint32_t float32_to_bits(const float value) noexcept
    {
        return std::bit_cast<uint32_t>(value);
    }

    static constexpr float bits_to_float32(const uint32_t bits) noexcept
    {
        return std::bit_cast<float>(bits);
    }

    static constexpr uint32_t round_shift_right(const uint32_t value, const uint8_t shift) noexcept
    {
        if(shift == 0) return value;
        const uint32_t half = 1UL << (shift - 1);
        const uint32_t mask = (1UL << shift) - 1UL;
        const uint32_t retained = value >> shift;
        const uint32_t dropped = value & mask;
        return retained + (dropped > half || (dropped == half && (retained & 1UL)));
    }

    static constexpr uint32_t count_leading_zeros32(const uint32_t value) noexcept
    {
        if(value == 0) return 32;
#if defined(__GNUC__) || defined(__clang__)
        if(!std::is_constant_evaluated()) return (uint32_t)__builtin_clz(value);
#endif
        uint32_t count = 0;
        uint32_t mask = 0x80000000UL;
        while((value & mask) == 0)
        {
            count++;
            mask >>= 1;
        }
        return count;
    }

    static constexpr uint16_t float32_bits_to_float16_bits(const uint32_t bits) noexcept
    {
        const uint16_t sign = (uint16_t)((bits >> 16) & 0x8000U);
        const uint32_t exp = (bits >> 23) & 0xFFU;
        const uint32_t mant = bits & 0x7FFFFFU;

        if(exp == 0xFFU)
        {
            if(mant == 0) return sign | 0x7C00U;
            uint16_t payload = (uint16_t)(mant >> 13);
            payload |= 0x0200U; // quiet NaN
            return sign | 0x7C00U | payload;
        }

        if(exp == 0) return sign;

        const int32_t half_exp = (int32_t)exp - 127 + 15;
        if(half_exp >= 0x1F) return sign | 0x7C00U;

        if(half_exp <= 0)
        {
            if(half_exp < -10) return sign;
            const uint32_t mant_with_hidden_bit = mant | 0x800000U;
            const uint32_t rounded = round_shift_right(mant_with_hidden_bit, (uint8_t)(14 - half_exp));
            return sign | (uint16_t)rounded;
        }

        uint32_t half_mant = round_shift_right(mant, 13);
        uint32_t exp_out = (uint32_t)half_exp;
        if(half_mant == 0x0400U)
        {
            half_mant = 0;
            exp_out++;
            if(exp_out >= 0x1FU) return sign | 0x7C00U;
        }

        return sign | (uint16_t)(exp_out << 10) | (uint16_t)half_mant;
    }

    static constexpr uint16_t float32_to_float16_bits(const float value) noexcept
    {
#if (defined(__arm__) || defined(__aarch64__)) && defined(__ARM_FP16_FORMAT_IEEE) && defined(__ARM_FEATURE_FP16_SCALAR_ARITHMETIC)
        if(!std::is_constant_evaluated()) return native_float32_to_float16_bits(value);
#endif
        return float32_bits_to_float16_bits(float32_to_bits(value));
    }

    static constexpr uint32_t float16_bits_to_float32_bits(const uint16_t bits) noexcept
    {
        const uint32_t sign = ((uint32_t)bits & 0x8000U) << 16;
        uint32_t exp = ((uint32_t)bits >> 10) & 0x1FU;
        uint32_t mant = (uint32_t)bits & 0x03FFU;

        if(exp == 0)
        {
            if(mant == 0) return sign;

            const uint32_t shift = count_leading_zeros32(mant) - 21U;
            mant <<= shift;
            mant &= 0x03FFU;
            exp = 113U - shift;
            return sign | (exp << 23) | (mant << 13);
        }

        if(exp == 0x1FU) return sign | 0x7F800000U | (mant << 13);

        exp = exp - 15 + 127;
        return sign | (exp << 23) | (mant << 13);
    }

    static constexpr float float16_bits_to_float32(const uint16_t bits) noexcept
    {
#if (defined(__arm__) || defined(__aarch64__)) && defined(__ARM_FP16_FORMAT_IEEE) && defined(__ARM_FEATURE_FP16_SCALAR_ARITHMETIC)
        if(!std::is_constant_evaluated()) return native_float16_bits_to_float32(bits);
#endif
        return bits_to_float32(float16_bits_to_float32_bits(bits));
    }

    uint16_t bits_ = 0;
};

static_assert(sizeof(float16) == sizeof(uint16_t));
static_assert(alignof(float16) == alignof(uint16_t));
static_assert(std::is_trivially_copyable_v<float16>);
static_assert(std::is_standard_layout_v<float16>);
static_assert(sizeof(float) == sizeof(uint32_t));
static_assert(std::numeric_limits<float>::is_iec559);
}
