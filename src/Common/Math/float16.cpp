#include "float16.hpp"

namespace iFOC
{
namespace
{
static_assert(float16::from_bits(0x0000U).bits() == 0x0000U);
static_assert(float16::from_bits(0x8000U).bits() == 0x8000U);
static_assert(float16::from_bits(0x7C00U).is_inf());
static_assert(float16::from_bits(0xFC00U).is_inf());
static_assert(float16::from_bits(0x7E00U).is_nan());
static_assert(!float16::from_bits(0x7BFFU).is_nan());
static_assert(float16::from_bits(0x7BFFU).is_finite());

static_assert(std::bit_cast<uint32_t>(float16::from_bits(0x0000U).to_float()) == 0x00000000U);
static_assert(std::bit_cast<uint32_t>(float16::from_bits(0x8000U).to_float()) == 0x80000000U);
static_assert(std::bit_cast<uint32_t>(float16::from_bits(0x3C00U).to_float()) == 0x3F800000U); // 1.0
static_assert(std::bit_cast<uint32_t>(float16::from_bits(0xC000U).to_float()) == 0xC0000000U); // -2.0
static_assert(std::bit_cast<uint32_t>(float16::from_bits(0x0400U).to_float()) == 0x38800000U); // min normal
static_assert(std::bit_cast<uint32_t>(float16::from_bits(0x0001U).to_float()) == 0x33800000U); // min subnormal
static_assert(std::bit_cast<uint32_t>(float16::from_bits(0x7C00U).to_float()) == 0x7F800000U); // +inf
static_assert(std::bit_cast<uint32_t>(float16::from_bits(0xFC00U).to_float()) == 0xFF800000U); // -inf

static_assert(float16(0.0f).bits() == 0x0000U);
static_assert(float16(-0.0f).bits() == 0x8000U);
static_assert(float16(1.0f).bits() == 0x3C00U);
static_assert(float16(-2.0f).bits() == 0xC000U);
static_assert(float16(65504.0f).bits() == 0x7BFFU);
static_assert(float16(0x1p-24f).bits() == 0x0001U);
static_assert(float16(0x1p-25f).bits() == 0x0000U);
static_assert(float16(0x1.8p-24f).bits() == 0x0002U);
static_assert(float16(1.00048828125f).bits() == 0x3C00U);
static_assert(float16(1.00146484375f).bits() == 0x3C02U);
static_assert(float16(1.0009765625f).bits() == 0x3C01U);
static_assert(float16(0x1p16f).bits() == 0x7C00U);

static_assert(std::is_convertible_v<float, float16>);
static_assert(!std::is_convertible_v<float16, float>);
static_assert([]{
    const float value = 1.0f;
    const float16 half = value;
    return half.bits() == 0x3C00U;
}());
static_assert([]{
    float16 half;
    half = 2.0f;
    return half.bits() == 0x4000U;
}());

static_assert(float16(1.0f) == float16(1.0f));
static_assert(float16::positive_zero() == float16::negative_zero());
static_assert(float16(1.0f) != float16(2.0f));
static_assert(float16(1.0f) < float16(2.0f));
static_assert(float16(2.0f) <= float16(2.0f));
static_assert(float16(3.0f) > float16(2.0f));
static_assert(float16(3.0f) >= float16(3.0f));
static_assert(float16::from_bits(0x7E00U) != float16::from_bits(0x7E00U));

static_assert((+float16(1.0f)).bits() == 0x3C00U);
static_assert((-float16(1.0f)).bits() == 0xBC00U);
static_assert((float16(1.0f) + float16(2.0f)).bits() == 0x4200U);
static_assert((float16(5.0f) - float16(2.0f)).bits() == 0x4200U);
static_assert((float16(2.0f) * float16(3.0f)).bits() == 0x4600U);
static_assert((float16(6.0f) / float16(2.0f)).bits() == 0x4200U);
static_assert([]{
    float16 value = 1.0f;
    value += float16(2.0f);
    value *= float16(2.0f);
    value -= float16(1.0f);
    value /= float16(5.0f);
    return value.bits() == 0x3C00U;
}());
}
}
