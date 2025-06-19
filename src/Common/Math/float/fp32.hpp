#pragma once

#include <cstdint>
#include <bit>

struct fp32{
    struct {
        uint32_t frac:23;
        uint32_t exp:8;
        uint32_t sign:1;
    }raw;

    constexpr explicit fp32(float fv):
            raw(std::bit_cast<uint32_t>(fv)){}

    constexpr explicit operator float() const {
        return std::bit_cast<float>(raw);
    }
};