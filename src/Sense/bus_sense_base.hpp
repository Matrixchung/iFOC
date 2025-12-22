#pragma once

#include "../Common/foc_types.hpp"
#include "../Common/foc_math.hpp"

namespace iFOC::Sense
{
enum class BusSenseType : uint8_t
{
    SINGLE_ENDED = 0,
    SERIAL = 1
};
class BusSenseBase
{
    OVERRIDE_NEW();
public:
    real_t voltage = 0.0f; // [V]
    real_t current = 0.0f; // [A], positive value means current flow from supply to driver
    [[nodiscard]] BusSenseType GetBusSenseType() const { return type; }
    virtual FuncRetCode Init() { return FuncRetCode::OK; };
    virtual void Update() = 0;
protected:
    BusSenseType type = BusSenseType::SINGLE_ENDED;
};

template<typename T>
concept BusSenseImpl = std::is_base_of<BusSenseBase, T>::value;
}