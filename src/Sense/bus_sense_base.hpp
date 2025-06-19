#pragma once

#include "../Common/foc_types.hpp"
#include "../Common/foc_math.hpp"

namespace iFOC::Sense
{
class BusSenseBase
{
    OVERRIDE_NEW();
public:
    real_t voltage = 0.0f;
    real_t current = 0.0f;
    virtual FuncRetCode Init() { return FuncRetCode::OK; };
    virtual void Update() = 0;
};

template<typename T>
concept BusSenseImpl = std::is_base_of<BusSenseBase, T>::value;
}