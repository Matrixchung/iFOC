#pragma once

#include "bus_sense_base.hpp"

namespace iFOC::Sense
{
class BusSenseStatic final : public BusSenseBase
{
public:
    BusSenseStatic() = delete;
    explicit BusSenseStatic(real_t Vbus);
    FuncRetCode Update() override { return FuncRetCode::OK; }
};

inline BusSenseStatic::BusSenseStatic(real_t Vbus)
{
    voltage = Vbus;
    current = 0.0f;
}
}
