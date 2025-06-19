#pragma once

#include "bus_sense_base.hpp"

namespace iFOC::Sense
{
class BusSenseMirror final : public BusSenseBase
{
public:
    BusSenseMirror() = delete;
    explicit BusSenseMirror(BusSenseBase* _other) : other(_other) {};
    void Update() final;
private:
    BusSenseBase* other = nullptr;
};

void BusSenseMirror::Update()
{
    voltage = other->voltage;
    current = other->current;
}
}