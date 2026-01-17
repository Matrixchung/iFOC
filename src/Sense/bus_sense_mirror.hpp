#pragma once

#include "bus_sense_base.hpp"

namespace iFOC::Sense
{
class BusSenseMirror final : public BusSenseBase
{
public:
    BusSenseMirror() = delete;
    explicit BusSenseMirror(BusSenseBase* _other);
    FuncRetCode Update() override;
private:
    BusSenseBase* other = nullptr;
};
}