#include "bus_sense_mirror.hpp"

namespace iFOC::Sense
{
BusSenseMirror::BusSenseMirror(BusSenseBase* _other) : other(_other) {};

void BusSenseMirror::Update()
{
    voltage = other->voltage;
    current = other->current;
}
}