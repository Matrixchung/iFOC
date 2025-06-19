#pragma once

#include "foc_types.hpp"

namespace iFOC
{
class ProtocolBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(ProtocolBase);
public:
    ProtocolBase() = default;
    virtual void Init() {};
protected:
    template <uint8_t> friend class MotorBase;
    void* _motor = nullptr;
    template<class T>
    T* GetMotor() { return reinterpret_cast<T*>(_motor); };
};
}