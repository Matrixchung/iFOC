#pragma once

#include "../../Common/foc_types.hpp"

namespace iFOC::Driver
{
class DriverBase
{
    DELETE_COPY_CONSTRUCTOR(DriverBase);
    OVERRIDE_NEW();
public:
    DriverBase() = default;
    virtual FuncRetCode Init(bool initCNT) = 0;
    virtual void EnableAllOutputs() = 0;
    virtual void DisableAllOutputs() = 0;
};

}