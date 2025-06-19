#include "gpio_base.hpp"

namespace iFOC::HAL
{

void GPIOBase::Toggle()
{
    Write(!Read());
}

GPIOBase &GPIOBase::ModeOutPP()
{
    SetMode(GPIOMode::OUTPUT_PUSHPULL);
    return *this;
}

GPIOBase &GPIOBase::ModeOutOD()
{
    SetMode(GPIOMode::OUTPUT_OPENDRAIN);
    return *this;
}

GPIOBase &GPIOBase::ModeAlter()
{
    SetMode(GPIOMode::ALTERNATE_MODE);
    return *this;
}

GPIOBase &GPIOBase::ModeInput()
{
    SetMode(GPIOMode::INPUT);
    return *this;
}

GPIOBase &GPIOBase::PullUp()
{
    SetPull(GPIOPull::PULL_UP);
    return *this;
}

GPIOBase &GPIOBase::PullDown()
{
    SetPull(GPIOPull::PULL_DOWN);
    return *this;
}

GPIOBase &GPIOBase::PullNo()
{
    SetPull(GPIOPull::PULL_NO);
    return *this;
}

GPIOBase &GPIOBase::operator=(const bool val)
{
    Write(val);
    return *this;
}

GPIOBase &GPIOBase::operator= (const int val)
{
    Write(val);
    return *this;
}

GPIOBase &GPIOBase::operator= (const GPIOBase & other)
{
    Write(other.Read());
    return *this;
}

GPIOBase::operator bool() const
{
    return Read();
}

}
