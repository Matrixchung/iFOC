#pragma once

#include "foc_types.hpp"

namespace iFOC::HAL
{
class SPIBase
{
    OVERRIDE_NEW();
public:
    enum class DataWidth : uint8_t
    {
        BYTE = 0,
        HALF_WORD = 1
    };
    virtual FuncRetCode Init() { return FuncRetCode::OK; };
    virtual FuncRetCode WriteBytes(const uint8_t* data, uint16_t size) = 0;
    virtual FuncRetCode ReadBytes(uint8_t* data, uint16_t size) = 0;
    virtual FuncRetCode WriteReadBytes(const uint8_t* write_data,
                                       uint8_t* read_data,
                                       uint16_t size) = 0;
    virtual void SetDataWidth(DataWidth w) {};
    virtual void SetClock(const uint32_t clock) {};
    virtual void SetCPOLCPHA(const uint8_t cpol, const uint8_t cpha) {};
};

template<typename T>
concept SPIImpl = std::is_base_of<SPIBase, T>::value;
}