#include "bus_sense_series.hpp"
#include "../DataType/board_config.hpp"

#define config iFOC::BoardConfig().GetConfig()

namespace iFOC::Sense
{
BusSenseSeries::BusSenseSeries(BusSenseBase* _pIn, BusSenseBase* _pOut) : pIn(_pIn), pOut(_pOut)
{
    type = BusSenseType::SERIAL;
}

FuncRetCode BusSenseSeries::Init()
{
    if(!pIn || !pOut) return FuncRetCode::PARAM_NOT_EXIST;
    auto ret = pIn->Init();
    if(ret != FuncRetCode::OK) return ret;
    ret = pOut->Init();
    if(ret != FuncRetCode::OK) return ret;
    return FuncRetCode::OK;
}

FuncRetCode BusSenseSeries::Update()
{
    auto ret = pIn->Update();
    if(ret != FuncRetCode::OK)
    {
        voltage = 0.0f;
        current = 0.0f;
        return ret;
    }
    ret = pOut->Update();
    if(ret != FuncRetCode::OK)
    {
        voltage = 0.0f;
        current = 0.0f;
        return ret;
    }
    real_t v = pIn->voltage;
    const real_t voltage_out = pOut->voltage;
    if(voltage_out >= config.bus_undervoltage_limit())
    {
        v += voltage_out;
        v *= 0.5f;
    }
    voltage = v;
    current = pIn->current + pOut->current;
    return FuncRetCode::OK;
}
}


