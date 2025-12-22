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

void BusSenseSeries::Update()
{
    pIn->Update();
    pOut->Update();
    real_t v = pIn->voltage;
    const real_t voltage_out = pOut->voltage;
    if(voltage_out >= config.bus_undervoltage_limit())
    {
        v += voltage_out;
        v *= 0.5f;
    }
    voltage = v;
    current = pIn->current + pOut->current;
}
}


