#pragma once

#include "bus_sense_base.hpp"

/*
 * VBus In -> ----------- MOSFETs ---------- -> VBus Out
 *  GND In -> | Shunt | -> PGND -> | Shunt | -> GND Out
 *            |       |            |       |
 *            V       V            V       V
 *             Sense1               Sense2
 */

namespace iFOC::Sense
{
class BusSenseSeries final : public BusSenseBase
{
    DELETE_COPY_CONSTRUCTOR(BusSenseSeries);
public:
    BusSenseSeries(BusSenseBase* _pIn, BusSenseBase* _pOut);
    FuncRetCode Init() override;
    FuncRetCode Update() override;
// private:
    BusSenseBase* pIn;
    BusSenseBase* pOut;
};
}