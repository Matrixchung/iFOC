#pragma once

#include "foc_types.hpp"
#include "foc_math.hpp"
#include "triple.hpp"

/*
 * 1) Position Control Mode:
 *    Target: pos. Feed-forward: speed, torque.
 * 2) Velocity Control Mode:
 *    Target: speed. Feed-forward: torque.
 * 3) Current Control Mode:
 *    Target: torque.
 *
 * Motion can be transformed via:
 * 1) Ref::{ELEC, BASE, OUTPUT};
 * 2) TorqueUnit::{AMP, NM};
 * 3) SpeedUnit::{RADS, DEGS, REVS, RPM, HZ};
 * 4) PosUnit::{RAD, DEG, REV};
 *
 * Note that units for internal-use are ELEC, AMP, RADS, RAD, respectively.
 */

namespace iFOC
{
struct Motion
{
    OVERRIDE_NEW();
public:
    enum class Ref :        uint8_t { ELEC, BASE, OUTPUT };
    enum class TorqueUnit : uint8_t { AMP,  NM };
    enum class SpeedUnit :  uint8_t { RADS, DEGS, REVS, RPM, HZ };
    enum class PosUnit :    uint8_t { RAD,  DEG,  REV };
    Ref ref{Ref::ELEC};
    triple<real_t, TorqueUnit> torque{0.0f, 0.0f, TorqueUnit::AMP};
    triple<real_t, SpeedUnit> speed{0.0f, 0.0f, SpeedUnit::RADS};
    triple<real_t, PosUnit> pos{0.0f, 0.0f,PosUnit::RAD};
    void Reset()
    {
        ref = Ref::ELEC;
        torque = {0.0f, TorqueUnit::AMP};
        speed = {0.0f, SpeedUnit::RADS};
        pos = {0.0f, PosUnit::RAD};
    }
    /// Transform speed and pos unit from default unit agreement: RADS, RAD
    /// \param speed_unit
    /// \param pos_unit
    void ConvertSpeedPosFromDefault(SpeedUnit speed_unit, PosUnit pos_unit)
    {
        if(speed.unit != SpeedUnit::RADS || pos.unit != PosUnit::RAD) return; // let's double-check
        switch(speed_unit)
        {
            case Motion::SpeedUnit::DEGS:
            {
                speed = {RAD2DEG(speed.value), RAD2DEG(speed.limit), speed_unit};
                break;
            }
            case Motion::SpeedUnit::REVS:
            case Motion::SpeedUnit::HZ:
            {
                speed = {RAD2REV(speed.value), RAD2REV(speed.limit), speed_unit};
                break;
            }
            case Motion::SpeedUnit::RPM:
            {
                speed = {RAD2RPM(speed.value, 1), RAD2RPM(speed.limit, 1), speed_unit};
                break;
            }
            default: break;
        }
        switch(pos_unit)
        {
            case Motion::PosUnit::DEG:
            {
                pos = {RAD2DEG(pos.value), RAD2DEG(pos.limit), pos_unit};
                break;
            }
            case Motion::PosUnit::REV:
            {
                pos = {RAD2REV(pos.value), RAD2REV(pos.limit), pos_unit};
                break;
            }
            default: break;
        }
    }
    /// Transform speed and pos unit to default unit agreement: RADS, RAD
    void ConvertSpeedPosToDefault()
    {
        switch(speed.unit)
        {
            case SpeedUnit::DEGS:
            {
                speed = {DEG2RAD(speed.value), DEG2RAD(speed.limit), SpeedUnit::RADS};
                break;
            }
            case SpeedUnit::REVS:
            case SpeedUnit::HZ:
            {
                speed = {REV2RAD(speed.value), REV2RAD(speed.limit), SpeedUnit::RADS};
                break;
            }
            case SpeedUnit::RPM:
            {
                speed = {RPM2RAD(speed.value, 1), RPM2RAD(speed.limit, 1), SpeedUnit::RADS};
                break;
            }
            default: break;
        }
        switch(pos.unit)
        {
            case PosUnit::DEG:
            {
                pos = {DEG2RAD(pos.value), DEG2RAD(pos.limit), PosUnit::RAD};
                break;
            }
            case PosUnit::REV:
            {
                pos = {REV2RAD(pos.value), REV2RAD(pos.limit), PosUnit::RAD};
                break;
            }
            default: break;
        }
    }
};
}