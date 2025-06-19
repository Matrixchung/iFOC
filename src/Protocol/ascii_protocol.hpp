#pragma once

#include "../Common/Interface/uart_base.hpp"
#include "../Motor/FOC/foc_motor.hpp"
#include "protocol_base.hpp"

namespace iFOC::Protocol
{
class ASCIIProtocolFOC final : public ProtocolBase
{
private:
    static constexpr uint16_t RX_BUFFER_SIZE = 128;
    iFOC::HAL::UARTBase* uart = nullptr;
    std::array<uint8_t, RX_BUFFER_SIZE> rx_buffer;
    uint16_t rx_ptr = 0; // store the pointer of first empty element, so rx_len = rx_ptr.
    Motion::Ref io_ref{Motion::Ref::BASE};
    Motion::TorqueUnit io_torque_unit{Motion::TorqueUnit::AMP};
    Motion::SpeedUnit io_speed_unit{Motion::SpeedUnit::RADS};
    Motion::PosUnit io_pos_unit{Motion::PosUnit::RAD};
    bool OnRxEvent(uint8_t* data, uint16_t len);
    void ProcessEachValidLine(uint8_t* data, uint16_t len);
    template<typename ... TArgs>
    void GenerateResponse(bool use_checksum, bool continuous, const char *fmt, TArgs &&... args);
    void PrintReflectedVariable(const char* name, uint8_t *ptr, Reflection::ProtoFieldType type, bool use_checksum, bool verbose);
    /// Command Handler Area
    void CmdTrajectory(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdPosition(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdPositionWithFF(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdVelocity(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdTorque(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdFeedback(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdUpdateWatchdog(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdSetAbsPosition(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdReadConfig(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdWriteConfig(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdSysInfo(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdMotorInfo(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdHelp(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdSystem(uint8_t *data, uint16_t len, bool use_checksum);
    void CmdUnknown(uint8_t* data, uint16_t len, bool use_checksum);

    bool CheckAndGetID(uint8_t* data, uint16_t len, bool use_checksum, uint8_t expected_len, uint8_t& id);
public:
    explicit ASCIIProtocolFOC(iFOC::HAL::UARTBase* base);
};
}