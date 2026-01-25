#pragma once

#include "../Common/Interface/uart_base.hpp"
#include "../Motor/FOC/foc_motor.hpp"
#include "../Motor/DC/dc_motor.hpp"
#include "protocol_base.hpp"
#include "ascii_tiny_printf.hpp"
#include <cstring>

namespace iFOC::Protocol
{
template<class Motor>
class ASCIIProtocol final : public ProtocolBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(ASCIIProtocol);
private:
    static constexpr uint16_t RX_BUFFER_SIZE = 128;
    iFOC::HAL::UARTBase* uart = nullptr;
    std::array<uint8_t, RX_BUFFER_SIZE> rx_buffer{};
    uint16_t rx_ptr = 0; // store the pointer of first empty element, so rx_len = rx_ptr.
    Motion::Ref io_ref{Motion::Ref::BASE};
    Motion::TorqueUnit io_torque_unit{Motion::TorqueUnit::AMP};
    Motion::SpeedUnit io_speed_unit{Motion::SpeedUnit::RADS};
    Motion::PosUnit io_pos_unit{Motion::PosUnit::RAD};

    Motor* GetInst() { return GetMotor<Motor>(); }
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
    void CmdVelocityOpenloop(uint8_t *data, uint16_t len, bool use_checksum);
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
    explicit ASCIIProtocol(HAL::UARTBase* base);
};

namespace {

using MotorState = iFOC::DataType::Base::MotorState;
using MotorControlMode = iFOC::DataType::Base::MotorControlMode;
using Motion = iFOC::Motion;

inline void append_uint_to_string(String& s, uint16_t num)
{
    if(num == 0)
    {
        s.append("0");
    }
    else
    {
        char buffer[6]{};
        uint8_t pos = 0;
        while(num > 0)
        {
            buffer[pos++] = '0' + num % 10;
            num /= 10;
        }
        buffer[pos] = '\0';
        String temp{buffer};
        std::reverse(temp.begin(), temp.end());
        s.append(temp);
    }
}

inline bool streq(const char* a, const char* b) { return std::strcmp(a, b) == 0; }

inline const char* to_string(MotorState s)
{
    switch(s)
    {
        case MotorState::IDLE: return "IDLE";
        case MotorState::STARTUP_SEQUENCE: return "STARTUP_SEQUENCE";
        case MotorState::BASIC_PARAM_CALIBRATION: return "BASIC_PARAM_CALIBRATION";
        case MotorState::ENCODER_INDEX_SEARCH: return "ENCODER_INDEX_SEARCH";
        case MotorState::ENCODER_CALIBRATION: return "ENCODER_CALIBRATION";
        case MotorState::EXTEND_PARAM_CALIBRATION: return "EXTEND_PARAM_CALIBRATION";
        case MotorState::SENSORED_CLOSED_LOOP_CONTROL: return "SENSORED_CLOSED_LOOP_CONTROL";
        case MotorState::SENSORLESS_CLOSED_LOOP_CONTROL: return "SENSORLESS_CLOSED_LOOP_CONTROL";
        case MotorState::OPEN_LOOP_VELOCITY_CONTROL: return "OPEN_LOOP_VELOCITY_CONTROL";
        default: return "UNKNOWN";
    }
}

inline bool from_string(const char* str, MotorState& out)
{
    if(streq(str, "IDLE")) { out = MotorState::IDLE; return true; }
    if(streq(str, "STARTUP_SEQUENCE")) { out = MotorState::STARTUP_SEQUENCE; return true; }
    if(streq(str, "BASIC_PARAM_CALIBRATION")) { out = MotorState::BASIC_PARAM_CALIBRATION; return true; }
    if(streq(str, "ENCODER_INDEX_SEARCH")) { out = MotorState::ENCODER_INDEX_SEARCH; return true; }
    if(streq(str, "ENCODER_CALIBRATION")) { out = MotorState::ENCODER_CALIBRATION; return true; }
    if(streq(str, "EXTEND_PARAM_CALIBRATION")) { out = MotorState::EXTEND_PARAM_CALIBRATION; return true; }
    if(streq(str, "SENSORED_CLOSED_LOOP_CONTROL")) { out = MotorState::SENSORED_CLOSED_LOOP_CONTROL; return true; }
    if(streq(str, "SENSORLESS_CLOSED_LOOP_CONTROL")) { out = MotorState::SENSORLESS_CLOSED_LOOP_CONTROL; return true; }
    if(streq(str, "OPEN_LOOP_VELOCITY_CONTROL")) { out = MotorState::OPEN_LOOP_VELOCITY_CONTROL; return true; }
    return false;
}

inline const char* to_string(MotorControlMode m)
{
    switch(m)
    {
        case MotorControlMode::CTRL_MODE_POSITION: return "CTRL_MODE_POSITION";
        case MotorControlMode::CTRL_MODE_VELOCITY: return "CTRL_MODE_VELOCITY";
        case MotorControlMode::CTRL_MODE_CURRENT: return "CTRL_MODE_CURRENT";
        case MotorControlMode::CTRL_MODE_HYBRID: return "CTRL_MODE_HYBRID";
        default: return "UNKNOWN";
    }
}

inline bool from_string(const char* str, MotorControlMode& out)
{
    if(streq(str, "CTRL_MODE_POSITION")) { out = MotorControlMode::CTRL_MODE_POSITION; return true; }
    if(streq(str, "CTRL_MODE_VELOCITY")) { out = MotorControlMode::CTRL_MODE_VELOCITY; return true; }
    if(streq(str, "CTRL_MODE_CURRENT")) { out = MotorControlMode::CTRL_MODE_CURRENT; return true; }
    if(streq(str, "CTRL_MODE_HYBRID")) { out = MotorControlMode::CTRL_MODE_HYBRID; return true; }
    return false;
}

inline const char* to_string(Motion::Ref r)
{
    switch(r)
    {
        case Motion::Ref::ELEC: return "ELEC";
        case Motion::Ref::BASE: return "BASE";
        case Motion::Ref::OUTPUT: return "OUTPUT";
        default: return "UNKNOWN";
    }
}

inline bool from_string(const char* str, Motion::Ref& out)
{
    if(streq(str, "ELEC")) { out = Motion::Ref::ELEC; return true; }
    if(streq(str, "BASE")) { out = Motion::Ref::BASE; return true; }
    if(streq(str, "OUTPUT")) { out = Motion::Ref::OUTPUT; return true; }
    return false;
}

inline const char* to_string(Motion::TorqueUnit u)
{
    switch(u)
    {
        case Motion::TorqueUnit::AMP: return "AMP";
        case Motion::TorqueUnit::NM: return "NM";
        default: return "UNKNOWN";
    }
}

inline bool from_string(const char* str, Motion::TorqueUnit& out)
{
    if(streq(str, "AMP")) { out = Motion::TorqueUnit::AMP; return true; }
    if(streq(str, "NM")) { out = Motion::TorqueUnit::NM; return true; }
    return false;
}

inline const char* to_string(Motion::SpeedUnit u)
{
    switch(u)
    {
        case Motion::SpeedUnit::RADS: return "RADS";
        case Motion::SpeedUnit::DEGS: return "DEGS";
        case Motion::SpeedUnit::REVS: return "REVS";
        case Motion::SpeedUnit::RPM: return "RPM";
        case Motion::SpeedUnit::HZ: return "HZ";
        default: return "UNKNOWN";
    }
}

inline bool from_string(const char* str, Motion::SpeedUnit& out)
{
    if(streq(str, "RADS")) { out = Motion::SpeedUnit::RADS; return true; }
    if(streq(str, "DEGS")) { out = Motion::SpeedUnit::DEGS; return true; }
    if(streq(str, "REVS")) { out = Motion::SpeedUnit::REVS; return true; }
    if(streq(str, "RPM")) { out = Motion::SpeedUnit::RPM; return true; }
    if(streq(str, "HZ")) { out = Motion::SpeedUnit::HZ; return true; }
    return false;
}

inline const char* to_string(Motion::PosUnit u)
{
    switch(u)
    {
        case Motion::PosUnit::RAD: return "RAD";
        case Motion::PosUnit::DEG: return "DEG";
        case Motion::PosUnit::REV: return "REV";
        default: return "UNKNOWN";
    }
}

inline bool from_string(const char* str, Motion::PosUnit& out)
{
    if(streq(str, "RAD")) { out = Motion::PosUnit::RAD; return true; }
    if(streq(str, "DEG")) { out = Motion::PosUnit::DEG; return true; }
    if(streq(str, "REV")) { out = Motion::PosUnit::REV; return true; }
    return false;
}

void splitData_f(char* pSource, const uint16_t Lsource, float* pDest, uint8_t* Ldest, const uint8_t max_dest_len, const uint8_t split)
{
    uint8_t temp = 0, point = 0, negative = 0;
    uint16_t numCount = 0;
    *(Ldest) = 0;
    // for(i = 0; i < maxLen; i++) *(pDest+i) = 0; // flush the dest buffer, maxLen represents maximum length of pDest (prevent overflow)
    for(uint16_t i = 0; i < Lsource && *(Ldest) <= max_dest_len; i++)
    {
        float& dest = *(pDest + *(Ldest));
        temp = *(pSource + i);
        if(temp != split)
        {
            if(numCount == 0 && dest != 0) dest = 0.0f; // flush the used pDest buffer
            if(temp == '-')
            {
                if(dest != 0) dest = 0.0f; // error: '-' after numbers, ignoring forward nums
                negative = 1;
            }
            else if(temp == '.')
            { // point
                if(point) point = 0; // error: multiple points in one continous number
                else point = 1;
            }
            else if(temp >= '0' && temp <= '9')
            {
                if(point)
                {
                    // *(pDest + *(Ldest)) += (float)((temp - '0') / (std::powf(10, point++)));
                    dest += (float)((temp - '0') / quick_powf<10>(point++));
                }
                else
                {
                    dest *= 10.0f;
                    dest += (float)(temp - '0');
                }
                numCount++;
            }
        }
        if(temp == split || i == Lsource - 1)
        {
            if(dest != 0 || (dest == 0 && numCount > 0))
            { // (pDest[Ldest] != 0 || (pDest[Ldest] == 0 && numCount >0) && temp == split, check negative and move Ldest pointer back
                if(negative) dest = (float)(-1.0f * dest);
                *(Ldest) += 1;
            }
            negative = point = numCount = 0;
        }
    }
}

size_t splitStr(const std::string_view& source, Vector<String>& dest, const char split)
{
    dest.clear();
    size_t start = 0, end = 0;
    while((end = source.find(split, start)) != std::string::npos)
    {
        dest.emplace_back(source.substr(start, end - start));
        start = end + 1;
    }
    dest.emplace_back(source.substr(start));
    return dest.size();
}
}

template <class Motor>
ASCIIProtocol<Motor>::ASCIIProtocol(HAL::UARTBase* base) : uart(base)
{
    rx_buffer.fill(0);
    uart->RegisterRxHandler(std::bind(&ASCIIProtocol::OnRxEvent, this, std::placeholders::_1, std::placeholders::_2));
}

/// Note 1: New data comes in, maybe containing multiple EOLs or no EOL. So we need to append it to our own
///         buffer, and search the whole section. If EOF matched, we can pop them out. If no match, we do nothing. \n
/// Note 2: If our buffer is full the next time new data in, we should pop out the same size of data from
///         the head, and push new data into the buffer, and repeat (1). \n
/// Note 3: Typically, those processes mentioned above suit dual-end data structures like std::deque, however
///         we will do this in a FIXED-SIZE std::array, to minimalize potential memory fragments or leaks. \n
template <class Motor>
bool ASCIIProtocol<Motor>::OnRxEvent(uint8_t* data, uint16_t len)
{
    do // if income len > rx_buffer.size(), we will process multiple times
    {
        uint16_t actual_len = MIN(len, rx_buffer.max_size());
        // Step #1: refers to Note 2
        if(rx_ptr + actual_len > rx_buffer.max_size())
        {
            uint16_t len_to_be_removed = rx_ptr + actual_len - rx_buffer.max_size();
            rx_ptr -= len_to_be_removed; // rx_len after removal
            memmove(rx_buffer.data(), rx_buffer.data() + len_to_be_removed, rx_ptr);
        }
        memcpy(rx_buffer.data() + rx_ptr, data, actual_len);
        rx_ptr += actual_len;
        // Step #2: we have received the datas into buffer. Here we can do a test print (for debug)
        //          and now its time to search our buffer. The whole length is rx_ptr.
        while(true)
        {
            uint16_t first_eol_section_ptr = 0; // including EOL character.
            while(first_eol_section_ptr < rx_ptr)
            {
                uint8_t c = *(rx_buffer.data() + first_eol_section_ptr);
                if(c == '\r' || c == '\n' || c == '!') break;
                first_eol_section_ptr++;
            }
            if(first_eol_section_ptr >= rx_ptr) break; // searched to end
            uint16_t valid_line_len = first_eol_section_ptr + 1;
            // no empty payload will be processed.
            // this also helps when meet consistent EOLs (like: foo 0.1!!!!!!! will only be processed as foo 0.1)
            if(valid_line_len > 1) ProcessEachValidLine(rx_buffer.data(), first_eol_section_ptr);
            // Step #3: refers to Note 1
            uint16_t size_remaining = rx_ptr - valid_line_len;
            memmove(rx_buffer.data(), rx_buffer.data() + valid_line_len, size_remaining);
            rx_ptr = size_remaining;
        }
        data += actual_len;
        len -= actual_len;
    } while(len > 0);

    // // Test print of the buffer
    // uint16_t temp = rx_ptr;
    // uint8_t* ptr = rx_buffer.data();
    // do {
    //     auto write = uart->WriteBytes(ptr, temp);
    //     if(write < temp) uart->StartTransmit(false);
    //     ptr += write;
    //     temp -= write;
    // } while(temp > 0);
    // uart->StartTransmit(false);
    return false;
}

template <class Motor>
void ASCIIProtocol<Motor>::ProcessEachValidLine(uint8_t* data, uint16_t len)
{
    uint8_t calc_checksum = 0;
    uint16_t checksum_start_len = 0xFFFF;
    // Step #1: Calculate the data checksum, find the OPTIONAL checksum asterisk
    //          and filter out the comments (starts with ';')
    for(uint16_t valid_len = 1; valid_len <= len; valid_len++)
    {
        uint8_t c = *(data + valid_len - 1);
        if(c == ';')
        {
            len = valid_len - 1;
            break;
        }
        if(checksum_start_len > valid_len)
        {
            if(c == '*') checksum_start_len = valid_len;
            else calc_checksum ^= c;
        }
    }

    // // Step #2: Copy to local buffer, and insert '\0' termination for sscanf()
    // // scanf() is a pretty slow and dangerous function in embedded systems
    // len = MIN(len, line_buffer.size() - 1);
    // memcpy(line_buffer.data(), data, len);
    // line_buffer[len] = '\0';

    // Step #2: if the command used checksum, we should read out and check
    bool use_checksum = (checksum_start_len < len);
    unsigned int readout_checksum = 0;
    if(use_checksum)
    {
        float temp = 0.0f;
        uint8_t numscan = 0;
        splitData_f((char*)(data + checksum_start_len), len - checksum_start_len, &temp, &numscan, 2, ' ');
        readout_checksum = (unsigned int)temp;
        if(numscan != 1 || readout_checksum != calc_checksum) return;
        len = checksum_start_len - 1;
        // line_buffer[len] = '\0'; // insert termination at new position
    }

    // Step #3: trim the head and tail, removing useless spaces
    // NOTE: this might cause problem when using checksums
    while(len)
    {
        if(data[0] == ' ')
        {
            data++;
            len--;
        }
        else if(data[len - 1] == ' ')
        {
            len--;
        }
        else break;
    }

    // Step #4: handle real command type
    switch(data[0])
    {
        case 'q': CmdPosition(data, len, use_checksum); break;
        case 'p': CmdPositionWithFF(data, len, use_checksum); break;
        case 't': CmdTrajectory(data, len, use_checksum); break;
        case 'v': CmdVelocity(data, len, use_checksum); break;
        case 'o': CmdVelocityOpenloop(data, len, use_checksum); break;
        case 'c': CmdTorque(data, len, use_checksum); break;
        case 'f': CmdFeedback(data, len, use_checksum); break;
        case 'u': CmdUpdateWatchdog(data, len, use_checksum); break;
        case 'r': CmdReadConfig(data, len, use_checksum); break;
        case 'w': CmdWriteConfig(data, len, use_checksum); break;
        case 'i': CmdSysInfo(data, len, use_checksum); break;
        case 'm': CmdMotorInfo(data, len, use_checksum); break;
        case 'h': CmdHelp(data, len, use_checksum); break;
        case 's': CmdSystem(data, len, use_checksum); break;
        default: CmdUnknown(data, len, use_checksum); break;
    }

    // // Test print of the buffer
    // char *ptr = reinterpret_cast<char*>(data);
    // do {
    //     auto write = uart->WriteBytes(reinterpret_cast<const uint8_t *>(ptr), len);
    //     if(write < len) uart->StartTransmit(false);
    //     ptr += write;
    //     len -= write;
    // } while(len > 0);
    // if(use_checksum)
    // {
    //     uart->Print(0, "cc:%d,rc:%d", calc_checksum, readout_checksum);
    // }
    // uart->Print(0, "\n");
    // uart->StartTransmit(false);
}

template <class Motor>
template <typename ... TArgs>
void ASCIIProtocol<Motor>::GenerateResponse(bool use_checksum, bool continuous, const char* fmt, TArgs&&... args)
{
    char tx_buf[128];
    size_t len = snprintf_(tx_buf, sizeof(tx_buf), fmt, std::forward<TArgs>(args)...);
    if(len > uart->GetTxAvailable()) uart->StartTransmit(false); // initiate a transmission
    uart->WriteBytes((const uint8_t*)tx_buf, len);
    if(use_checksum)
    {
        uint8_t checksum = 0;
        for(size_t i = 0; i < len; ++i) checksum ^= tx_buf[i];
        uart->Print(!continuous, "*%d\r\n", checksum);
    }
    else uart->Print(!continuous, "\r\n");
}

template <class Motor>
void ASCIIProtocol<Motor>::PrintReflectedVariable(const char* name, uint8_t* ptr, Reflection::ProtoFieldType type,
    bool use_checksum, bool verbose)
{
    switch(type)
    {
        // case Reflection::ProtoFieldType::DOUBLE:
        // {
        //     if(verbose) GenerateResponse(use_checksum, true, "%s(double): %.5f", name, *(double*)ptr);
        //     else GenerateResponse(use_checksum, true, "%.5f", *(double*)ptr);
        //     break;
        // }
        case Reflection::ProtoFieldType::FLOAT:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(float): %.6f", name, *(float*)ptr);
            else GenerateResponse(use_checksum, true, "%.6f", *(float*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::INT32:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(int32): %d", name, *(int32_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(int32_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::INT64:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(int64): %d", name, *(int64_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(int64_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::UINT32:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(uint32): %d", name, *(uint32_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(uint32_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::UINT64:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(uint64): %d", name, *(uint64_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(uint64_t*)ptr);
            break;
        }
        case Reflection::ProtoFieldType::BOOL:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(bool): %d", name, *(uint8_t*)ptr);
            else GenerateResponse(use_checksum, true, "%d", *(uint8_t*)ptr);
            break;
        }
        default:
        {
            if(verbose) GenerateResponse(use_checksum, true, " - %s(unknown)", name);
            break;
        }
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdTrajectory(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id) && id == motor->GetInternalID())
    {
        uint8_t target_count = 0;
        float target[3]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 3, ' ');
        if(target_count < 1 || target_count > 2)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {0.0f, 0.0f, io_torque_unit},
            .speed = {0.0f, 0.0f, io_speed_unit},
            .pos = {target[0], 0.0f, io_pos_unit}
        };
        bool s_curve = false;
        if(target_count == 2) s_curve = target[1] == 1.0f;
        motor->UpdateWatchdog();
        motor->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
        motor->SetTrajectoryTargetMotion(target_motion, s_curve);
        GenerateResponse(use_checksum, false, "ok");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdPosition(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id) && id == motor->GetInternalID())
    {
        uint8_t target_count = 0;
        float target[4]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 4, ' ');
        if(target_count < 1 || target_count > 3)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {0.0f, target_count == 3 ? target[2] : 0.0f, io_torque_unit},
            .speed = {0.0f, target_count >= 2 ? target[1] : 0.0f, io_speed_unit},
            .pos = {target[0], 0.0f, io_pos_unit}
        };
        motor->UpdateWatchdog();
        motor->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
        motor->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdPositionWithFF(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id) && id == motor->GetInternalID())
    {
        uint8_t target_count = 0;
        float target[4]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 4, ' ');
        if(target_count < 1 || target_count > 3)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {target_count == 3 ? target[2] : 0.0f, 0.0f, io_torque_unit},
            .speed = {target_count >= 2 ? target[1] : 0.0f, 0.0f, io_speed_unit},
            .pos = {target[0], 0.0f, io_pos_unit}
        };
        motor->UpdateWatchdog();
        motor->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
        motor->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdVelocity(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id) && id == motor->GetInternalID())
    {
        uint8_t target_count = 0;
        float target[3]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 3, ' ');
        if(target_count < 1 || target_count > 2)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {target_count == 2 ? target[1] : 0.0f, 0.0f, io_torque_unit},
            .speed = {target[0], 0.0f, io_speed_unit},
            .pos = {0.0f, 0.0f, io_pos_unit}
        };
        motor->UpdateWatchdog();
        motor->SetControlMode(MotorControlMode::CTRL_MODE_VELOCITY);
        motor->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdVelocityOpenloop(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id) && id == motor->GetInternalID())
    {
        uint8_t target_count = 0;
        float target[4]{};
        splitData_f((char*)(data + 4), len - 4, target, &target_count, 4, ' ');
        if(target_count != 3 || target[2] < 0.0f)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {target[1], 0.0f, io_torque_unit},
            .speed = {target[0], 0.0f, io_speed_unit},
            .pos = {0.0f, 0.0f, io_pos_unit}
        };
        motor->UpdateWatchdog();
        motor->SetControlMode(MotorControlMode::CTRL_MODE_VELOCITY);
        motor->SetRampedTargetMotion(target_motion, target[2]);
        GenerateResponse(use_checksum, false, "ok");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdTorque(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 5, id) && id == motor->GetInternalID())
    {
        uint8_t target_count = 0;
        float target = 0.0f;
        splitData_f((char*)(data + 4), len - 4, &target, &target_count, 1, ' ');
        if(target_count != 1)
        {
            GenerateResponse(use_checksum, false, "invalid argument input");
            return;
        }
        Motion target_motion{
            .ref = io_ref,
            .torque = {target, 0.0f, io_torque_unit},
            .speed = {0.0f, 0.0f, io_speed_unit},
            .pos = {0.0f, 0.0f, io_pos_unit}
        };
        motor->UpdateWatchdog();
        motor->SetControlMode(MotorControlMode::CTRL_MODE_CURRENT);
        motor->SetTargetMotion(target_motion);
        GenerateResponse(use_checksum, false, "ok");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdFeedback(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    uint8_t id = 0;
    if(CheckAndGetID(data, len, use_checksum, 3, id) && id == motor->GetInternalID())
    {
        const auto& current = motor->GetCurrentMotionStruct(io_ref, io_torque_unit, io_speed_unit, io_pos_unit);
        GenerateResponse(use_checksum, false, "%.3f %.3f", current.pos.value, current.speed.value);
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdUpdateWatchdog(uint8_t* data, uint16_t len, bool use_checksum)
{
    if(len < 3) return;
    const auto motor = GetInst();
    uint8_t dst = *(data + 2);
    if(dst - '0' == motor->GetInternalID())
    {
        motor->UpdateWatchdog();
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdSetAbsPosition(uint8_t* data, uint16_t len, bool use_checksum)
{

}

template <class Motor>
void ASCIIProtocol<Motor>::CmdReadConfig(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    if(len < 3)
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    uint8_t dst = *(data + 2);
    if((!isdigit(dst) && dst != 'b') || (isdigit(dst) && dst - '0' + 1 > SYSTEM_MOTOR_NUM))
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    if(dst == 'b')
    {
        if(motor->GetInternalID() != 0) return; // board config should be only handled by motor #0
    }
    else if(dst - '0' != motor->GetInternalID()) return;
    const auto& reflect = dst == 'b' ? BoardConfig().GetConfig().GetReflectMap() : motor->GetConfig().GetReflectMap();
    uint8_t *start_ptr = dst == 'b' ? (uint8_t*)(&BoardConfig().GetConfig()) : (uint8_t*)(&motor->GetConfig());
    if(len < 5) // print all config
    {
        if(dst == 'b') GenerateResponse(use_checksum, true, "Board Config:");
        else GenerateResponse(use_checksum, true, "Motor %c Config:", dst);
        for(const auto& [name, info] : reflect)
        {
            uint8_t *ptr = start_ptr + info.second;
            PrintReflectedVariable(name, ptr, info.first, use_checksum, true);
        }
        uart->StartTransmit(false);
    }
    else
    {
        uint8_t arg_len = len - 4;
        Vector<char> temp(arg_len + 2);
        memcpy(temp.data(), data + 4, arg_len);
        if(temp[arg_len - 1] != '_')
        {
            temp[arg_len] = '_';
            arg_len++;
        }
        temp[arg_len] = '\0';
        if(const auto& it = reflect.find(temp.data()); it != reflect.end())
        {
            const auto& info = it->second;
            uint8_t *ptr = start_ptr + info.second;
            PrintReflectedVariable(it->first, ptr, info.first, use_checksum, false);
            uart->StartTransmit(false);
        }
        else GenerateResponse(use_checksum, false, "invalid property");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdWriteConfig(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    if(len < 5)
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    uint8_t dst = *(data + 2);
    if((!isdigit(dst) && dst != 'b') || (isdigit(dst) && dst - '0' + 1 > SYSTEM_MOTOR_NUM) || *(data + 3) != ' ')
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    if(dst == 'b')
    {
        if(motor->GetInternalID() != 0) return; // board config should be only handled by motor #0)
    }
    else if(dst - '0' != motor->GetInternalID()) return;
    const auto& reflect = dst == 'b' ? BoardConfig().GetConfig().GetReflectMap() : motor->GetConfig().GetReflectMap();
    uint8_t *start_ptr = dst == 'b' ? (uint8_t*)(&BoardConfig().GetConfig()) : (uint8_t*)(&motor->GetConfig());
    uint8_t arg_len = 1;
    while(arg_len < (len - 4) - 1)
    {
        char temp = *(data + 3 + arg_len + 1);
        if(temp == ' ' || temp < 'A' || temp > 'z') break;
        arg_len++;
    }
    float target = 0.0f;
    uint8_t target_count = 0;
    splitData_f((char*)(data + 5 + arg_len), len - 5 - arg_len, &target, &target_count, 1, ' ');
    if(target_count != 1)
    {
        GenerateResponse(use_checksum, false, "invalid argument input");
        return;
    }
    Vector<char> temp(arg_len + 2);
    memcpy(temp.data(), data + 4, arg_len);
    if(temp[arg_len - 1] != '_')
    {
        temp[arg_len] = '_';
        arg_len++;
    }
    temp[arg_len] = '\0';
    if(const auto& it = reflect.find(temp.data()); it != reflect.end())
    {
        const auto& info = it->second;
        uint8_t *ptr = start_ptr + info.second;
        switch(info.first)
        {
            case Reflection::ProtoFieldType::FLOAT:
                *(float*)ptr = target; break;
            case Reflection::ProtoFieldType::INT32:
            {
                if(target > INT32_MAX || target < INT32_MIN) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(int32_t*)ptr = (int32_t)target;
                break;
            }
            case Reflection::ProtoFieldType::INT64:
            {
                if(target > INT64_MAX || target < INT64_MIN) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(int64_t*)ptr = (int64_t)target;
                break;
            }
            case Reflection::ProtoFieldType::UINT32:
            {
                if(target > UINT32_MAX || target < 0) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(uint32_t*)ptr = (uint32_t)target;
                break;
            }
            case Reflection::ProtoFieldType::UINT64:
            {
                if(target > UINT64_MAX || target < 0) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(uint64_t*)ptr = (uint64_t)target;
                break;
            }
            case Reflection::ProtoFieldType::BOOL:
            {
                if(target > 1 || target < 0) { GenerateResponse(use_checksum, false, "argument out of range"); return; }
                else *(uint8_t*)ptr = (uint8_t)target;
                break;
            }
            default:
            {
                GenerateResponse(use_checksum, false, "unsupported property type");
                return;
            }
        }
        GenerateResponse(use_checksum, false, "ok");
    }
    else GenerateResponse(use_checksum, false, "invalid property");
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdSysInfo(uint8_t* data, uint16_t len, bool use_checksum)
{
    static constexpr char tskRUNNING_CHAR   = 'X';
    static constexpr char tskBLOCKED_CHAR   = 'B';
    static constexpr char tskREADY_CHAR     = 'R';
    static constexpr char tskDELETED_CHAR   = 'D';
    static constexpr char tskSUSPENDED_CHAR = 'S';
    static constexpr char tskINVALID_CHAR   = '?';
    const auto motor = GetInst();
    if(motor->GetInternalID() == 0)
    {
        if(len > 1)
        {
            GenerateResponse(use_checksum, false, "invalid command format");
            return;
        }
#if configUSE_TRACE_FACILITY == 1
        TaskStatus_t *pxTaskStatusArray;
        UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
        uint32_t ulTotalRunTime = 0;
        pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
        if(pxTaskStatusArray)
        {
            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
            ulTotalRunTime /= 100UL;
            char cStatus = tskINVALID_CHAR;
#if configGENERATE_RUN_TIME_STATS == 1
            uint32_t ulStatsAsPercentage = 0;
            if(ulTotalRunTime > 0)
                uart->Print(false, "   TaskName\tState\tPrio.\tMin.Stack\tCPUTime\tTaskID\r\n");
            else
#endif
                uart->Print(false, "   TaskName\tState\tPrio.\tMin.Stack\tTaskID\r\n");
            for(UBaseType_t x = 0; x < uxArraySize; x++)
            {
                switch(pxTaskStatusArray[x].eCurrentState)
                {
                    case eRunning: cStatus = tskRUNNING_CHAR; break;
                    case eReady: cStatus = tskREADY_CHAR; break;
                    case eBlocked: cStatus = tskBLOCKED_CHAR; break;
                    case eSuspended: cStatus = tskSUSPENDED_CHAR; break;
                    case eDeleted: cStatus = tskDELETED_CHAR; break;
                    case eInvalid:
                    default:
                        break;
                }
                uart->Print(false, " - %s", pxTaskStatusArray[x].pcTaskName);
#if configGENERATE_RUN_TIME_STATS == 1
                if(ulTotalRunTime > 0)
                {
                    ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;
                    if(ulStatsAsPercentage > 0UL)
                    {
                        uart->Print(false, "\t%c\t%u\t%u\t%lu%%\t%u\r\n", cStatus,
                            (uint32_t)pxTaskStatusArray[x].uxCurrentPriority,
                            (uint32_t)pxTaskStatusArray[x].usStackHighWaterMark,
                            ulStatsAsPercentage,
                            (uint32_t)pxTaskStatusArray[x].xTaskNumber);
                    }
                    else
                    {
                        uart->Print(false, "\t%c\t%u\t%u\t<1%%\t%u\r\n", cStatus,
                            (uint32_t)pxTaskStatusArray[x].uxCurrentPriority,
                            (uint32_t)pxTaskStatusArray[x].usStackHighWaterMark,
                            (uint32_t)pxTaskStatusArray[x].xTaskNumber);
                    }
                }
                else
#endif
                uart->Print(false, "\t%c\t%u\t%u\t%u\r\n", cStatus,
                            (uint32_t)pxTaskStatusArray[x].uxCurrentPriority,
                            (uint32_t)pxTaskStatusArray[x].usStackHighWaterMark,
                            (uint32_t)pxTaskStatusArray[x].xTaskNumber);
            }
            uart->StartTransmit(false);
        }
        vPortFree(pxTaskStatusArray);
#endif
        float mem_usage_now = 1.0f - ((float)xPortGetFreeHeapSize() / (float)(configTOTAL_HEAP_SIZE));
        float mem_usage_max = 1.0f - ((float)xPortGetMinimumEverFreeHeapSize() / (float)(configTOTAL_HEAP_SIZE));
        GenerateResponse(use_checksum, true, "Compile Time: %d-%d-%d %d:%d", YEAR(), MONTH(), DAY(), HOUR(), MINUTE());
        // Uptime is represented as: X hours, Y mins, Z seconds
        auto uptime_sec = HAL::GetUptimeSeconds();
        uint32_t uptime_hour = uptime_sec / 3600;
        uint8_t uptime_min = (uptime_sec % 3600) / 60;
        uint8_t remaining_sec = uptime_sec % 60;
        GenerateResponse(use_checksum, true, "Uptime: %dh%dm%ds", uptime_hour, uptime_min, remaining_sec);
        if constexpr(std::is_same_v<Motor, FOCMotor>)
        {
            GenerateResponse(use_checksum, true, "Task Times(us): RT:%d, RT<->REM.:%d, REM:%d, MID:%d", motor->task_times.rt_main_task.elapsed_time_us,
                                                                                                    motor->task_times.rt_waiting_for_remaining.elapsed_time_us,
                                                                                                    motor->task_times.rt_remaining_task.elapsed_time_us,
                                                                                                    motor->task_times.mid_interval_task.elapsed_time_us);
        }
        else if constexpr(std::is_same_v<Motor, DCMotor>)
        {
            GenerateResponse(use_checksum, true, "Task Times(us): RT:%d, MID:%d", motor->task_times.rt_main_task.elapsed_time_us,
                                                                                               motor->task_times.mid_interval_task.elapsed_time_us);
        }
        GenerateResponse(use_checksum, true, "Memory: %d/%d Bytes (%.2f,Max:%.2f)", configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize(),
                                                                                            configTOTAL_HEAP_SIZE,
                                                                                            mem_usage_now,
                                                                                            mem_usage_max);
#if defined(USE_FLASHDB)
        auto used_size = BoardConfig().GetNVMUsedSize();
        auto total_size = BoardConfig().GetNVMTotalSize();
        if(total_size > 0)
            GenerateResponse(use_checksum, true, "KVDB: %d/%d Bytes (%.2f)", used_size, total_size, ((float)used_size / (float)total_size));
#endif
        GenerateResponse(use_checksum, true, "Core clock: %d MHz", HAL::GetCoreClockHz() / 1000000);
        GenerateResponse(use_checksum, false, "HW S/N: %lu", HAL::GetSerialNumber());
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdMotorInfo(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    if(len < 3)
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    uint8_t dst = *(data + 2);
    if(!isdigit(dst) || (isdigit(dst) && dst - '0' + 1 > SYSTEM_MOTOR_NUM))
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    if(dst - '0' == motor->GetInternalID())
    {
        if(len < 4) // print motor info
        {
            if constexpr(std::is_same_v<Motor, FOCMotor>)
            {
                GenerateResponse(use_checksum, true, "FOCMotor %c (node_id:%d) Info: ", dst, motor->GetConfig().node_id());
                GenerateResponse(use_checksum, true, " - Vbus: %.1f", motor->GetBusSense()->voltage);
                GenerateResponse(use_checksum, true, " - Ibus: %.1f", motor->GetBusSense()->current);
                GenerateResponse(use_checksum, true, " - curr_state: %s", to_string(motor->state_machine.GetState()));
            }
            else if constexpr(std::is_same_v<Motor, DCMotor>)
            {
                GenerateResponse(use_checksum, true, "DCMotor %c (node_id:%d) Info: ", dst, motor->GetConfig().node_id());
                GenerateResponse(use_checksum, true, " - Vbus: %.1f", motor->GetBusSense()->voltage);
                GenerateResponse(use_checksum, true, " - Ibus: %.1f", motor->GetBusSense()->current);
            }
            else
            {
                return;
            }
            GenerateResponse(use_checksum, true, " - curr_ctrl_mode: %s", to_string(motor->GetControlMode()));
            String curr_error_idx_list{};
            uint64_t error = motor->GetError();
            uint8_t index = 0;
            while(error)
            {
                if(error & 0x01)
                {
                    append_uint_to_string(curr_error_idx_list, index);
                    curr_error_idx_list.append(",");
                }
                index++;
                error >>= 1LL;
            }
            GenerateResponse(use_checksum, false, " - curr_error: %s", curr_error_idx_list.c_str());
            const auto& current_motion = motor->GetCurrentMotionStruct(io_ref, io_torque_unit, io_speed_unit, io_pos_unit);
            const auto& current_target = motor->GetTargetMotionStruct(io_ref, io_torque_unit, io_speed_unit, io_pos_unit);
            GenerateResponse(use_checksum, false, " - curr/target: %.1f/%.1f, %.1f/%.1f, %.1f/%.1f", current_motion.torque.value,
                                                                                                            current_target.torque.value,
                                                                                                            current_motion.speed.value,
                                                                                                            current_target.speed.value,
                                                                                                            current_motion.pos.value,
                                                                                                            current_target.pos.value);
            const auto& encoders = motor->GetEncoders();
            if(encoders.size() > 0)
            {
                GenerateResponse(use_checksum, true, " - Encoders (%d):", encoders.size());
                for(size_t i = 0; i < encoders.size(); i++)
                {
                    const auto& enc = encoders[i];
                    switch(enc->GetEncoderType())
                    {
                        case Encoder::Type::ABSOLUTE_ENCODER: GenerateResponse(use_checksum, true, "    - #%d (Abs.) %d", i + 1, enc->IsResultValid()); break;
                        case Encoder::Type::INCREMENTAL_ENCODER: GenerateResponse(use_checksum, true, "    - #%d (Inc.) %d", i + 1, enc->IsResultValid()); break;
                        case Encoder::Type::SENSORLESS_ENCODER: GenerateResponse(use_checksum, true, "    - #%d (Est.) %d", i + 1, enc->IsResultValid()); break;
                        default: GenerateResponse(use_checksum, true, "    - #%d (N/A) %d", i + 1, enc->IsResultValid()); break;
                    }
                    GenerateResponse(use_checksum, true, "    - sing_rad: %.5f", enc->single_round_angle_rad);
                    GenerateResponse(use_checksum, false, "    - mult_rad: %.5f", enc->multi_round_angle_rad);
                }
            }
            // print RT & mid & normal tasks call list
            const auto& task_list = motor->GetTaskProcessor().GetTaskList();
            String rt_tasks_list{};
            String mid_tasks_list{};
            // Vector<std::pair<UBaseType_t, const char*>> normal_tasks_vector(task_list.size());
            for(const auto& task : task_list)
            {
                if(task->IsTaskRegistered(Task::TaskType::RT_TASK))
                {
                    if(!rt_tasks_list.empty()) rt_tasks_list.append(" -> ");
                    rt_tasks_list.append(task->GetName());
                }
                if(task->IsTaskRegistered(Task::TaskType::MID_TASK))
                {
                    if(!mid_tasks_list.empty()) mid_tasks_list.append(" -> ");
                    mid_tasks_list.append(task->GetName());
                }
                // if(task->IsTaskRegistered(Task::TaskType::NORMAL_TASK))
                // {
                //     normal_tasks_vector.emplace_back(task->GetRTOSPriority(), task->GetName());
                // }
            }
            if(!rt_tasks_list.empty()) GenerateResponse(use_checksum, false, " - RT Tasks: %s", rt_tasks_list.c_str());
            if(!mid_tasks_list.empty()) GenerateResponse(use_checksum, false, " - Mid Tasks: %s", mid_tasks_list.c_str());
            // sort the normal task by priority. (higher one is prioritized)
            // std::sort(normal_tasks_vector.begin(), normal_tasks_vector.end(), [](const std::pair<UBaseType_t, const char*>& a, const std::pair<UBaseType_t, const char*>& b){
            //     return a.first > b.first;
            // });
            // if(!normal_tasks_vector.empty())
            // {
            //     uart->Print(false, " - Normal Tasks: ");
            //     for(size_t i = 0; i < normal_tasks_vector.size(); i++)
            //     {
            //         const auto& pair = normal_tasks_vector[i];
            //         if(i == normal_tasks_vector.size() - 1) uart->Print(true, "%s(%d)\r\n", pair.second, pair.first);
            //         else uart->Print(false, "%s(%d) -> ", pair.second, pair.first);
            //     }
            // }
        }
        else // request a state change
        {
            uint8_t arg_len = len - 4;
            Vector<char> temp(arg_len + 1);
            memcpy(temp.data(), data + 4, arg_len);
            temp[arg_len] = '\0';
            MotorState req_state = MotorState::IDLE;
            MotorControlMode req_mode = MotorControlMode::CTRL_MODE_POSITION;
            if(from_string(temp.data(), req_state))
            {
                if constexpr(std::is_same_v<Motor, FOCMotor>)
                {
                    GenerateResponse(use_checksum, true, "Requested state: %s", temp.data());
                    GenerateResponse(use_checksum, false, "New state: %s", to_string(motor->state_machine.RequestState(req_state)));
                }
            }
            else if(from_string(temp.data(), req_mode))
            {
                motor->SetControlMode(req_mode);
                GenerateResponse(use_checksum, true, "Requested mode: %s", temp.data());
                GenerateResponse(use_checksum, false, "New mode: %s", to_string(motor->GetControlMode()));
            }
            else GenerateResponse(use_checksum, false, "invalid state input");
        }
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdHelp(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    // For help commands, only first motor instance will respond.
    if(motor->GetInternalID() == 0)
    {
        GenerateResponse(use_checksum, true,  "Avail cmds syntax, <optional>:");
        GenerateResponse(use_checksum, true,  " - Trajectory: t motor_id pos <s_curve>");
        GenerateResponse(use_checksum, true,  " - Position: q motor_id pos <vel-lim> <curr-lim>");
        GenerateResponse(use_checksum, true,  " - Position: p motor_id pos <vel-ff> <curr-ff>");
        GenerateResponse(use_checksum, true,  " - Velocity: v motor_id vel <curr-ff>");
        GenerateResponse(use_checksum, true,  " - Velocity (Open loop): o motor_id vel torque time_sec");
        GenerateResponse(use_checksum, true,  " - Torque: c motor_id set_torque");
        GenerateResponse(use_checksum, true,  " - Feedback: f motor_id");
        GenerateResponse(use_checksum, true,  " - Update Watchdog: u motor_id");
        GenerateResponse(use_checksum, true,  " - Read config: r motor_id/b <property>");
        GenerateResponse(use_checksum, true,  " - Write config: w motor_id/b property float_value");
        GenerateResponse(use_checksum, true,  " - System info: i");
        GenerateResponse(use_checksum, true,  " - Motor info: m motor_id <request_state/mode>");
        GenerateResponse(use_checksum, true,  " - Save cfg: ss motor_id/b");
        GenerateResponse(use_checksum, true,  " - Erase cfg: se motor_id/b (erase only)");
        GenerateResponse(use_checksum, true,  " - Set I/O ref: sf motor_id Ref TorqueUnit SpeedUnit PosUnit");
        GenerateResponse(use_checksum, true,  " - Clear error: sc motor_id <error_idx_1 idx_2...>");
        GenerateResponse(use_checksum, false, " - Reboot: sr");
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdSystem(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    if(len < 2)
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return;
    }
    switch(*(data + 1))
    {
        case 's': // Save cfg: ss motor_id / ss b
        {
            if(len != 4 || (!isdigit(*(data + 3)) && *(data + 3) != 'b') || (isdigit(*(data + 3)) && *(data + 3) - '0' + 1 > SYSTEM_MOTOR_NUM))
            {
                if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                return;
            }
            if constexpr(std::is_same_v<Motor, FOCMotor>)
            {
                if(motor->state_machine.GetState() != MotorState::IDLE) // must in IDLE state to save the config
                {
                    GenerateResponse(use_checksum, false, "state not in IDLE");
                    return;
                }
            }
            if(motor->GetInternalID() == 0)
            {
                if(*(data + 3) == 'b')
                {
                    auto ret = BoardConfig().SaveNVMConfig();
                    if(ret == FuncRetCode::OK) GenerateResponse(use_checksum, false, "board config save ok");
                    else GenerateResponse(use_checksum, false, "board config save failed:%d", to_underlying(ret));
                    return;
                }
            }
            if(*(data + 3) - '0' == motor->GetInternalID())
            {
                auto ret = motor->config.SaveNVMConfig();
                if(ret == FuncRetCode::OK) GenerateResponse(use_checksum, false, "motor %d config save ok", motor->GetInternalID());
                else GenerateResponse(use_checksum, false, "motor %d config save failed:%d", motor->GetInternalID(), to_underlying(ret));
                return;
            }
            break;
        }
        case 'e': // Erase cfg: se motor_id / se b (only erase, not save)
        {
            if(len != 4 || (!isdigit(*(data + 3)) && *(data + 3) != 'b') || (isdigit(*(data + 3)) && *(data + 3) - '0' + 1 > SYSTEM_MOTOR_NUM))
            {
                if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                return;
            }
            if constexpr(std::is_same_v<Motor, FOCMotor>)
            {
                if(motor->state_machine.GetState() != MotorState::IDLE) // must in IDLE state to erase the config
                {
                    GenerateResponse(use_checksum, false, "state not in IDLE");
                    return;
                }
            }
            if(motor->GetInternalID() == 0)
            {
                if(*(data + 3) == 'b')
                {
                    BoardConfig().GetConfig().clear();
                    GenerateResponse(use_checksum, false, "board config erase ok");
                    return;
                }
            }
            if(*(data + 3) - '0' == motor->GetInternalID())
            {
                motor->GetConfig().clear();
                GenerateResponse(use_checksum, false, "motor %d config erase ok", motor->GetInternalID());
                return;
            }
            break;
        }
        case 'c': // clear error
        {
            // if(len != 4)
            // {
            //     if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
            //     break;
            // }
            // if(*(data + 3) - '0' == motor->GetInternalID())
            // {
            //     motor->ClearError();
            //     GenerateResponse(use_checksum, false, "ok");
            // }
            if(len < 4)
            {
                if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                break;
            }
            if(*(data + 3) - '0' == motor->GetInternalID())
            {
                if(len == 4)
                {
                    motor->ClearError();
                    GenerateResponse(use_checksum, false, "ok");
                    break;
                }
                float target_idx[6]{};
                uint8_t target_count = 0;
                splitData_f((char*)(data + 5), len - 5, target_idx, &target_count, 6, ' ');
                if(target_count < 1)
                {
                    GenerateResponse(use_checksum, false, "invalid argument");
                    break;
                }
                String success_list{};
                for(uint8_t i = 0; i < target_count; i++)
                {
                    if(target_idx[i] < 0.0f || target_idx[i] > 63.0f) continue;
                    uint8_t uint_idx = (uint8_t)target_idx[i];
                    uint64_t temp_error = (1 << uint_idx);
                    if(motor->GetError() & temp_error)
                    {
                        motor->ClearError(temp_error);
                        append_uint_to_string(success_list, uint_idx);
                        success_list.append(",");
                    }
                }
                if(success_list.length() > 1)
                {
                    GenerateResponse(use_checksum, false, "cleared: %s", success_list.c_str());
                    break;
                }
                GenerateResponse(use_checksum, false, "error");
            }
            break;
        }
        case 'r': // Reboot: sr
        {
            if(motor->GetInternalID() == 0)
            {
                if(len == 2) HAL::SystemReboot();
                else GenerateResponse(use_checksum, false, "invalid command format");
            }
            break;
        }
        case 'f': // Set I/O Reference frame: sf motor_id Ref TorqueUnit SpeedUnit PosUnit
        {
            if(len < 4)
            {
                if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
                break;
            }
            if(*(data + 3) - '0' == motor->GetInternalID())
            {
                if(len < 6)
                {
                    GenerateResponse(use_checksum, true, "Set I/O ref: sf motor_id Ref TorqueUnit SpeedUnit PosUnit");
                    GenerateResponse(use_checksum, true, "usage: sf id ELEC/BASE/OUTPUT AMP/NM RADS/DEGS/REVS/RPM/HZ RAD/DEG/REV");
                    GenerateResponse(use_checksum, false, "Current ref:%s, torque:%s, speed:%s, pos: %s",
                                     to_string(io_ref),
                                     to_string(io_torque_unit),
                                     to_string(io_speed_unit),
                                     to_string(io_pos_unit));
                }
                else
                {
                    std::string_view view((const char*)(data + 5), len - 5);
                    Vector<String> result;
                    result.reserve(4);
                    size_t arg_count = splitStr(view, result, ' ');
                    if(arg_count != 4)
                    {
                        GenerateResponse(use_checksum, false, "invalid reference count: %d", arg_count);
                        break;
                    }
                    if(!from_string(result[0].c_str(), io_ref))
                    {
                        GenerateResponse(use_checksum, false, "invalid ref frame: %s", result[0].c_str());
                        break;
                    }
                    if(!from_string(result[1].c_str(), io_torque_unit))
                    {
                        GenerateResponse(use_checksum, false, "invalid torque unit: %s", result[1].c_str());
                        break;
                    }
                    if(!from_string(result[2].c_str(), io_speed_unit))
                    {
                        GenerateResponse(use_checksum, false, "invalid speed unit: %s", result[2].c_str());
                        break;
                    }
                    if(!from_string(result[3].c_str(), io_pos_unit))
                    {
                        GenerateResponse(use_checksum, false, "invalid pos unit: %s", result[3].c_str());
                        break;
                    }
                    GenerateResponse(use_checksum, false, "ok");
                }
            }
            break;
        }
        default:
        {
            if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
            break;
        }
    }
}

template <class Motor>
void ASCIIProtocol<Motor>::CmdUnknown(uint8_t* data, uint16_t len, bool use_checksum)
{
    const auto motor = GetInst();
    // For unknown commands, only first motor instance will respond.
    if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "unknown command");
}

template <class Motor>
bool ASCIIProtocol<Motor>::CheckAndGetID(uint8_t* data, uint16_t len, bool use_checksum, uint8_t expected_len,
    uint8_t& id)
{
    const auto motor = GetInst();
    id = 0;
    if(len < expected_len)
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return false;
    }
    uint8_t dst = *(data + 2);
    if(!isdigit(dst) || (isdigit(dst) && dst - '0' + 1 > SYSTEM_MOTOR_NUM) || *(data + 3) != ' ')
    {
        if(motor->GetInternalID() == 0) GenerateResponse(use_checksum, false, "invalid command format");
        return false;
    }
    id = dst - '0';
    return true;
}
}
