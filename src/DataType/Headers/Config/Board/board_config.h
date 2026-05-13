#pragma once

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif

#include "board_config.pb.h"
#include "../../../reflection.h"
#include "uart_protocol.h"
#include "uart_baudrate.h"

namespace iFOC {
namespace DataType {
namespace Config {

class BoardConfig
{
public:
    BoardConfig_data _d{};   /* MUST be first data member for reflection */

    BoardConfig() = default;

    REFLECT(
        MEMBER_SIZE_OFFSET(BoardConfig_data, pwm_wave_freq),
        MEMBER_SIZE_OFFSET(BoardConfig_data, speed_loop_freq),
        MEMBER_SIZE_OFFSET(BoardConfig_data, bus_overvoltage_limit),
        MEMBER_SIZE_OFFSET(BoardConfig_data, bus_undervoltage_limit),
        MEMBER_SIZE_OFFSET(BoardConfig_data, bus_max_positive_current),
        MEMBER_SIZE_OFFSET(BoardConfig_data, bus_max_negative_current),
        MEMBER_SIZE_OFFSET(BoardConfig_data, bus_sense_shunt_ohm),
        MEMBER_SIZE_OFFSET(BoardConfig_data, max_regen_current),
        MEMBER_SIZE_OFFSET(BoardConfig_data, current_sense_gain),
        MEMBER_SIZE_OFFSET(BoardConfig_data, current_sense_shunt_ohm),
        MEMBER_SIZE_OFFSET(BoardConfig_data, current_sense_f_lp),
        MEMBER_SIZE_OFFSET(BoardConfig_data, play_startup_tone),
        MEMBER_SIZE_OFFSET(BoardConfig_data, use_square_wave_tone),
        MEMBER_SIZE_OFFSET(BoardConfig_data, enable_can_terminal_resistor)
    )

    static const pb_msgdesc_t* pb_fields() { return BoardConfig_data_fields; }
    const void* pb_data() const             { return &_d; }
    void*       pb_data()                   { return &_d; }
    void        clear()                     { memset(&_d, 0, sizeof(_d)); }

    /* ── Inline getters / setters (zero code-size overhead) ── */
    iFOC::DataType::Comm::UARTProtocol uart_1_protocol()                      const { return static_cast<iFOC::DataType::Comm::UARTProtocol>(_d.uart_1_protocol); }
    iFOC::DataType::Comm::UARTProtocol get_uart_1_protocol()                   const { return static_cast<iFOC::DataType::Comm::UARTProtocol>(_d.uart_1_protocol); }
    void set_uart_1_protocol(iFOC::DataType::Comm::UARTProtocol v)                     { _d.uart_1_protocol = static_cast<int32_t>(v); }
    iFOC::DataType::Comm::UARTBaudrate uart_1_baudrate()                      const { return static_cast<iFOC::DataType::Comm::UARTBaudrate>(_d.uart_1_baudrate); }
    iFOC::DataType::Comm::UARTBaudrate get_uart_1_baudrate()                   const { return static_cast<iFOC::DataType::Comm::UARTBaudrate>(_d.uart_1_baudrate); }
    void set_uart_1_baudrate(iFOC::DataType::Comm::UARTBaudrate v)                     { _d.uart_1_baudrate = static_cast<int32_t>(v); }
    uint32_t pwm_wave_freq()                        const { return _d.pwm_wave_freq; }
    uint32_t get_pwm_wave_freq()                     const { return _d.pwm_wave_freq; }
    void set_pwm_wave_freq(uint32_t v)                       { _d.pwm_wave_freq = v; }
    uint32_t speed_loop_freq()                      const { return _d.speed_loop_freq; }
    uint32_t get_speed_loop_freq()                   const { return _d.speed_loop_freq; }
    void set_speed_loop_freq(uint32_t v)                     { _d.speed_loop_freq = v; }
    float bus_overvoltage_limit()                const { return _d.bus_overvoltage_limit; }
    float get_bus_overvoltage_limit()             const { return _d.bus_overvoltage_limit; }
    void set_bus_overvoltage_limit(float v)               { _d.bus_overvoltage_limit = v; }
    float bus_undervoltage_limit()               const { return _d.bus_undervoltage_limit; }
    float get_bus_undervoltage_limit()            const { return _d.bus_undervoltage_limit; }
    void set_bus_undervoltage_limit(float v)              { _d.bus_undervoltage_limit = v; }
    float bus_max_positive_current()             const { return _d.bus_max_positive_current; }
    float get_bus_max_positive_current()          const { return _d.bus_max_positive_current; }
    void set_bus_max_positive_current(float v)            { _d.bus_max_positive_current = v; }
    float bus_max_negative_current()             const { return _d.bus_max_negative_current; }
    float get_bus_max_negative_current()          const { return _d.bus_max_negative_current; }
    void set_bus_max_negative_current(float v)            { _d.bus_max_negative_current = v; }
    float bus_sense_shunt_ohm()                  const { return _d.bus_sense_shunt_ohm; }
    float get_bus_sense_shunt_ohm()               const { return _d.bus_sense_shunt_ohm; }
    void set_bus_sense_shunt_ohm(float v)                 { _d.bus_sense_shunt_ohm = v; }
    float max_regen_current()                    const { return _d.max_regen_current; }
    float get_max_regen_current()                 const { return _d.max_regen_current; }
    void set_max_regen_current(float v)                   { _d.max_regen_current = v; }
    float current_sense_gain()                   const { return _d.current_sense_gain; }
    float get_current_sense_gain()                const { return _d.current_sense_gain; }
    void set_current_sense_gain(float v)                  { _d.current_sense_gain = v; }
    float current_sense_shunt_ohm()              const { return _d.current_sense_shunt_ohm; }
    float get_current_sense_shunt_ohm()           const { return _d.current_sense_shunt_ohm; }
    void set_current_sense_shunt_ohm(float v)             { _d.current_sense_shunt_ohm = v; }
    uint32_t current_sense_f_lp()                   const { return _d.current_sense_f_lp; }
    uint32_t get_current_sense_f_lp()                const { return _d.current_sense_f_lp; }
    void set_current_sense_f_lp(uint32_t v)                  { _d.current_sense_f_lp = v; }
    bool play_startup_tone()                    const { return _d.play_startup_tone; }
    bool get_play_startup_tone()                 const { return _d.play_startup_tone; }
    void set_play_startup_tone(bool v)                   { _d.play_startup_tone = v; }
    bool use_square_wave_tone()                 const { return _d.use_square_wave_tone; }
    bool get_use_square_wave_tone()              const { return _d.use_square_wave_tone; }
    void set_use_square_wave_tone(bool v)                { _d.use_square_wave_tone = v; }
    bool enable_can_terminal_resistor()         const { return _d.enable_can_terminal_resistor; }
    bool get_enable_can_terminal_resistor()      const { return _d.enable_can_terminal_resistor; }
    void set_enable_can_terminal_resistor(bool v)        { _d.enable_can_terminal_resistor = v; }
};

} // namespace Config
} // namespace DataType
} // namespace iFOC

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
