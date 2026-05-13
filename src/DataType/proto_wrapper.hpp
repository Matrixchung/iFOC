#pragma once

#include <pb_encode.h>
#include <pb_decode.h>
#include "Headers/Base/proto_header.h"
#include "../Common/foc_math.hpp"

namespace iFOC::DataType
{

template<typename T>
concept NanopbMessage = requires(T t) {
    { T::pb_fields() } -> std::convertible_to<const pb_msgdesc_t*>;
    { t.pb_data()    } -> std::convertible_to<void*>;
};

/*
 * ProtoWrapper<msg_t>
 *
 * Wraps a nanopb message with the on-wire packet framing:
 *   [Header (2 B)] [PayloadLen (2 B)] [Protobuf payload] [CRC16 (2 B)]
 */
template<NanopbMessage msg_t>
class ProtoWrapper
{
    OVERRIDE_NEW();
public:
    explicit ProtoWrapper(Base::ProtoHeader _header)
        : header(static_cast<uint16_t>(_header)) {}

    msg_t&   payload()                             { return p; }
    [[nodiscard]] uint8_t*       GetBuffer()       { return buffer; }
    [[nodiscard]] constexpr size_t GetBufferSize() { return MAX_PACKET_SIZE; }

    FuncRetCode Serialize(size_t& output_size);
    FuncRetCode Deserialize(size_t input_size);

private:
    static constexpr size_t MAX_MSG_SIZE    = sizeof(msg_t) * 2;
    static constexpr size_t MAX_PACKET_SIZE = MAX_MSG_SIZE + sizeof(uint16_t) * 4;

    msg_t    p;
    uint16_t header = static_cast<uint16_t>(Base::ProtoHeader::NOT_USED);
    uint8_t  buffer[MAX_PACKET_SIZE]{};
};

template<NanopbMessage msg_t>
FuncRetCode ProtoWrapper<msg_t>::Serialize(size_t& output_size)
{
    output_size = 0;

    pb_ostream_t stream = pb_ostream_from_buffer(
        reinterpret_cast<pb_byte_t*>(buffer + 4), MAX_MSG_SIZE);

    if (!pb_encode(&stream, msg_t::pb_fields(), p.pb_data()))
        return FuncRetCode::BUFFER_FULL;

    const uint16_t payload_length = static_cast<uint16_t>(stream.bytes_written);
    if (payload_length > MAX_MSG_SIZE)
        return FuncRetCode::BUFFER_FULL;

    buffer[0] = static_cast<uint8_t>(header);
    buffer[1] = static_cast<uint8_t>(header >> 8);
    buffer[2] = static_cast<uint8_t>(payload_length);
    buffer[3] = static_cast<uint8_t>(payload_length >> 8);

    const uint16_t crc16 = get_crc16(buffer, payload_length + sizeof(uint16_t) * 2);
    buffer[4 + payload_length]     = static_cast<uint8_t>(crc16);
    buffer[4 + payload_length + 1] = static_cast<uint8_t>(crc16 >> 8);

    output_size = payload_length + sizeof(uint16_t) * 3;
    return FuncRetCode::OK;
}

template<NanopbMessage msg_t>
FuncRetCode ProtoWrapper<msg_t>::Deserialize(const size_t input_size)
{
    if (input_size > MAX_PACKET_SIZE || input_size < sizeof(uint16_t) * 3)
        return FuncRetCode::BUFFER_FULL;

    const uint16_t rx_header = static_cast<uint16_t>(buffer[1]) << 8 | buffer[0];
    if (rx_header != header)
        return FuncRetCode::INVALID_INPUT;

    const uint16_t payload_length = static_cast<uint16_t>(buffer[3]) << 8 | buffer[2];
    if (payload_length > MAX_MSG_SIZE)
        return FuncRetCode::INVALID_INPUT;

    const uint16_t crc16    = get_crc16(buffer, payload_length + sizeof(uint16_t) * 2);
    const uint16_t rx_crc16 = static_cast<uint16_t>(buffer[4 + payload_length + 1]) << 8
                            | buffer[4 + payload_length];
    if (crc16 != rx_crc16)
        return FuncRetCode::CRC_MISMATCH;

    msg_t temp_payload{};
    pb_istream_t stream = pb_istream_from_buffer(
        reinterpret_cast<const pb_byte_t*>(buffer + 4), payload_length);

    if (!pb_decode(&stream, msg_t::pb_fields(), temp_payload.pb_data()))
        return FuncRetCode::BUFFER_FULL;

    memcpy(&p, &temp_payload, sizeof(msg_t));
    return FuncRetCode::OK;
}

} // namespace iFOC::DataType
