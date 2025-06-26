#pragma once

#include "../ThirdParty/EmbeddedProto/src/MessageInterface.h"
#include "buffer_helpers.hpp"
#include "Headers/Base/proto_header.h"

namespace iFOC::DataType
{
template<typename T>
concept ProtoMessage = std::is_base_of<::EmbeddedProto::MessageInterface, T>::value;

/*
 * @brief ProtoWrapper consists of a specific ProtoMessage type,
 * with a shared HW layer packet struct for read/write
 * HW layer packet:
 * |------ Header (2 bytes, uint16) ------| |
 * |-- Payload Length (2 bytes, uint16) --| |
 * |----- Protobuf Payload (n bytes) -----| |
 * |-- CRC16 Checksum (2 bytes, uint16) --| |
 *                                          V
 */
template<ProtoMessage msg_t>
class ProtoWrapper
{
    OVERRIDE_NEW();
public:
    explicit ProtoWrapper(Base::ProtoHeader _header) : header(static_cast<uint16_t>(_header)) {};
    msg_t& payload() { return p; };
    [[nodiscard]] uint8_t* GetBuffer() { return buffer; };
    [[nodiscard]] constexpr size_t GetBufferSize() { return MAX_PACKET_SIZE; }
    /// To raw buffer
    ::EmbeddedProto::Error Serialize(size_t& output_size);
    /// From raw buffer
    ::EmbeddedProto::Error Deserialize(size_t input_size);
private:
    static constexpr size_t MAX_MSG_SIZE = sizeof(msg_t) * 2;
    static constexpr size_t MAX_PACKET_SIZE = MAX_MSG_SIZE + sizeof(uint16_t) * 4;
    msg_t p;
    uint16_t header = static_cast<uint16_t>(Base::ProtoHeader::NOT_USED);
    uint8_t buffer[MAX_PACKET_SIZE]{};
};

template<ProtoMessage msg_t>
::EmbeddedProto::Error ProtoWrapper<msg_t>::Serialize(size_t& output_size)
{
    output_size = 0;
    _buffer_helper::ExternalBufferWriter writer(buffer + 4, MAX_MSG_SIZE);
    auto result = p.serialize(writer);
    if(result != EmbeddedProto::Error::NO_ERRORS) return result;

    uint16_t payload_length = writer.get_size();
    if(payload_length > MAX_MSG_SIZE) return EmbeddedProto::Error::BUFFER_FULL;

    buffer[0] = (uint8_t)header;
    buffer[1] = (uint8_t)(header >> 8);
    buffer[2] = (uint8_t)payload_length;
    buffer[3] = (uint8_t)(payload_length >> 8);

    uint16_t crc16 = get_crc16(buffer, payload_length + sizeof(uint16_t) * 2);

    buffer[4 + payload_length] = (uint8_t)crc16;
    buffer[4 + payload_length + 1] = (uint8_t)(crc16 >> 8);

    output_size = payload_length + sizeof(uint16_t) * 3;

    return EmbeddedProto::Error::NO_ERRORS;
}

template<ProtoMessage msg_t>
::EmbeddedProto::Error ProtoWrapper<msg_t>::Deserialize(const size_t input_size)
{
    if(input_size > MAX_PACKET_SIZE || input_size < sizeof(uint16_t) * 3) return EmbeddedProto::Error::END_OF_BUFFER;
    uint16_t rx_header = (uint16_t)buffer[1] << 8 | buffer[0];
    if(rx_header != header) return EmbeddedProto::Error::INVALID_WIRETYPE;
    uint16_t payload_length = (uint16_t)buffer[3] << 8 | buffer[2];
    if(payload_length > MAX_MSG_SIZE) return EmbeddedProto::Error::INDEX_OUT_OF_BOUND;
    uint16_t crc16 = get_crc16(buffer, payload_length + sizeof(uint16_t) * 2);
    uint16_t rx_crc16 = (uint16_t)buffer[4 + payload_length + 1] << 8 | buffer[4 + payload_length];
    if(crc16 != rx_crc16) return EmbeddedProto::Error::INVALID_FIELD_ID;

    _buffer_helper::ExternalBufferReader reader(buffer + 4, MAX_MSG_SIZE);
    reader.set_bytes_written(payload_length);

    // auto result = p.deserialize(reader);
    // return result;

    /// Ensure the deserialize process throws no exception, then memcpy the target payload
    // auto* temp_payload = (msg_t*)pvPortMalloc(sizeof(msg_t));
    // memset(temp_payload, 0, sizeof(msg_t));
    // auto result = temp_payload->deserialize(reader);
    // if(result == EmbeddedProto::Error::NO_ERRORS) memcpy(&p, temp_payload, sizeof(msg_t));
    // vPortFree(temp_payload);
    msg_t temp_payload;
    auto result = temp_payload.deserialize(reader);
    if(result == EmbeddedProto::Error::NO_ERRORS) memcpy(&p, &temp_payload, sizeof(msg_t));
    return result;
}

}
