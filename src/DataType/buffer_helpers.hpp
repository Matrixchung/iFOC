#pragma once

#include <cstddef>
#include <span>
#include "../Common/foc_math.hpp"
#include "../Common/foc_types.hpp"
#include "../ThirdParty/EmbeddedProto/src/ReadBufferInterface.h"
#include "../ThirdParty/EmbeddedProto/src/WriteBufferInterface.h"

namespace iFOC
{
namespace _buffer_helper
{
class ExternalBufferReader final : public ::EmbeddedProto::ReadBufferInterface
{
    OVERRIDE_NEW();
public:
    ExternalBufferReader(uint8_t *ptr, size_t size) : buffer(ptr), BUFFER_SIZE(size) {};
    explicit ExternalBufferReader(std::span<uint8_t> sp) : buffer(sp.data()), BUFFER_SIZE(sp.size()) {};
    ~ExternalBufferReader() final = default;
    [[nodiscard]] uint32_t get_size() const final
    {
        return write_index_;
    }
    [[nodiscard]] uint32_t get_max_size() const final
    {
        return BUFFER_SIZE;
    }
    bool peek(uint8_t& byte) const final
    {
        const bool return_value = write_index_ > read_index_;
        if(return_value)
        {
            byte = buffer[read_index_];
        }
        return return_value;
    }
    bool advance() final
    {
        const bool return_value = write_index_ > read_index_;
        if(return_value)
        {
            ++read_index_;
        }
        return return_value;
    }
    bool advance(const uint32_t N) final
    {
        const uint32_t new_read_index = read_index_ + N;
        const bool return_value = write_index_ >= new_read_index;
        if(return_value)
        {
            read_index_ = new_read_index;
        }
        return return_value;
    }
    bool pop(uint8_t& byte) final
    {
        const bool return_value = write_index_ > read_index_;
        if(return_value)
        {
            byte = buffer[read_index_];
            ++read_index_;
        }
        return return_value;
    }
    void set_bytes_written(const uint32_t n_bytes)
    {
        write_index_ = MIN(n_bytes, BUFFER_SIZE);
    }
    void clear()
    {
        read_index_ = 0;
        write_index_ = 0;
    }
private:
    uint8_t *buffer = nullptr;
    size_t BUFFER_SIZE = 0;
    uint32_t write_index_ = 0;
    uint32_t read_index_ = 0;
};

class ExternalBufferWriter final : public ::EmbeddedProto::WriteBufferInterface
{
    OVERRIDE_NEW();
public:
    ExternalBufferWriter(uint8_t *ptr, size_t size) : buffer(ptr), BUFFER_SIZE(size) {};
    explicit ExternalBufferWriter(std::span<uint8_t> sp) : buffer(sp.data()), BUFFER_SIZE(sp.size()) {};
    ~ExternalBufferWriter() final = default;
    void clear() final
    {
        write_index_ = 0;
    }
    uint32_t get_size() const final
    {
        return write_index_;
    }
    uint32_t get_max_size() const final
    {
        return BUFFER_SIZE;
    }
    uint32_t get_available_size() const final
    {
        return BUFFER_SIZE - write_index_;
    }
    bool push(const uint8_t byte) final
    {
        bool return_value = BUFFER_SIZE > write_index_;
        if(return_value)
        {
            buffer[write_index_] = byte;
            ++write_index_;
        }
        return return_value;
    }
    bool push(const uint8_t* bytes, const uint32_t length) final
    {
        bool return_value = BUFFER_SIZE > (write_index_ + length);
        if(return_value)
        {
            memcpy(buffer + write_index_, bytes, length);
            write_index_ += length;
        }
        return return_value;
    }
private:
    uint8_t *buffer = nullptr;
    size_t BUFFER_SIZE = 0;
    uint32_t write_index_ = 0;
};

}
}