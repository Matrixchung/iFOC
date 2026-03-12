#pragma once

#include "../Common/foc_types.hpp"
#include "../Common/foc_math.hpp"

namespace iFOC::DataType
{
class LookupTable
{
    OVERRIDE_NEW();
public:
    LookupTable() = default;
    LookupTable(size_t table_size, float input_min, float input_max);

    [[nodiscard]] float lookup(float input) const;
    void fillFromSamples(const Vector<std::pair<float, float>>& samples);

    [[nodiscard]] float lookupPeriodic(float input) const; // suitable for angle lookup
    void fillFromSamplesPeriodic(const Vector<std::pair<float, float>>& samples);

    // uint32_t size + float min + float max + (size * float value) + uint16_t CRC16
    bool serialize(uint8_t* buffer, size_t& in_out_len) const;
    bool deserialize(const uint8_t* buffer, size_t len);

    [[nodiscard]] size_t getTableSize() const;
    [[nodiscard]] float getValueByIndex(size_t index) const;
    void setValueByIndex(size_t index, float value);
    [[nodiscard]] size_t getSerializedSize() const;
    static size_t getTableSizeBySerializedSize(size_t serialized_size);
private:
    float input_min_ = 0.0f;
    float input_max_ = 0.0f;
    float step_ = 0.0f;
    Vector<float> table{};
};
}