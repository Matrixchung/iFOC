#include "lookup_table.hpp"
#include <algorithm>

static float wrapToRange(const float x, const float min_v, const float max_v)
{
    const float period = max_v - min_v;
    if(period <= 0.0f) return min_v;

    float v = x - min_v;
    v = std::fmodf(v, period);
    if(v < 0.0f) v += period;
    return v + min_v;
}

namespace iFOC::DataType
{
static std::pair<Vector<std::pair<float, float>>::const_iterator,
                 Vector<std::pair<float, float>>::const_iterator>
        findNearestSamples(const float x, const Vector<std::pair<float, float>>& sorted_samples)
{
    if(x <= sorted_samples.front().first) return {sorted_samples.cbegin(), sorted_samples.cbegin()};
    if(x >= sorted_samples.back().first) return {--sorted_samples.cend(), --sorted_samples.cend()};
    for(auto it = sorted_samples.cbegin(); it != sorted_samples.cend(); ++it)
    {
        if(it->first >= x) return {it - 1, it};
    }
    return {sorted_samples.cbegin(), sorted_samples.cbegin()};
}

LookupTable::LookupTable(const size_t table_size,
                        const float input_min,
                        const float input_max) : input_min_(input_min), input_max_(input_max)
{
    if(table_size >= 1)
    {
        table.resize(table_size, 0.0f);
        step_ = (table_size == 1) ? 0.0f : (input_max - input_min) / (table_size - 1);
    }
}

float LookupTable::lookup(const float input) const
{
    if(table.size() == 0) return 0.0f;
    if(table.size() == 1) return table[0];
    if(input <= input_min_) return table.front();
    if(input >= input_max_) return table.back();
    const float float_index = (input - input_min_) / step_;
    const auto base_index = (size_t)floorf(float_index);
    if(base_index >= table.size() - 1) return table.back();
    const float fraction = float_index - (float)base_index;
    return table[base_index] * (1.0f - fraction) + table[base_index + 1] * fraction;
}

void LookupTable::fillFromSamples(const Vector<std::pair<float, float>>& samples)
{
    if(samples.empty()) return;

    Vector sorted_samples(samples);
    std::sort(sorted_samples.begin(), sorted_samples.end(),
        [](const std::pair<float, float>& a, const std::pair<float, float>& b)
        {
            return a.first < b.first;
        });

    bool has_valid_sample = false;
    for(const auto& s : sorted_samples)
    {
        if(s.first >= input_min_ && s.first <= input_max_)
        {
            has_valid_sample = true;
            break;
        }
    }
    if(!has_valid_sample) return;

    // iterate through current table
    for(size_t idx = 0; idx < table.size(); ++idx)
    {
        const float table_x = input_min_ + idx * step_;
        const auto [prev, next] = findNearestSamples(table_x, sorted_samples);
        if(prev == next) table[idx] = prev->second;
        else if(table_x == (float)prev->first) table[idx] = prev->second;
        else
        {
            const float t = (table_x - prev->first) / (next->first - prev->first);
            table[idx] = prev->second * (1.0f - t) + next->second * t;
        }
    }
}

float LookupTable::lookupPeriodic(float input) const
{
    if(table.empty()) return 0.0f;
    if(table.size() == 1) return table[0];

    const float period = input_max_ - input_min_;
    if(period <= 0.0f || step_ <= 0.0f) return table.front();

    // wrap input to [input_min_, input_max_)
    float x = input - input_min_;
    x = std::fmod(x, period);
    if(x < 0.0f) x += period;
    x += input_min_;

    const float float_index = (x - input_min_) / step_;
    size_t idx0 = (size_t)floorf(float_index);

    if(idx0 >= table.size() - 1) idx0 = table.size() - 2; // max idx0 == size - 2

    const size_t idx1 = idx0 + 1;
    float frac = float_index - (float)idx0;
    if(frac < 0.0f) frac = 0.0f;
    if(frac > 1.0f) frac = 1.0f;

    return table[idx0] * (1.0f - frac) + table[idx1] * frac;
}

void LookupTable::fillFromSamplesPeriodic(const Vector<std::pair<float, float>>& samples)
{
    if(samples.empty() || table.empty()) return;

    if(table.size() == 1)
    {
        table[0] = samples.front().second;
        return;
    }

    const float period = input_max_ - input_min_;
    if(period <= 0.0f || step_ <= 0.0f) return;

    // 1) 先把样本 x 归一化到 [input_min_, input_max_)
    Vector<std::pair<float, float>> normalized_samples;
    normalized_samples.reserve(samples.size());

    for(const auto& s : samples)
    {
        normalized_samples.emplace_back(
            wrapToRange(s.first, input_min_, input_max_),
            s.second
        );
    }

    // 2) 按 x 排序
    std::sort(normalized_samples.begin(), normalized_samples.end(),
        [](const std::pair<float, float>& a, const std::pair<float, float>& b)
        {
            return a.first < b.first;
        });

    // 3) 去重：避免 input_max 和 input_min 归一化后重合，或重复样本导致除零
    constexpr float EPS = 1e-6f;
    Vector<std::pair<float, float>> unique_samples;
    unique_samples.reserve(normalized_samples.size());

    for(const auto& s : normalized_samples)
    {
        if(unique_samples.empty() || std::fabs(unique_samples.back().first - s.first) > EPS)
        {
            unique_samples.push_back(s);
        }
        else
        {
            // 同 x 样本简单平均；也可以改成保留最后一个
            unique_samples.back().second =
                0.5f * (unique_samples.back().second + s.second);
        }
    }

    if(unique_samples.empty()) return;

    if(unique_samples.size() == 1)
    {
        std::fill(table.begin(), table.end(), unique_samples.front().second);
        return;
    }

    // 4) 填表
    // 因为当前表是闭区间表，最后一个点 table.back() 作为周期重复端点
    // 所以只填 [0, size-2]，最后强制 back = front
    for(size_t idx = 0; idx < table.size() - 1; ++idx)
    {
        const float table_x = input_min_ + (float)idx * step_;

        auto next = std::lower_bound(
            unique_samples.begin(),
            unique_samples.end(),
            table_x,
            [](const std::pair<float, float>& s, float x)
            {
                return s.first < x;
            });

        auto prev = next;

        float x0, y0, x1, y1, xq = table_x;

        if(next == unique_samples.begin())
        {
            // 落在“最后一个样本 -> 第一个样本+period”的环形区间
            prev = unique_samples.end() - 1;

            x0 = prev->first;
            y0 = prev->second;

            x1 = unique_samples.front().first + period;
            y1 = unique_samples.front().second;

            if(xq < x0) xq += period;
        }
        else if(next == unique_samples.end())
        {
            // 同样是环形尾段
            prev = unique_samples.end() - 1;

            x0 = prev->first;
            y0 = prev->second;

            x1 = unique_samples.front().first + period;
            y1 = unique_samples.front().second;

            if(xq < x0) xq += period;
        }
        else
        {
            prev = next - 1;

            x0 = prev->first;
            y0 = prev->second;

            x1 = next->first;
            y1 = next->second;
        }

        if(std::fabs(x1 - x0) <= EPS)
        {
            table[idx] = y0;
        }
        else
        {
            const float t = (xq - x0) / (x1 - x0);
            table[idx] = y0 * (1.0f - t) + y1 * t;
        }
    }
    // 5) 闭区间周期表的重复端点
    table.back() = table.front();
}

bool LookupTable::serialize(uint8_t* buffer, size_t& in_out_len) const
{
    const auto serialized_size = getSerializedSize();
    if(in_out_len < serialized_size) return false;

    uint8_t* ptr = buffer;
    in_out_len = 0;

    const uint32_t table_size = table.size();
    memcpy(ptr, &table_size, sizeof(uint32_t));
    ptr += sizeof(uint32_t);
    in_out_len += sizeof(uint32_t);

    memcpy(ptr, &input_min_, sizeof(float));
    ptr += sizeof(float);
    in_out_len += sizeof(float);

    memcpy(ptr, &input_max_, sizeof(float));
    ptr += sizeof(float);
    in_out_len += sizeof(float);

    for(size_t i = 0; i < table.size(); i++)
    {
        memcpy(ptr, &table[i], sizeof(float));
        ptr += sizeof(float);
        in_out_len += sizeof(float);
    }

    // calc CRC16
    const uint16_t calc_crc16 = get_crc16(buffer, in_out_len);
    memcpy(ptr, &calc_crc16, sizeof(uint16_t));
    in_out_len += sizeof(uint16_t);

    return in_out_len == serialized_size;
}

bool LookupTable::deserialize(const uint8_t* buffer, size_t len)
{
    if(len < sizeof(uint32_t) + sizeof(float) * 2 + sizeof(uint16_t)) return false;
    const uint32_t table_size = *(uint32_t*)buffer;
    if(len != sizeof(uint32_t) + sizeof(float) * 2 + sizeof(float) * table_size + sizeof(uint16_t)) return false;

    // check CRC16
    const uint16_t crc16 = *(uint16_t*)(buffer + sizeof(uint32_t) + sizeof(float) * 2 + sizeof(float) * table_size);
    if(crc16 != get_crc16(buffer, len - sizeof(uint16_t))) return false;

    // deserialize
    table.resize(table_size);
    table.shrink_to_fit();

    input_min_ = *(float*)(buffer + sizeof(uint32_t));
    input_max_ = *(float*)(buffer + sizeof(uint32_t) + sizeof(float));

    step_ = (table.size() <= 1) ? 0.0f : (input_max_ - input_min_) / (float)(table.size() - 1);

    const uint8_t* ptr = buffer + sizeof(uint32_t) + sizeof(float) * 2;

    for(size_t i = 0; i < table_size; i++)
    {
        memcpy(&table[i], ptr, sizeof(float));
        ptr += sizeof(float);
    }

    return true;
}

size_t LookupTable::getTableSize() const
{
    return table.size();
}

float LookupTable::getValueByIndex(const size_t index) const
{
    if(index >= table.size()) return 0.0f;
    return table[index];
}

void LookupTable::setValueByIndex(const size_t index, const float value)
{
    if(index >= table.size()) return;
    table[index] = value;
}

size_t LookupTable::getSerializedSize() const
{
    return sizeof(uint32_t) + sizeof(float) * 2 + sizeof(float) * table.size() + sizeof(uint16_t);
}

size_t LookupTable::getTableSizeBySerializedSize(size_t serialized_size)
{
    constexpr size_t header = sizeof(uint32_t) + sizeof(float) * 2 + sizeof(uint16_t);
    if(serialized_size <= header) return 0;
    serialized_size -= header;
    if(serialized_size % sizeof(float) != 0) return 0;
    return serialized_size / sizeof(float);
}
}
