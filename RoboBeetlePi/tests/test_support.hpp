#pragma once

#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

namespace rbp2_test {

extern int failures;

inline void expect(bool condition, const char *message)
{
    if (!condition) {
        ++failures;
        // Keep the RED/GREEN output deterministic and dependency-free.
        std::fprintf(stderr, "FAIL: %s\n", message);
    }
}

inline std::vector<std::uint8_t> bytes(std::initializer_list<std::uint8_t> values)
{
    return std::vector<std::uint8_t>(values);
}

inline std::vector<std::uint8_t> hex(const std::string &text)
{
    std::vector<std::uint8_t> result;
    result.reserve(text.size() / 2U);
    auto nibble = [](char value) -> std::uint8_t {
        if (value >= '0' && value <= '9') {
            return static_cast<std::uint8_t>(value - '0');
        }
        if (value >= 'a' && value <= 'f') {
            return static_cast<std::uint8_t>(value - 'a' + 10);
        }
        return static_cast<std::uint8_t>(value - 'A' + 10);
    };
    for (std::size_t index = 0; index < text.size(); index += 2U) {
        result.push_back(static_cast<std::uint8_t>(
            (nibble(text[index]) << 4U) | nibble(text[index + 1U])));
    }
    return result;
}

inline std::vector<std::uint8_t> without_delimiter(
    const std::vector<std::uint8_t> &wire)
{
    std::vector<std::uint8_t> body = wire;
    if (!body.empty() && body.back() == 0U) {
        body.pop_back();
    }
    return body;
}

inline std::vector<std::uint8_t> concat(
    const std::vector<std::uint8_t> &first,
    const std::vector<std::uint8_t> &second)
{
    std::vector<std::uint8_t> result = first;
    result.insert(result.end(), second.begin(), second.end());
    return result;
}

} // namespace rbp2_test
