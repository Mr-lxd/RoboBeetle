#include "robobeetle/protocol/cobs.hpp"

namespace robobeetle::protocol {

Bytes cobs_encode(const Bytes &input)
{
    Bytes output;
    output.reserve(input.size() + input.size() / 254U + 1U);
    output.push_back(0U);

    std::size_t code_index = 0U;
    Byte code = 1U;
    for (const Byte value : input) {
        if (value == 0U) {
            output[code_index] = code;
            code_index = output.size();
            output.push_back(0U);
            code = 1U;
            continue;
        }

        output.push_back(value);
        ++code;
        if (code == 0xFFU) {
            output[code_index] = code;
            code_index = output.size();
            output.push_back(0U);
            code = 1U;
        }
    }

    output[code_index] = code;
    return output;
}

bool cobs_decode(const Bytes &input, Bytes &output)
{
    output.clear();
    if (input.empty()) {
        return false;
    }

    std::size_t read_index = 0U;
    while (read_index < input.size()) {
        const Byte code = input[read_index++];
        if (code == 0U) {
            return false;
        }

        const std::size_t copy_count = static_cast<std::size_t>(code) - 1U;
        if (copy_count > input.size() - read_index) {
            return false;
        }

        output.insert(output.end(),
                      input.begin() + static_cast<std::ptrdiff_t>(read_index),
                      input.begin() + static_cast<std::ptrdiff_t>(read_index + copy_count));
        read_index += copy_count;
        if (code != 0xFFU && read_index < input.size()) {
            output.push_back(0U);
        }
    }
    return true;
}

} // namespace robobeetle::protocol
