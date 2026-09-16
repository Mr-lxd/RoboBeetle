#pragma once

#include "robobeetle/protocol/codec.hpp"

#include <cstddef>
#include <vector>

namespace robobeetle::protocol {

class StreamDecoder final {
public:
    static constexpr std::size_t MaxEncodedFrameSize = 96U;

    std::vector<DecodeResult> feed(const Bytes &bytes);
    void reset();

private:
    Bytes buffer_;
    bool discard_until_delimiter_{false};
};

} // namespace robobeetle::protocol
