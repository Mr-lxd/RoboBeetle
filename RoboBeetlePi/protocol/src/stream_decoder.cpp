#include "robobeetle/protocol/stream_decoder.hpp"

namespace robobeetle::protocol {

std::vector<DecodeResult> StreamDecoder::feed(const Bytes &bytes)
{
    std::vector<DecodeResult> results;
    for (const Byte value : bytes) {
        if (value == 0U) {
            if (discard_until_delimiter_) {
                discard_until_delimiter_ = false;
                buffer_.clear();
                continue;
            }
            if (!buffer_.empty()) {
                results.push_back(Codec::decodeWire(buffer_));
                buffer_.clear();
            }
            continue;
        }

        if (discard_until_delimiter_) {
            continue;
        }

        buffer_.push_back(value);
        if (buffer_.size() > MaxEncodedFrameSize) {
            DecodeResult overflow;
            overflow.error = DecodeError::InvalidLength;
            overflow.detail = "Encoded frame exceeded the receive size limit";
            results.push_back(overflow);
            buffer_.clear();
            discard_until_delimiter_ = true;
        }
    }
    return results;
}

void StreamDecoder::reset()
{
    buffer_.clear();
    discard_until_delimiter_ = false;
}

} // namespace robobeetle::protocol
