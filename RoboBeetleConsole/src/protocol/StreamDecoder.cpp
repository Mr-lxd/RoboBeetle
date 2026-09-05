#include "protocol/StreamDecoder.h"

#include "protocol/PacketCodec.h"

namespace rb {

QVector<DecodeResult> StreamDecoder::feed(QByteArrayView bytes)
{
    QVector<DecodeResult> results;
    for (const char byte : bytes) {
        if (byte == '\0') {
            if (discardUntilDelimiter_) {
                discardUntilDelimiter_ = false;
                buffer_.clear();
                continue;
            }
            if (!buffer_.isEmpty()) {
                results.append(PacketCodec::decodeWire(buffer_));
                buffer_.clear();
            }
            continue;
        }

        if (discardUntilDelimiter_) {
            continue;
        }
        buffer_.append(byte);
        if (buffer_.size() > MaxEncodedFrameSize) {
            DecodeResult overflow;
            overflow.error = DecodeError::InvalidLength;
            overflow.detail = QStringLiteral("Encoded frame exceeded the Phase 1 size limit");
            results.append(overflow);
            buffer_.clear();
            discardUntilDelimiter_ = true;
        }
    }
    return results;
}

void StreamDecoder::reset()
{
    buffer_.clear();
    discardUntilDelimiter_ = false;
}

} // namespace rb

