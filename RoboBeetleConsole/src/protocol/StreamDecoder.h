#pragma once

#include "protocol/Packet.h"

#include <QByteArray>
#include <QByteArrayView>
#include <QVector>

namespace rb {

class StreamDecoder final {
public:
    QVector<DecodeResult> feed(QByteArrayView bytes);
    void reset();

private:
    static constexpr qsizetype MaxEncodedFrameSize = 96;
    QByteArray buffer_;
    bool discardUntilDelimiter_{false};
};

} // namespace rb

