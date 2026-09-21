#pragma once

#include "vision/DetectionMetadata.h"

#include <QByteArray>
#include <QString>
#include <QVector>

namespace rb::vision {

struct DetectionDecodeResult {
    QVector<DetectionFrame> frames;
    bool fatal{false};
    QString error;
};

class DetectionStreamDecoder final {
public:
    DetectionDecodeResult feed(const QByteArray &bytes);
    void reset();

    [[nodiscard]] bool hasPartialRecord() const noexcept
    {
        return !buffer_.isEmpty();
    }

private:
    QByteArray buffer_;
};

} // namespace rb::vision
