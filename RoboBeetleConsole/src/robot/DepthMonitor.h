#pragma once

#include "protocol/Packet.h"
#include "robot/DepthSnapshot.h"
#include "transport/ITransport.h"

#include <QObject>
#include <QString>

#include <optional>

namespace rb {

enum class DepthStatus {
    Unknown,
    Receiving,
    Stale,
    Error,
};

struct DepthMonitorState {
    DepthStatus status{DepthStatus::Unknown};
    std::optional<DepthSnapshot> snapshot;
    qint64 lastReceivedAtMs{-1};
    QString error;
};

class DepthMonitor final : public QObject {
    Q_OBJECT

public:
    static constexpr qint64 StaleTimeoutMs = 3500;

    explicit DepthMonitor(QObject *parent = nullptr) : QObject(parent) {}

    [[nodiscard]] const DepthMonitorState &state() const { return state_; }

    void handlePacket(const Packet &packet, qint64 receivedAtMs);
    void handleTransportState(TransportState state);
    void handleLivenessLost();
    void tick(qint64 nowMs);

signals:
    void changed();

private:
    void resetUnknown();

    DepthMonitorState state_;
};

QString depthStatusText(DepthStatus status);

} // namespace rb
