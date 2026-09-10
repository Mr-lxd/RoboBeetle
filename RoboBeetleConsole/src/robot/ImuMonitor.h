#pragma once

#include "protocol/Packet.h"
#include "robot/ImuSnapshot.h"
#include "transport/ITransport.h"

#include <QObject>
#include <QString>

#include <optional>

namespace rb {

enum class ImuStatus {
    Unknown,
    Receiving,
    Stale,
    Error,
};

struct ImuMonitorState {
    ImuStatus status{ImuStatus::Unknown};
    std::optional<ImuSnapshot> snapshot;
    qint64 lastReceivedAtMs{-1};
    QString error;
};

class ImuMonitor final : public QObject {
    Q_OBJECT

public:
    static constexpr qint64 StaleTimeoutMs = 3500;

    explicit ImuMonitor(QObject *parent = nullptr) : QObject(parent) {}

    [[nodiscard]] const ImuMonitorState &state() const { return state_; }

    void handlePacket(const Packet &packet, qint64 receivedAtMs);
    void handleTransportState(TransportState state);
    void handleLivenessLost();
    void tick(qint64 nowMs);

signals:
    void changed();

private:
    void resetUnknown();

    ImuMonitorState state_;
};

QString imuStatusText(ImuStatus status);

} // namespace rb
