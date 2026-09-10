#include "robot/ImuMonitor.h"

namespace rb {

void ImuMonitor::handlePacket(const Packet &packet, qint64 receivedAtMs)
{
    if (packet.type != MessageType::ImuSnapshot) {
        return;
    }

    QString detail;
    const std::optional<ImuSnapshot> decoded =
        ImuSnapshot::decodePayload(packet.payload, &detail);
    if (!decoded.has_value()) {
        state_.status = ImuStatus::Error;
        state_.snapshot.reset();
        state_.lastReceivedAtMs = -1;
        state_.error = detail;
        emit changed();
        return;
    }

    state_.status = ImuStatus::Receiving;
    state_.snapshot = decoded;
    state_.lastReceivedAtMs = receivedAtMs;
    state_.error.clear();
    emit changed();
}

void ImuMonitor::handleTransportState(TransportState state)
{
    if (state != TransportState::Connected) {
        resetUnknown();
    }
}

void ImuMonitor::handleLivenessLost()
{
    resetUnknown();
}

void ImuMonitor::tick(qint64 nowMs)
{
    if (state_.status != ImuStatus::Receiving
        || state_.lastReceivedAtMs < 0
        || nowMs - state_.lastReceivedAtMs < StaleTimeoutMs) {
        return;
    }

    state_.status = ImuStatus::Stale;
    state_.snapshot.reset();
    state_.error = QStringLiteral("No ImuSnapshot received within the stale window");
    emit changed();
}

void ImuMonitor::resetUnknown()
{
    if (state_.status == ImuStatus::Unknown
        && !state_.snapshot.has_value()
        && state_.lastReceivedAtMs < 0
        && state_.error.isEmpty()) {
        return;
    }
    state_.status = ImuStatus::Unknown;
    state_.snapshot.reset();
    state_.lastReceivedAtMs = -1;
    state_.error.clear();
    emit changed();
}

QString imuStatusText(ImuStatus status)
{
    switch (status) {
    case ImuStatus::Unknown: return QStringLiteral("Unknown");
    case ImuStatus::Receiving: return QStringLiteral("Receiving");
    case ImuStatus::Stale: return QStringLiteral("Stale");
    case ImuStatus::Error: return QStringLiteral("Error");
    }
    return QStringLiteral("Unknown");
}

} // namespace rb
