#include "robot/DepthMonitor.h"

#include <optional>

namespace rb {

void DepthMonitor::handlePacket(const Packet &packet, qint64 receivedAtMs)
{
    if (packet.type != MessageType::DepthSnapshot) {
        return;
    }

    QString detail;
    const std::optional<DepthSnapshot> decoded =
        DepthSnapshot::decodePayload(packet.payload, &detail);
    if (!decoded.has_value()) {
        state_.status = DepthStatus::Error;
        state_.snapshot.reset();
        state_.lastReceivedAtMs = -1;
        state_.error = detail;
        emit changed();
        return;
    }

    const bool sensorSampleIsCurrent =
        decoded->depthValid()
        && decoded->sampleAgeMs != DepthSnapshot::UnknownSampleAgeMs;
    state_.status = sensorSampleIsCurrent
        ? DepthStatus::Receiving
        : DepthStatus::Stale;
    state_.snapshot = decoded;
    state_.lastReceivedAtMs = receivedAtMs;
    state_.error = sensorSampleIsCurrent
        ? QString{}
        : QStringLiteral("Depth sensor sample is stale or unavailable");
    emit changed();
}

void DepthMonitor::handleTransportState(TransportState state)
{
    if (state != TransportState::Connected) {
        resetUnknown();
    }
}

void DepthMonitor::handleLivenessLost()
{
    resetUnknown();
}

void DepthMonitor::tick(qint64 nowMs)
{
    if (state_.status != DepthStatus::Receiving
        || state_.lastReceivedAtMs < 0
        || nowMs < state_.lastReceivedAtMs
        || nowMs - state_.lastReceivedAtMs < StaleTimeoutMs) {
        return;
    }

    state_.status = DepthStatus::Stale;
    state_.snapshot.reset();
    state_.error = QStringLiteral(
        "No DepthSnapshot received within the stale window");
    emit changed();
}

void DepthMonitor::resetUnknown()
{
    if (state_.status == DepthStatus::Unknown
        && !state_.snapshot.has_value()
        && state_.lastReceivedAtMs < 0
        && state_.error.isEmpty()) {
        return;
    }

    state_.status = DepthStatus::Unknown;
    state_.snapshot.reset();
    state_.lastReceivedAtMs = -1;
    state_.error.clear();
    emit changed();
}

QString depthStatusText(DepthStatus status)
{
    switch (status) {
    case DepthStatus::Unknown: return QStringLiteral("Unknown");
    case DepthStatus::Receiving: return QStringLiteral("Receiving");
    case DepthStatus::Stale: return QStringLiteral("Stale");
    case DepthStatus::Error: return QStringLiteral("Error");
    }
    return QStringLiteral("Unknown");
}

} // namespace rb
