#pragma once
#include "robobeetle/gateway/rbrp_codec.hpp"
#include "robobeetle/protocol/motion_state.hpp"
#include <QByteArray>

namespace rb {
using MotionStateSample = robobeetle::protocol::MotionStateSample;
using MotionStateBatch = robobeetle::protocol::MotionStateBatch;
using MotionStateTelemetry = robobeetle::gateway::GatewayMotionStateTelemetry;

inline robobeetle::protocol::Bytes motionStateBytes(const QByteArray &payload) {
    const auto *begin = reinterpret_cast<const std::uint8_t *>(payload.constData());
    return {begin, begin + payload.size()};
}
inline std::optional<MotionStateBatch> decodeMotionStateBatch(const QByteArray &payload) {
    return robobeetle::protocol::decode_motion_state_batch(motionStateBytes(payload));
}
inline std::optional<QByteArray> encodeMotionStateBatch(const MotionStateBatch &batch) {
    const auto p = robobeetle::protocol::encode_motion_state_batch(batch);
    if (!p)
        return std::nullopt;
    return QByteArray(reinterpret_cast<const char *>(p->data()), p->size());
}
inline std::optional<MotionStateTelemetry> decodeMotionStateTelemetry(const QByteArray &payload) {
    return robobeetle::gateway::decode_motion_state_telemetry(motionStateBytes(payload));
}
} // namespace rb
