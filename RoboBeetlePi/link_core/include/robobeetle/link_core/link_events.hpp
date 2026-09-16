#pragma once

#include "robobeetle/protocol/frame.hpp"
#include "robobeetle/protocol/protocol_error.hpp"

#include <cstdint>
#include <optional>

namespace robobeetle::link_core {

enum class LinkState {
    Unconfirmed,
    Active,
    Degraded,
    Lost,
};

enum class LinkEventType {
    HeartbeatDispatched,
    OrdinaryDispatched,
    RequestAccepted,
    RequestRejected,
    RequestOutcomeUnknown,
    RequestCancelled,
    AckMalformed,
    AckIgnored,
    FrameReceived,
    UnknownMessage,
    DecodeError,
    StateChanged,
    TransportWriteFailed,
};

enum class AckDisposition {
    None,
    Unmatched,
    TypeMismatch,
    Duplicate,
    Late,
};

enum class OutcomeKind {
    None,
    Accepted,
    Rejected,
    OutcomeUnknown,
    Cancelled,
};

enum class SubmitStatus {
    Accepted,
    NotActive,
    PayloadTooLarge,
    QueueFull,
    TransportRejected,
};

struct SubmitResult {
    SubmitStatus status{SubmitStatus::NotActive};
    std::optional<std::uint16_t> sequence;
};

struct CommandOutcome {
    OutcomeKind kind{OutcomeKind::None};
    std::uint16_t sequence{0U};
    std::uint8_t request_type{0U};
    std::uint8_t result{0U};
};

struct LinkEvent {
    LinkEventType type{LinkEventType::FrameReceived};
    LinkState state{LinkState::Unconfirmed};
    AckDisposition ack_disposition{AckDisposition::None};
    CommandOutcome outcome;
    protocol::Frame frame;
    protocol::DecodeError decode_error{protocol::DecodeError::None};
};

} // namespace robobeetle::link_core
