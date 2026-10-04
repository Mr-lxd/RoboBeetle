#pragma once

#include "vision/VisualPolicyConfig.h"

#include <cstdint>
#include <optional>

namespace rb::vision {

// This policy has no application/transport adapter. Only tests instantiate it.
struct DispatchRequest {
    std::uint64_t id;
    ProposedCommand command;
};

class VisualCommandSendPort {
public:
    virtual ~VisualCommandSendPort() = default;
    // Submission is not confirmation; the owner supplies the later outcome.
    virtual void send(const DispatchRequest &request) = 0;
};

struct VisualDispatchConfig {
    static constexpr std::int64_t minDwellMs = 1000;
    std::int64_t ackTimeoutMs{1000};
    std::uint16_t requiredServoMask{0x001b};
};

struct VisualDispatchInput {
    std::int64_t nowMs{0};
    bool linkConnected{false};
    bool controlOwned{false};
    std::uint16_t enabledMask{0};
    std::uint16_t poseKnownMask{0};
    std::optional<int> confirmedTurnSign; // No default; caller must confirm it.
    VisualState state{VisualState::Stale};
    ProposedCommand suggestion{ProposedCommand::Stop};
    std::optional<double> ex;
};

enum class ArmReason {
    Ready, InvalidConfig, InvalidTime, LinkDisconnected, ControlNotOwned,
    ServosNotEnabled, PoseUnknown, TurnSignUnconfirmed, NotTracking
};
[[nodiscard]] const char *armReasonName(ArmReason reason) noexcept;

enum class DispatchOutcome { Ok = 0, Busy = 7, Rejected, OutcomeUnknown };
enum class ManualInputKind { NonMotion, Motion, Stop };

class VisualDispatchStateMachine {
public:
    explicit VisualDispatchStateMachine(VisualCommandSendPort &port,
                                        VisualDispatchConfig config = {});
    [[nodiscard]] ArmReason arm(const VisualDispatchInput &input);
    void evaluate(const VisualDispatchInput &input);
    void acknowledge(std::uint64_t id, DispatchOutcome outcome, std::int64_t nowMs);
    void manualInput(ManualInputKind kind);

    [[nodiscard]] bool armed() const noexcept { return armed_; }
    [[nodiscard]] std::optional<ProposedCommand> currentMode() const noexcept { return currentMode_; }
    [[nodiscard]] bool stopTimeoutAlert() const noexcept { return stopTimeoutAlert_; }
    [[nodiscard]] unsigned stopTimeoutCount() const noexcept { return stopTimeoutCount_; }

private:
    struct PendingMotion { DispatchRequest request; std::int64_t sentMs; };
    [[nodiscard]] ArmReason eligibility(const VisualDispatchInput &input) const;
    [[nodiscard]] bool observeTime(std::int64_t nowMs);
    void beginStop(std::int64_t nowMs);
    void sendStop(std::int64_t nowMs);
    void retryStop(std::int64_t nowMs);
    void invalidateRequests();
    void failSafe(std::int64_t nowMs);

    VisualCommandSendPort &port_;
    VisualDispatchConfig config_;
    bool armed_{false};
    std::optional<ProposedCommand> currentMode_;
    bool modeConfirmed_{false};
    std::optional<std::int64_t> lastObservedMs_;
    std::optional<std::int64_t> lastNonStopSentMs_;
    std::optional<std::int64_t> lastStopAcceptedMs_;
    std::optional<PendingMotion> pendingMotion_;
    bool stopEpisode_{false};
    bool stopAwaiting_{false};
    // STOP retries are consecutive IDs: retain the entire episode in O(1).
    std::optional<std::uint64_t> firstStopId_;
    std::uint64_t lastStopId_{0};
    std::int64_t lastStopSentMs_{0};
    unsigned stopTimeoutCount_{0};
    bool stopTimeoutAlert_{false};
    std::uint64_t nextId_{1};
};

} // namespace rb::vision
