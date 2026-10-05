#pragma once

namespace rb::ui {

// Operator-facing Auto Follow state. Priority, highest first:
// FAULT > STOPPING > ARMED > DRY RUN > READY > NOT READY.
enum class AutoFollowState { DryRun, NotReady, Ready, Armed, Stopping, Fault };

struct AutoFollowStateInput {
    bool fault{false};               // latched alarm (STOP timeout, pose mismatch, ...)
    bool stopping{false};            // disarmed, automatic/operator STOP still unconfirmed
    bool armed{false};
    bool enabled{false};             // dispatch switch on and Remote RBRP backend
    bool readinessComplete{false};   // checklist: link+control, servos+pose, tracking
    bool eligible{false};            // VisualDispatchStateMachine::eligibility() == Ready
};

// READY means "pressing Arm will succeed": the checklist alone is not enough
// because arm() can still be refused (invalid config/time, unconfirmed sign).
[[nodiscard]] constexpr AutoFollowState classifyAutoFollowState(const AutoFollowStateInput &in) noexcept
{
    if (in.fault) return AutoFollowState::Fault;
    if (in.stopping) return AutoFollowState::Stopping;
    if (in.armed) return AutoFollowState::Armed;
    if (!in.enabled) return AutoFollowState::DryRun;
    return in.readinessComplete && in.eligible ? AutoFollowState::Ready
                                               : AutoFollowState::NotReady;
}

[[nodiscard]] constexpr const char *autoFollowStateText(AutoFollowState state) noexcept
{
    switch (state) {
    case AutoFollowState::DryRun: return "DRY RUN";
    case AutoFollowState::NotReady: return "NOT READY";
    case AutoFollowState::Ready: return "READY";
    case AutoFollowState::Armed: return "ARMED";
    case AutoFollowState::Stopping: return "STOPPING";
    case AutoFollowState::Fault: return "FAULT";
    }
    return "";
}

} // namespace rb::ui
