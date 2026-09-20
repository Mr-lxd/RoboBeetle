#include "vision/InferenceUiState.h"

#include <cstdio>
#include <cstdlib>
#include <functional>

namespace {

using namespace rb::vision;

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

VisionCaptureStatus manualStatus(const QString &state,
                                 bool cameraRunning = true,
                                 const QString &operation = {})
{
    VisionCaptureStatus status;
    status.haveCameraStatus = true;
    status.cameraRunning = cameraRunning;
    status.haveCaptureStatus = true;
    status.haveInferenceStatus = true;
    status.haveInferenceState = true;
    status.inferenceConfigured = true;
    status.inferenceControlSupported = true;
    status.inferenceOperationValid = true;
    status.inferenceOperation = operation;
    status.inferenceState = state;
    status.inferenceLastError = QStringLiteral("worker detail");
    status.haveInferenceFps = true;
    status.inferenceFps = 12.5;
    status.haveInferenceLatencyMs = true;
    status.inferenceLatencyMs = 3.0;
    status.haveInferenceDetectionCount = true;
    status.inferenceDetectionCount = 4;
    return status;
}

void initial_unknown_masks_everything()
{
    const auto state = makeInferenceUiState({}, false, false, false);
    expect(state.stateText == QStringLiteral("Inference Unknown"),
           "stale status renders Inference Unknown");
    expect(!state.startEnabled && !state.stopEnabled && !state.showActiveMetrics,
           "unknown status disables actions and metrics");
}

void unavailable_and_legacy_are_read_only()
{
    VisionCaptureStatus absent;
    absent.haveCameraStatus = true;
    absent.cameraRunning = true;
    const auto unavailable = makeInferenceUiState(absent, true, false, false);
    expect(unavailable.stateText == QStringLiteral("Inference Unavailable"),
           "missing inference object is unavailable");

    VisionCaptureStatus legacy = manualStatus(QStringLiteral("running"));
    legacy.inferenceConfigured.reset();
    legacy.inferenceControlSupported.reset();
    const auto readOnly = makeInferenceUiState(legacy, true, false, false);
    expect(readOnly.stateText == QStringLiteral("Inference Running")
               && !readOnly.startEnabled && !readOnly.stopEnabled,
           "legacy running status is read-only");
    expect(readOnly.showActiveMetrics && readOnly.reason.contains(QStringLiteral("read-only")),
           "legacy running keeps diagnostics but explains read-only controls");
}

void manual_worker_state_table_is_exact()
{
    const auto disabled = makeInferenceUiState(
        manualStatus(QStringLiteral("disabled")), true, false, false);
    expect(disabled.stateText == QStringLiteral("Inference Disabled")
               && disabled.startText == QStringLiteral("Start Inference")
               && disabled.startEnabled && !disabled.stopEnabled
               && !disabled.showActiveMetrics,
           "Disabled enables Start only with a running camera");

    const auto starting = makeInferenceUiState(
        manualStatus(QStringLiteral("starting")), true, false, false);
    expect(starting.stateText == QStringLiteral("Inference Starting")
               && !starting.startEnabled && starting.stopEnabled
               && !starting.showActiveMetrics,
           "Starting enables Stop and masks active metrics");

    const auto running = makeInferenceUiState(
        manualStatus(QStringLiteral("running")), true, false, false);
    expect(running.stateText == QStringLiteral("Inference Running")
               && !running.startEnabled && running.stopEnabled
               && running.showActiveMetrics,
           "Running enables Stop and shows active metrics");

    const auto failed = makeInferenceUiState(
        manualStatus(QStringLiteral("failed")), true, false, false);
    expect(failed.stateText == QStringLiteral("Inference Error")
               && failed.startText == QStringLiteral("Retry Inference")
               && failed.startEnabled && failed.stopEnabled
               && !failed.showActiveMetrics,
           "Failed maps to Retry and Clear Error");
}

void operation_precedence_and_error_are_visible()
{
    const auto stopping = makeInferenceUiState(
        manualStatus(QStringLiteral("disabled"), true, QStringLiteral("stopping")),
        true, false, false);
    expect(stopping.stateText == QStringLiteral("Inference Stopping")
               && !stopping.startEnabled && !stopping.stopEnabled,
           "stopping operation takes precedence over disabled worker state");

    const auto retrying = makeInferenceUiState(
        manualStatus(QStringLiteral("failed"), true, QStringLiteral("retrying")),
        true, false, false);
    expect(retrying.stateText == QStringLiteral("Inference Retrying")
               && !retrying.startEnabled && !retrying.stopEnabled,
           "retrying operation does not display ready controls");
    expect(retrying.reason.contains(QStringLiteral("worker detail")),
           "retained operation error is visible in the reason");
}

void camera_gate_only_affects_start_side()
{
    const auto disabled = makeInferenceUiState(
        manualStatus(QStringLiteral("disabled"), false), true, false, false);
    expect(!disabled.startEnabled && !disabled.stopEnabled,
           "disabled Start requires a confirmed running camera");

    const auto running = makeInferenceUiState(
        manualStatus(QStringLiteral("running"), false), true, false, false);
    expect(!running.startEnabled && running.stopEnabled,
           "Stop remains available without a running camera");

    const auto failed = makeInferenceUiState(
        manualStatus(QStringLiteral("failed"), false), true, false, false);
    expect(!failed.startEnabled && failed.stopEnabled,
           "Clear Error remains available without a running camera");
}

void busy_and_reconcile_mask_actions_without_faking_state()
{
    const auto runningBusy = makeInferenceUiState(
        manualStatus(QStringLiteral("running")), true, true, false);
    expect(runningBusy.stateText == QStringLiteral("Inference Running")
               && !runningBusy.startEnabled && !runningBusy.stopEnabled
               && runningBusy.showActiveMetrics,
           "actionBusy masks buttons but retains confirmed state/metrics");

    const auto disabledReconciling = makeInferenceUiState(
        manualStatus(QStringLiteral("disabled")), true, false, true);
    expect(disabledReconciling.stateText == QStringLiteral("Inference Disabled")
               && !disabledReconciling.startEnabled && !disabledReconciling.stopEnabled,
           "reconciliation masks actions without claiming a new worker state");
}

void malformed_state_fails_closed()
{
    VisionCaptureStatus malformed = manualStatus(QStringLiteral("mystery"));
    const auto state = makeInferenceUiState(malformed, true, false, false);
    expect(state.stateText == QStringLiteral("Inference Unavailable")
               && !state.startEnabled && !state.stopEnabled && !state.showActiveMetrics,
           "unknown worker states fail closed");

    malformed.inferenceOperationValid = false;
    malformed.inferenceOperation.clear();
    malformed.inferenceState = QStringLiteral("running");
    const auto invalidOperation = makeInferenceUiState(malformed, true, false, false);
    expect(invalidOperation.stateText == QStringLiteral("Inference Unavailable"),
           "invalid operation metadata fails closed");
}

} // namespace

int main()
{
    initial_unknown_masks_everything();
    unavailable_and_legacy_are_read_only();
    manual_worker_state_table_is_exact();
    operation_precedence_and_error_are_visible();
    camera_gate_only_affects_start_side();
    busy_and_reconcile_mask_actions_without_faking_state();
    malformed_state_fails_closed();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
