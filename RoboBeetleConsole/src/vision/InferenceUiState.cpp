#include "vision/InferenceUiState.h"

namespace rb::vision {

namespace {

QString knownStateText(const QString &state)
{
    if (state == QStringLiteral("disabled")) {
        return QStringLiteral("Inference Disabled");
    }
    if (state == QStringLiteral("starting")) {
        return QStringLiteral("Inference Starting");
    }
    if (state == QStringLiteral("running")) {
        return QStringLiteral("Inference Running");
    }
    if (state == QStringLiteral("failed") || state == QStringLiteral("error")) {
        return QStringLiteral("Inference Error");
    }
    return QString();
}

void appendError(QString &reason, const VisionCaptureStatus &status)
{
    if (!status.inferenceLastError.isEmpty()) {
        if (!reason.isEmpty()) {
            reason += QStringLiteral(" ");
        }
        reason += status.inferenceLastError;
    }
}

} // namespace

InferenceUiState makeInferenceUiState(const VisionCaptureStatus &status,
                                      bool statusFresh,
                                      bool actionBusy,
                                      bool reconcilePending)
{
    InferenceUiState result;
    if (!statusFresh) {
        result.stateText = QStringLiteral("Inference Unknown");
        result.reason = QStringLiteral("No fresh Vision status is available");
        return result;
    }
    if (!status.haveInferenceStatus || !status.haveInferenceState) {
        result.stateText = QStringLiteral("Inference Unavailable");
        result.reason = QStringLiteral("The Vision status has no usable inference object");
        return result;
    }

    const QString stateText = knownStateText(status.inferenceState);
    if (stateText.isEmpty()) {
        result.stateText = QStringLiteral("Inference Unavailable");
        result.reason = QStringLiteral("The Vision worker state is unknown");
        return result;
    }

    if (status.inferenceOperationValid && !status.inferenceOperation.isEmpty()) {
        if (status.inferenceOperation == QStringLiteral("stopping")) {
            result.stateText = QStringLiteral("Inference Stopping");
        } else {
            result.stateText = QStringLiteral("Inference Retrying");
        }
        result.reason = QStringLiteral("Vision inference operation is %1")
            .arg(status.inferenceOperation);
        appendError(result.reason, status);
        return result;
    }

    const bool readOnly = !status.inferenceControlSupported.has_value()
        || !*status.inferenceControlSupported;
    if (readOnly) {
        result.stateText = stateText;
        result.reason = QStringLiteral("Inference control is read-only");
        result.showActiveMetrics = status.inferenceState == QStringLiteral("running")
            && status.inferenceOperation.isEmpty();
        appendError(result.reason, status);
        return result;
    }
    if (!status.inferenceConfigured.has_value() || !*status.inferenceConfigured
        || !status.inferenceOperationValid) {
        result.stateText = QStringLiteral("Inference Unavailable");
        result.reason = QStringLiteral("Manual inference control metadata is incomplete");
        appendError(result.reason, status);
        return result;
    }

    result.stateText = stateText;
    if (status.inferenceState == QStringLiteral("disabled")) {
        result.startEnabled = status.haveCameraStatus && status.cameraRunning;
    } else if (status.inferenceState == QStringLiteral("starting")) {
        result.stopEnabled = true;
    } else if (status.inferenceState == QStringLiteral("running")) {
        result.stopEnabled = true;
        result.showActiveMetrics = true;
    } else if (status.inferenceState == QStringLiteral("failed")) {
        result.startText = QStringLiteral("Retry Inference");
        result.startEnabled = status.haveCameraStatus && status.cameraRunning;
        result.stopText = QStringLiteral("Clear Error");
        result.stopEnabled = true;
    }
    appendError(result.reason, status);
    if (actionBusy || reconcilePending) {
        result.startEnabled = false;
        result.stopEnabled = false;
    }
    return result;
}

} // namespace rb::vision
