#pragma once

#include "vision/VisionControlClient.h"

namespace rb::vision {

struct InferenceUiState {
    QString stateText;
    QString startText{QStringLiteral("Start Inference")};
    QString stopText{QStringLiteral("Stop Inference")};
    QString reason;
    bool startEnabled{false};
    bool stopEnabled{false};
    bool showActiveMetrics{false};
};

InferenceUiState makeInferenceUiState(const VisionCaptureStatus &status,
                                      bool statusFresh,
                                      bool actionBusy,
                                      bool reconcilePending);

} // namespace rb::vision
