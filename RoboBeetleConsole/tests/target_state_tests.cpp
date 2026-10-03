#include "vision/TargetState.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {

using rb::vision::DetectionFrame;
using rb::vision::DetectionObservation;
using rb::vision::selectTargetState;

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

DetectionFrame frameAt(QPointF point, QSize size = QSize(640, 480))
{
    return DetectionFrame{
        42U, 1'000'000U, size,
        {DetectionObservation{2, QStringLiteral("fish"), 0.88, point}}};
}

void errorSignsAndScale()
{
    struct Case { QPointF point; double ex; double ey; };
    const Case cases[] = {
        {{160.0, 240.0}, -0.5, 0.0},
        {{320.0, 240.0}, 0.0, 0.0},
        {{480.0, 240.0}, 0.5, 0.0},
        {{320.0, 120.0}, 0.0, -0.5},
        {{320.0, 360.0}, 0.0, 0.5},
        {{0.0, 0.0}, -1.0, -1.0},
        {{639.0, 479.0}, 0.996875, 0.9958333333333333},
    };
    for (const auto &item : cases) {
        const auto state = selectTargetState(frameAt(item.point));
        expect(state.has_value(), "valid original-frame centroid yields a target state");
        if (!state) {
            continue;
        }
        expect(std::abs(state->ex - item.ex) < 1e-9,
               "horizontal error has the expected original-image sign and scale");
        expect(std::abs(state->ey - item.ey) < 1e-9,
               "vertical error is negative above and positive below image center");
        expect(state->frameId == 42U && state->captureTimestampNs == 1'000'000U
                   && state->sourceSize == QSize(640, 480)
                   && state->target.classId == 2
                   && state->target.className == QStringLiteral("fish")
                   && state->target.confidence == 0.88,
               "target state retains source frame provenance and selected observation");
    }

    const auto larger = selectTargetState(frameAt({960.0, 360.0}, {1280, 720}));
    expect(larger && std::abs(larger->ex - 0.5) < 1e-9
               && std::abs(larger->ey) < 1e-9,
           "normalization uses actual source dimensions rather than model or widget size");
}

void selectionAndTies()
{
    DetectionFrame frame = frameAt({160.0, 240.0});
    frame.detections.push_back({3, QStringLiteral("ray"), 0.94, {480.0, 120.0}});
    frame.detections.push_back({4, QStringLiteral("turtle"), 0.94, {320.0, 240.0}});
    const auto state = selectTargetState(frame);
    expect(state && state->target.classId == 3 && state->ex == 0.5 && state->ey == -0.5,
           "highest confidence wins and an equal-confidence tie keeps the first observation");
}

void invalidInputDoesNotProduceErrors()
{
    DetectionFrame empty = frameAt({320.0, 240.0});
    empty.detections.clear();
    expect(!selectTargetState(empty), "empty detections produce no target");
    for (const QSize size : {QSize(0, 480), QSize(640, 0), QSize(-1, 480)}) {
        expect(!selectTargetState(frameAt({0.0, 0.0}, size)),
               "zero or negative dimensions do not cause division by zero");
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const QPointF point : {
             QPointF(-1.0, 240.0), QPointF(640.0, 240.0),
             QPointF(320.0, -1.0), QPointF(320.0, 480.0),
             QPointF(nan, 240.0), QPointF(320.0, inf)}) {
        expect(!selectTargetState(frameAt(point)),
               "non-finite and out-of-image coordinates produce no target");
    }
    for (const double confidence : {-0.1, 1.1, nan, inf}) {
        auto frame = frameAt({320.0, 240.0});
        frame.detections.first().confidence = confidence;
        expect(!selectTargetState(frame), "invalid confidence produces no target");
    }
    auto frame = frameAt({nan, 240.0});
    frame.detections.push_back({1, QStringLiteral("ray"), 0.6, {480.0, 240.0}});
    const auto state = selectTargetState(frame);
    expect(state && state->target.classId == 1 && state->ex == 0.5,
           "an invalid candidate cannot hide a valid lower-confidence target");
}

} // namespace

int main()
{
    errorSignsAndScale();
    selectionAndTies();
    invalidInputDoesNotProduceErrors();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
