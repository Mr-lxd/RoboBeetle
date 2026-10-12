#pragma once
#include "vision/VisualApproachPolicy.h"
#include <QString>
#include <QtGlobal>

namespace rb::vision
{
enum class VisualExecutionMode { Discrete, Proportional, Continuous };
struct VisualApproachCsvInfo {
    bool proportional{false}, dryRun{false};
    std::optional<double> area;
    QString state;
    std::optional<VisualSetpoint> setpoint;
    bool submitted{false};
    QString event, detail;
    qint64 nowMs{0};
    std::optional<quint64> frameId;
    quint64 sessionId{0};
    bool continuous{false};
};
struct VisualDispatchRecord {
    qint64 nowMs{0};
    quint64 policyId{0};
    std::optional<quint32> wireId;
    ProposedCommand command{ProposedCommand::Stop};
    QString result;
    quint8 rawResult{0xff};
    qint64 ackRttMs{-1};
    std::optional<VisualApproachCsvInfo> approach;
};
} // namespace rb::vision
