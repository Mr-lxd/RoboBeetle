#include "vision/VideoView.h"

#include <QFontMetrics>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QRect>
#include <QStringList>

namespace rb::vision {

VideoView::VideoView(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(240, 160);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAutoFillBackground(false);
}

void VideoView::setFrame(
    const QImage &image,
    quint64 frameId,
    quint64 captureTimestampNs)
{
    if (image.isNull()) {
        return;
    }
    if (paintPending_ && !image_.isNull()) {
        ++replacedPendingFrames_;
    }
    image_ = image;
    if (detectionOverlay_ && detectionOverlay_->sourceSize != image_.size()) {
        clearDetectionOverlay();
    }
    frameId_ = frameId;
    captureTimestampNs_ = captureTimestampNs;
    paintPending_ = true;
    update();
}

void VideoView::clearFrame()
{
    image_ = QImage();
    frameId_ = 0U;
    captureTimestampNs_ = 0U;
    detectionOverlay_.reset();
    targetState_.reset();
    diagnosticState_ = DetectionDisplayState::Stale;
    paintPending_ = false;
    update();
}

void VideoView::setDetectionOverlay(const DetectionFrame &frame, const std::optional<TargetState> &selected)
{
    if (image_.isNull() || frame.sourceSize != image_.size()) {
        clearDetectionOverlay();
        return;
    }
    if (frame.detections.isEmpty()) {
        clearDetectionOverlay(DetectionDisplayState::NoTarget);
        return;
    }
    bool haveValidDetection = false;
    for (qsizetype i = 0; i < frame.detections.size(); ++i) {
        if (targetStateAt(frame, i)) { haveValidDetection = true; break; }
    }
    if (!haveValidDetection) {
        clearDetectionOverlay();
        return;
    }
    detectionOverlay_ = frame;
    targetState_ = selected && selected->frameId == frame.frameId
        && selected->captureTimestampNs == frame.captureTimestampNs
        && selected->sourceSize == frame.sourceSize ? selected : std::nullopt;
    diagnosticState_ = targetState_ ? DetectionDisplayState::Target : DetectionDisplayState::NoTarget;
    update();
}

void VideoView::clearDetectionOverlay(DetectionDisplayState reason)
{
    if (!detectionOverlay_.has_value() && diagnosticState_ == reason) {
        return;
    }
    detectionOverlay_.reset();
    targetState_.reset();
    diagnosticState_ = reason;
    update();
}

void VideoView::setVisualDiagnostic(const VisualDiagnosticSnapshot &snapshot)
{
    visualDiagnostic_ = snapshot;
    update();
}

void VideoView::setVisualDispatchPresentation(bool enabled, bool armed, const QString &status)
{
    dispatchEnabled_ = enabled;
    dispatchArmed_ = armed;
    dispatchStatus_ = status;
    update();
}

QString VideoView::visualDiagnosticText() const
{
    if (visualDiagnostic_) {
        const auto &s = *visualDiagnostic_;
        const auto &c = s.command;
        QStringList lines{
            dispatchEnabled_ ? QStringLiteral("VISION DISPATCH - %1").arg(dispatchArmed_ ? QStringLiteral("ARMED") : QStringLiteral("DISARMED"))
                             : QStringLiteral("VISION DRY_RUN - no motion output"),
            QStringLiteral("manual controls live | %1").arg(QString::fromLatin1(visualStateName(s.state)))};
        if (s.target) {
            const auto &t = *s.target;
            lines << QStringLiteral("%1 conf=%2 (associated)").arg(t.target.className)
                         .arg(t.target.confidence, 0, 'f', 3)
                  << QStringLiteral("u=%1 v=%2")
                         .arg(t.target.originalPoint.x(), 0, 'f', 1)
                         .arg(t.target.originalPoint.y(), 0, 'f', 1)
                  << QStringLiteral("ex=%1 ey=%2")
                         .arg(t.ex, 0, 'f', 3).arg(t.ey, 0, 'f', 3);
        } else {
            lines << QStringLiteral("u=-- v=--") << QStringLiteral("ex=-- ey=--");
        }
        lines << QStringLiteral("ex_f=%1 yaw=%2")
                     .arg(c.ex_f ? QString::number(*c.ex_f, 'f', 3) : QStringLiteral("--"))
                     .arg(c.yaw_cmd, 0, 'f', 3)
              << (dispatchEnabled_ ? QStringLiteral("PROPOSED: %1") : QStringLiteral("PROPOSED (not sent): %1"))
                     .arg(QString::fromLatin1(proposedCommandName(c.proposed)));
        if (s.axis != VisualAxisMode::Yaw)
            lines << QStringLiteral("axis=%1 ey_f=%2 pitch_sign=%3")
                         .arg(s.axis == VisualAxisMode::Pitch ? QStringLiteral("PITCH") : QStringLiteral("BOTH"))
                         .arg(c.ey_f ? QString::number(*c.ey_f, 'f', 3) : QStringLiteral("--"))
                         .arg(s.pitchSign);
        if (!dispatchEnabled_)
            lines << QStringLiteral("turn_sign=%1 (实机符号未验证)").arg(s.turnSign);
        if (s.awaitingVideo) { lines << QStringLiteral("waiting for video"); }
        QString lock = QStringLiteral("lock: %1").arg(QString::fromLatin1(associationStatusName(s.associationStatus)));
        if (s.associationStatus == AssociationStatus::Associated) {
            lock += QStringLiteral(" d=%1px").arg(s.associationDistancePx
                ? QString::number(*s.associationDistancePx, 'f', 1) : QStringLiteral("--"));
        } else if (s.associationStatus == AssociationStatus::Miss) {
            lock += QStringLiteral(" %1ms").arg(s.associationMissMs.value_or(0));
        }
        lines << lock;
        if (c.proposed == ProposedCommand::Hold) {
            lines << QStringLiteral("retained / holding: %1")
                         .arg(QString::fromLatin1(proposedCommandName(c.effective)));
        }
        if (c.dwellBlocked) { lines << QStringLiteral("min_dwell: holding proposal"); }
        if (c.waitingForSample) { lines << QStringLiteral("waiting for new target sample"); }
        if (dispatchEnabled_) lines << dispatchStatus_;
        return lines.join('\n');
    }
    if (!targetState_) {
        QString reason = QStringLiteral("STALE");
        if (diagnosticState_ == DetectionDisplayState::NoTarget) {
            reason = QStringLiteral("NO_TARGET");
        } else if (diagnosticState_ == DetectionDisplayState::InferenceOff) {
            reason = QStringLiteral("INFERENCE_OFF");
        }
        return QStringLiteral(
            "VISION DRY_RUN | %1\n"
            "selection: supplied target\n"
            "u=-- v=--\n"
            "ex=-- ey=--").arg(reason);
    }
    const TargetState &state = *targetState_;
    return QStringLiteral(
        "VISION DRY_RUN | TARGET\n"
        "%1 | conf=%2 (selected)\n"
        "u=%3 v=%4\n"
        "ex=%5 ey=%6")
        .arg(state.target.className)
        .arg(state.target.confidence, 0, 'f', 3)
        .arg(state.target.originalPoint.x(), 0, 'f', 1)
        .arg(state.target.originalPoint.y(), 0, 'f', 1)
        .arg(state.ex, 0, 'f', 3)
        .arg(state.ey, 0, 'f', 3);
}

void VideoView::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), palette().window());

    if (image_.isNull()) {
        return;
    }

    const QSize scaled = image_.size().scaled(size(), Qt::KeepAspectRatio);
    const QRect target(
        (width() - scaled.width()) / 2,
        (height() - scaled.height()) / 2,
        scaled.width(),
        scaled.height());
    painter.drawImage(target, image_);

    // This diagnostic path has no motion output. Reuse original-frame geometry.
    painter.save();
    painter.setClipRect(target);
    const QColor errorColor(0x00, 0xE5, 0xFF);
    painter.setPen(QPen(errorColor, 2));
    const QPointF center(
        target.left() + target.width() / 2.0,
        target.top() + target.height() / 2.0);
    if (targetState_) {
        const QPointF point = targetState_->target.originalPoint;
        painter.drawLine(center, QPointF(
            target.left() + point.x() * target.width() / image_.width(),
            target.top() + point.y() * target.height() / image_.height()));
    }
    constexpr int centerArm = 10;
    painter.drawLine(center - QPointF(centerArm, 0), center + QPointF(centerArm, 0));
    painter.drawLine(center - QPointF(0, centerArm), center + QPointF(0, centerArm));

    QFont diagnosticFont = font();
    diagnosticFont.setPointSizeF(9.0);
    painter.setFont(diagnosticFont);
    const QFontMetrics diagnosticMetrics(diagnosticFont);
    const QStringList lines = visualDiagnosticText().split('\n');
    int textWidth = 0;
    for (const QString &line : lines) {
        textWidth = qMax(textWidth, diagnosticMetrics.horizontalAdvance(line));
    }
    constexpr int padding = 5;
    constexpr int margin = 6;
    const int panelWidth = qMin(textWidth + 2 * padding, target.width() - 2 * margin);
    const int contentWidth = qMax(1, panelWidth - 2 * padding);
    const int titleHeight = diagnosticMetrics.boundingRect(
        QRect(0, 0, contentWidth, 1000), Qt::TextWordWrap, lines.front()).height();
    const QRect panel(
        target.right() - margin - panelWidth + 1,
        target.top() + margin,
        panelWidth,
        titleHeight + (static_cast<int>(lines.size()) - 1) * diagnosticMetrics.lineSpacing()
            + 2 * padding);
    painter.fillRect(panel, QColor(0, 0, 0, 190));
    painter.setPen(Qt::white);
    painter.drawText(QRect(panel.left() + padding, panel.top() + padding,
                          contentWidth, titleHeight), Qt::TextWordWrap, lines.front());
    for (qsizetype index = 1; index < lines.size(); ++index) {
        painter.drawText(
            panel.left() + padding,
            panel.top() + padding + titleHeight + diagnosticMetrics.ascent()
                + (static_cast<int>(index) - 1) * diagnosticMetrics.lineSpacing(),
            diagnosticMetrics.elidedText(
                lines[index], Qt::ElideRight, panelWidth - 2 * padding));
    }
    painter.restore();

    if (detectionOverlay_.has_value()
        && detectionOverlay_->sourceSize == image_.size()) {
        painter.save();
        painter.setClipRect(target);
        QFont overlayFont = font();
        overlayFont.setBold(true);
        overlayFont.setPointSizeF(qMax(9.0, overlayFont.pointSizeF()));
        painter.setFont(overlayFont);
        const QFontMetrics metrics(overlayFont);
        const QColor overlayColor(0xFF, 0xB0, 0x00);
        constexpr int markerArmRadiusPx = 6;
        constexpr int markerStrokeWidthPx = 2;
        constexpr int markerDotRadiusPx = 3;
        constexpr int labelOffsetPx = 9;

        for (qsizetype index = 0; index < detectionOverlay_->detections.size(); ++index) {
            if (!targetStateAt(*detectionOverlay_, index)) continue;
            const auto &detection = detectionOverlay_->detections[index];
            const QString label = QStringLiteral("%1 %2")
                .arg(detection.className)
                .arg(detection.confidence, 0, 'f', 2);

            const double mappedX =
                static_cast<double>(target.left())
                + detection.originalPoint.x()
                    * static_cast<double>(target.width())
                    / static_cast<double>(image_.width());
            const double mappedY =
                static_cast<double>(target.top())
                + detection.originalPoint.y()
                    * static_cast<double>(target.height())
                    / static_cast<double>(image_.height());
            const QPoint marker(qRound(mappedX), qRound(mappedY));

            painter.setPen(QPen(
                overlayColor,
                markerStrokeWidthPx,
                Qt::SolidLine,
                Qt::SquareCap));
            painter.drawLine(
                marker.x() - markerArmRadiusPx,
                marker.y(),
                marker.x() + markerArmRadiusPx,
                marker.y());
            painter.drawLine(
                marker.x(),
                marker.y() - markerArmRadiusPx,
                marker.x(),
                marker.y() + markerArmRadiusPx);
            painter.setPen(Qt::NoPen);
            painter.setBrush(overlayColor);
            painter.drawEllipse(
                marker,
                markerDotRadiusPx,
                markerDotRadiusPx);
            painter.setBrush(Qt::NoBrush);

            const int textWidth = metrics.horizontalAdvance(label);
            const int minimumBaseline = target.top() + metrics.ascent();
            const int maximumBaseline = target.bottom() - metrics.descent();
            const int x = qBound(
                target.left(),
                marker.x() + labelOffsetPx,
                qMax(target.left(), target.right() - textWidth));
            const int baseline = qBound(
                minimumBaseline,
                marker.y() - labelOffsetPx,
                qMax(minimumBaseline, maximumBaseline));

            painter.setPen(QColor(0, 0, 0, 210));
            painter.drawText(x + 1, baseline + 1, label);
            painter.setPen(overlayColor);
            painter.drawText(x, baseline, label);
            if (targetState_ && visualDiagnostic_ && !visualDiagnostic_->awaitingVideo
                && visualDiagnostic_->frameId == detectionOverlay_->frameId
                && visualDiagnostic_->selectedDetectionIndex == index) {
                painter.setPen(QPen(errorColor, 2));
                painter.drawEllipse(QPointF(mappedX, mappedY), 11, 11);
                painter.drawText(marker + QPoint(12, 17), QStringLiteral("LOCK"));
            }
        }
        painter.restore();
    }

    paintPending_ = false;
    if (!fpsTimer_.isValid()) {
        fpsTimer_.start();
    }
    ++paintWindowFrames_;
    const qint64 elapsed = fpsTimer_.elapsed();
    if (elapsed >= 1000) {
        displayedFps_ =
            static_cast<double>(paintWindowFrames_) * 1000.0 /
            static_cast<double>(elapsed);
        paintWindowFrames_ = 0U;
        fpsTimer_.restart();
        emit displayedFpsChanged(displayedFps_);
    }
}

} // namespace rb::vision
