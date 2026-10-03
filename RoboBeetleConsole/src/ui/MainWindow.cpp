#include "ui/MainWindow.h"
#include "ui/ElidedLabel.h"

#include "robot/ServoDescriptor.h"
#include "vision/DetectionClient.h"
#include "vision/VideoView.h"
#include "vision/VisionClient.h"
#include "vision/VisionControlClient.h"
#include "vision/InferenceUiState.h"
#include "vision/VisualDiagnosticSession.h"
#include "vision/VisualCsvLogger.h"

#include <QCloseEvent>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QFileDialog>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGuiApplication>
#include <QGridLayout>
#include <QGroupBox>
#include <QHostAddress>
#include <QKeyEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QMessageBox>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStyle>

#include <numeric>
#include <QScrollArea>
#include <QScreen>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QUrl>
#include <QTimer>

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>

namespace rb {
namespace {

QString formatAcc(const std::array<qint16, 3> &values)
{
    return QStringLiteral("%1, %2, %3 g")
        .arg(static_cast<double>(values[0]) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(values[1]) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(values[2]) / 1000.0, 0, 'f', 3);
}

QString formatGyro(const std::array<qint16, 3> &values)
{
    return QStringLiteral("%1, %2, %3 dps")
        .arg(static_cast<double>(values[0]) / 10.0, 0, 'f', 1)
        .arg(static_cast<double>(values[1]) / 10.0, 0, 'f', 1)
        .arg(static_cast<double>(values[2]) / 10.0, 0, 'f', 1);
}

QString formatAngle(const std::array<qint16, 3> &values)
{
    return QStringLiteral("%1, %2, %3 deg")
        .arg(static_cast<double>(values[0]) / 100.0, 0, 'f', 2)
        .arg(static_cast<double>(values[1]) / 100.0, 0, 'f', 2)
        .arg(static_cast<double>(values[2]) / 100.0, 0, 'f', 2);
}

QString formatDepth(qint32 depthMm)
{
    return QStringLiteral("%1 m")
        .arg(static_cast<double>(depthMm) / 1000.0, 0, 'f', 3);
}

QString formatTemperature(qint16 temperatureCentiC)
{
    return QStringLiteral("%1 C")
        .arg(static_cast<double>(temperatureCentiC) / 100.0, 0, 'f', 2);
}

quint16 servoPreviewAngleToPwm(const ServoDescriptor &descriptor,
                               qint16 angleCdeg)
{
    angleCdeg = qBound(descriptor.electricalMinAngleCdeg,
                       angleCdeg,
                       descriptor.electricalMaxAngleCdeg);
    if (angleCdeg < 0) {
        const qint32 denominator =
            -static_cast<qint32>(descriptor.electricalMinAngleCdeg);
        if (denominator <= 0) {
            return descriptor.neutralPwmUs;
        }
        const qint32 pulse = static_cast<qint32>(descriptor.neutralPwmUs)
            + (static_cast<qint32>(angleCdeg)
               * (static_cast<qint32>(descriptor.neutralPwmUs)
                  - static_cast<qint32>(descriptor.electricalMinPwmUs)))
                / denominator;
        return static_cast<quint16>(pulse);
    }

    const qint32 denominator =
        static_cast<qint32>(descriptor.electricalMaxAngleCdeg);
    if (denominator <= 0) {
        return descriptor.neutralPwmUs;
    }
    const qint32 pulse = static_cast<qint32>(descriptor.neutralPwmUs)
        + (static_cast<qint32>(angleCdeg)
           * (static_cast<qint32>(descriptor.electricalMaxPwmUs)
              - static_cast<qint32>(descriptor.neutralPwmUs)))
            / denominator;
    return static_cast<quint16>(pulse);
}

bool pulseBetween(quint16 pulse, quint16 endpoint, quint16 neutral)
{
    const quint16 low = std::min(endpoint, neutral);
    const quint16 high = std::max(endpoint, neutral);
    return pulse >= low && pulse <= high;
}

qint16 servoPreviewPwmToAngle(const ServoDescriptor &descriptor,
                              quint16 pulseUs)
{
    if (pulseUs == descriptor.neutralPwmUs) {
        return 0;
    }
    if (pulseBetween(pulseUs, descriptor.electricalMinPwmUs,
                     descriptor.neutralPwmUs)) {
        const qint32 denominator =
            static_cast<qint32>(descriptor.neutralPwmUs)
            - static_cast<qint32>(descriptor.electricalMinPwmUs);
        if (denominator != 0) {
            const qint32 angle =
                (static_cast<qint32>(pulseUs)
                 - static_cast<qint32>(descriptor.neutralPwmUs))
                * (-static_cast<qint32>(descriptor.electricalMinAngleCdeg))
                / denominator;
            return static_cast<qint16>(angle);
        }
    }
    if (pulseBetween(pulseUs, descriptor.electricalMaxPwmUs,
                     descriptor.neutralPwmUs)) {
        const qint32 denominator =
            static_cast<qint32>(descriptor.electricalMaxPwmUs)
            - static_cast<qint32>(descriptor.neutralPwmUs);
        if (denominator != 0) {
            const qint32 angle =
                (static_cast<qint32>(pulseUs)
                 - static_cast<qint32>(descriptor.neutralPwmUs))
                * static_cast<qint32>(descriptor.electricalMaxAngleCdeg)
                / denominator;
            return static_cast<qint16>(angle);
        }
    }
    return 0;
}

QString inferenceStateText(const QString &state)
{
    const QString normalized = state.trimmed().toLower();
    if (normalized == QStringLiteral("disabled")) {
        return QStringLiteral("Inference Disabled");
    }
    if (normalized == QStringLiteral("starting")) {
        return QStringLiteral("Inference STARTING");
    }
    if (normalized == QStringLiteral("running")) {
        return QStringLiteral("Inference RUNNING");
    }
    if (normalized == QStringLiteral("failed")
        || normalized == QStringLiteral("error")) {
        return QStringLiteral("Inference Error");
    }
    return QStringLiteral("Inference Unavailable");
}

QString optionalInferenceFrameId(bool haveValue, quint64 value)
{
    return haveValue ? QString::number(value) : QStringLiteral("--");
}

QString optionalInferenceDouble(bool haveValue, double value)
{
    return haveValue
        ? QString::number(value, 'f', 1)
        : QStringLiteral("--");
}

QString optionalInferenceUInt64(bool haveValue, quint64 value)
{
    return haveValue ? QString::number(value) : QStringLiteral("--");
}

QString optionalBool(const std::optional<bool> &value)
{
    if (!value.has_value()) {
        return QStringLiteral("--");
    }
    return *value ? QStringLiteral("true") : QStringLiteral("false");
}

QString bracketedEndpointHost(const QString &host)
{
    const QString trimmed = host.trimmed();
    if (trimmed.contains(QChar(':')) && !trimmed.startsWith(QChar('['))) {
        return QStringLiteral("[%1]").arg(trimmed);
    }
    return trimmed;
}

QString inferenceDetailsText(const vision::VisionCaptureStatus &status,
                             bool fresh,
                             const QString &reason)
{
    const QString freshness = fresh
        ? QStringLiteral("Fresh")
        : QStringLiteral("Last received / stale");
    const QString artifact = status.inferenceArtifactName.isEmpty()
        ? QStringLiteral("--")
        : status.inferenceArtifactName;
    const QString sha256 = status.inferenceModelSha256.isEmpty()
        ? QStringLiteral("--")
        : status.inferenceModelSha256;
    const QString state = status.haveInferenceState
        ? status.inferenceState
        : QStringLiteral("--");
    const QString operation = status.inferenceOperationValid
        ? (status.inferenceOperation.isEmpty()
               ? QStringLiteral("none")
               : status.inferenceOperation)
        : QStringLiteral("--");
    const QString lastError = status.inferenceLastError.isEmpty()
        ? QStringLiteral("--")
        : status.inferenceLastError;

    QString text = QStringLiteral("%1\nConfigured: %2 | Control supported: %3\n"
                                 "State: %4 | Operation: %5\nArtifact: %6\n"
                                 "SHA-256: %7\nConfidence threshold: %8\n"
                                 "FPS: %9 | Latency ms: %10\n"
                                 "Latest inference frame: %11\n"
                                 "Capture timestamp ns: %12\n"
                                 "Processed frames: %13 | Skipped frames: %14\n"
                                 "Detection count: %15\nLast error: %16\nReason: %17")
        .arg(freshness)
        .arg(optionalBool(status.inferenceConfigured))
        .arg(optionalBool(status.inferenceControlSupported))
        .arg(state)
        .arg(operation)
        .arg(artifact)
        .arg(sha256)
        .arg(status.haveInferenceConfidenceThreshold
                 ? QString::number(status.inferenceConfidenceThreshold, 'f', 2)
                 : QStringLiteral("--"))
        .arg(optionalInferenceDouble(
            status.haveInferenceFps, status.inferenceFps))
        .arg(optionalInferenceDouble(
            status.haveInferenceLatencyMs, status.inferenceLatencyMs));
    text = text.arg(optionalInferenceUInt64(
                        status.haveInferenceLatestFrame,
                        status.inferenceLatestFrameId))
        .arg(optionalInferenceUInt64(
                 status.haveInferenceCaptureTimestampNs,
                 status.inferenceCaptureTimestampNs))
        .arg(optionalInferenceUInt64(
                 status.haveInferenceProcessedFrames,
                 status.inferenceProcessedFrames))
        .arg(optionalInferenceUInt64(
                 status.haveInferenceSkippedFrames,
                 status.inferenceSkippedFrames))
        .arg(optionalInferenceUInt64(
                 status.haveInferenceDetectionCount,
                 status.inferenceDetectionCount))
        .arg(lastError)
        .arg(reason.isEmpty() ? QStringLiteral("--") : reason);
    return text;
}

QString inferencePerformanceText(const vision::VisionCaptureStatus &status,
                                 bool showActiveMetrics)
{
    if (!showActiveMetrics) {
        return QStringLiteral("-- FPS / -- ms");
    }
    return QStringLiteral("%1 FPS / %2 ms")
        .arg(optionalInferenceDouble(status.haveInferenceFps, status.inferenceFps))
        .arg(optionalInferenceDouble(status.haveInferenceLatencyMs,
                                     status.inferenceLatencyMs));
}

QString inferenceDetectionText(const vision::VisionCaptureStatus &status,
                               bool showActiveMetrics)
{
    return QStringLiteral("Detections %1")
        .arg(showActiveMetrics
                 ? optionalInferenceUInt64(status.haveInferenceDetectionCount,
                                           status.inferenceDetectionCount)
                 : QStringLiteral("--"));
}

QString inferenceMemoryText(const vision::VisionCaptureStatus &status,
                            bool showActiveMetrics)
{
    if (!showActiveMetrics) {
        return QStringLiteral("Memory -- / --");
    }
    const QString rss = status.haveVisionProcessRss
        ? QStringLiteral("%1 MiB")
              .arg(QString::number(static_cast<double>(status.visionProcessRssBytes)
                                        / (1024.0 * 1024.0),
                                    'f', 1))
        : QStringLiteral("--");
    const QString total = status.haveSystemTotalMemory
        ? QStringLiteral("%1 GiB")
              .arg(QString::number(static_cast<double>(status.systemTotalMemoryBytes)
                                        / (1024.0 * 1024.0 * 1024.0),
                                    'f', 1))
        : QStringLiteral("--");
    return QStringLiteral("Memory %1 / %2").arg(rss, total);
}

QString captureDetailsText(const vision::VisionCaptureStatus &status,
                           bool fresh)
{
    const QString freshness = fresh
        ? QStringLiteral("Fresh")
        : QStringLiteral("Last received / stale");
    const QString state = status.haveCaptureStatus
        ? status.state
        : QStringLiteral("--");
    const QString session = status.sessionId.isEmpty()
        ? QStringLiteral("--")
        : status.sessionId;
    const QString segment = status.segment.isEmpty()
        ? QStringLiteral("--")
        : status.segment;
    const QString freeDisk = status.haveFreeDisk
        ? QString::number(static_cast<double>(status.freeDiskBytes)
                              / (1024.0 * 1024.0 * 1024.0), 'f', 1)
        : QStringLiteral("--");
    const QString lastError = status.lastError.isEmpty()
        ? QStringLiteral("--")
        : status.lastError;
    return QStringLiteral("%1\nState: %2 | Recording: %3\nSession: %4 | Segment: %5\n"
                          "Recorded frames: %6 | Snapshots: %7\n"
                          "Queue bytes: %8 / %9\nFree disk GiB: %10\nLast error: %11")
        .arg(freshness)
        .arg(state)
        .arg(status.haveCaptureStatus
                 ? (status.recording ? QStringLiteral("true") : QStringLiteral("false"))
                 : QStringLiteral("--"))
        .arg(session)
        .arg(segment)
        .arg(status.haveCaptureStatus ? QString::number(status.recordedFrames)
                                      : QStringLiteral("--"))
        .arg(status.haveCaptureStatus ? QString::number(status.snapshotCount)
                                      : QStringLiteral("--"))
        .arg(status.haveCaptureStatus ? QString::number(status.queueBytes)
                                      : QStringLiteral("--"))
        .arg(status.haveCaptureStatus ? QString::number(status.maxQueueBytes)
                                      : QStringLiteral("--"))
        .arg(freeDisk)
        .arg(lastError);
}

QString normalizedHostCandidate(const QString &candidate)
{
    const QString value = candidate.trimmed();
    QHostAddress address;
    if (address.setAddress(value)) {
        return address.toString();
    }
    return value.toLower();
}

bool validHostSyntax(const QString &candidate)
{
    const QString value = candidate.trimmed();
    if (value.isEmpty()) {
        return false;
    }
    for (const QChar character : value) {
        if (character.isSpace()) {
            return false;
        }
    }
    if (value.contains(QStringLiteral("://"))
        || value.contains(QChar('/'))
        || value.contains(QChar('?'))
        || value.contains(QChar('#'))
        || value.contains(QChar('@'))) {
        return false;
    }
    QHostAddress address;
    if (address.setAddress(value)) {
        return true;
    }
    if (value.contains(QChar(':'))) {
        return false;
    }
    static const QRegularExpression hostnamePattern(
        QStringLiteral(R"(^[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?(?:\.[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?)*$)"));
    return hostnamePattern.match(value).hasMatch();
}

QString motionModeText(MotionMode mode)
{
    switch (mode) {
    case MotionMode::Stop: return QStringLiteral("Stop");
    case MotionMode::Forward: return QStringLiteral("Forward");
    case MotionMode::Backward: return QStringLiteral("Brake");
    case MotionMode::TurnLeft: return QStringLiteral("Turn Left");
    case MotionMode::TurnRight: return QStringLiteral("Turn Right");
    case MotionMode::Ascend: return QStringLiteral("Ascend");
    case MotionMode::Descend: return QStringLiteral("Descend");
    case MotionMode::Count: break;
    }
    return QStringLiteral("UNKNOWN");
}

void applySectionStyle(QGroupBox *box)
{
    box->setStyleSheet(QStringLiteral(
        "QGroupBox {"
        "  background: #F5F8FA;"
        "  border: 1px solid #C5D4E0;"
        "  border-radius: 8px;"
        "  margin-top: 0px;"
        "  padding-top: 0px;"
        "}"
        "QGroupBox::title {"
        "  color: transparent;"
        "  background: transparent;"
        "  padding: 0;"
        "}"
    ));
}

void applyCardStyle(QGroupBox *box)
{
    box->setStyleSheet(QStringLiteral(
        "QGroupBox {"
        "  background: #FFFFFF;"
        "  border: 1px solid #CCD9E4;"
        "  border-radius: 8px;"
        "  margin-top: 14px;"
        "  padding-top: 6px;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  left: 10px;"
        "  top: 1px;"
        "  padding: 0 5px;"
        "  background: #FFFFFF;"
        "  color: #29465F;"
        "  font-size: 12px;"
        "  font-weight: 600;"
        "}"
    ));
}

void applySubpanelStyle(QGroupBox *box)
{
    box->setStyleSheet(QStringLiteral(
        "QGroupBox {"
        "  background: #F5F8FA;"
        "  border: 1px solid #C5D4E0;"
        "  border-radius: 8px;"
        "  margin-top: 14px;"
        "  padding-top: 6px;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  left: 9px;"
        "  top: 1px;"
        "  padding: 0 5px;"
        "  background: #F5F8FA;"
        "  color: #334A5C;"
        "  font-size: 12px;"
        "  font-weight: 600;"
        "}"
    ));
}

void applyDashboardCardStyle(QGroupBox *box)
{
    box->setStyleSheet(QStringLiteral(
        "QGroupBox {"
        "  background: #FFFFFF;"
        "  border: 1px solid #C5D4E0;"
        "  border-radius: 8px;"
        "  margin-top: 0px;"
        "  padding-top: 0px;"
        "}"
        "QGroupBox::title {"
        "  color: transparent;"
        "  background: transparent;"
        "  padding: 0;"
        "}"
    ));
}

void addDashboardCardHeader(QVBoxLayout *layout,
                            QGroupBox *box,
                            const QString &text)
{
    auto *header = new QHBoxLayout;
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(7);

    auto *accent = new QWidget(box);
    accent->setFixedSize(3, 16);
    accent->setStyleSheet(QStringLiteral(
        "background: #2F80C9;"
        "border-radius: 1px;"
    ));
    header->addWidget(accent);

    auto *title = new QLabel(text, box);
    title->setStyleSheet(QStringLiteral(
        "color: #1F4058;"
        "font-size: 13px;"
        "font-weight: 700;"
    ));
    header->addWidget(title);
    header->addStretch();

    layout->addLayout(header);

    auto *divider = new QWidget(box);
    divider->setFixedHeight(1);
    divider->setStyleSheet(
        QStringLiteral("background: #E3EBF2;"));
    layout->addWidget(divider);
}

void addSectionHeader(QVBoxLayout *layout,
                      QWidget *parent,
                      const QString &text)
{
    auto *header = new QHBoxLayout;
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(7);

    auto *accent = new QWidget(parent);
    accent->setFixedSize(4, 18);
    accent->setStyleSheet(QStringLiteral(
        "background: #1976D2;"
        "border-radius: 2px;"
    ));
    header->addWidget(accent);

    auto *title = new QLabel(text, parent);
    title->setStyleSheet(QStringLiteral(
        "color: #18364F;"
        "font-size: 14px;"
        "font-weight: 700;"
    ));
    header->addWidget(title);
    header->addStretch();

    layout->addLayout(header);
}

} // namespace

MainWindow::MainWindow(
    IConsoleController *controller,
    vision::VisionClient *visionClient,
    vision::VisionControlClient *visionControlClient,
    vision::DetectionClient *detectionClient,
    QWidget *parent)
    : QMainWindow(parent),
      controller_(controller),
      visionClient_(visionClient),
      visionControlClient_(visionControlClient),
      detectionClient_(detectionClient)
{
    Q_ASSERT(controller_ != nullptr);
    committedPiHost_ = visionControlClient_ != nullptr
        && !visionControlClient_->host().trimmed().isEmpty()
        ? visionControlClient_->host().trimmed()
        : QStringLiteral("192.168.10.2");
    if (visionControlClient_ != nullptr && visionControlClient_->host().isEmpty()) {
        // This is a local endpoint initialization only; it does not issue IO.
        visionControlClient_->setEndpoint(
            committedPiHost_, visionControlClient_->port());
    }
    setWindowTitle(QStringLiteral("RoboBeetle Console"));
    setMinimumSize(1100, 720);
    const QScreen *startupScreen = QGuiApplication::primaryScreen();
    if (startupScreen != nullptr) {
        const QRect available = startupScreen->availableGeometry();
        const int initialWidth = qMax(1100, qMin(1420, available.width() - 32));
        const int initialHeight = qMax(720, qMin(1000, available.height() - 80));
        resize(initialWidth, initialHeight);
        const QSize frameSize = frameGeometry().size();
        move(available.center()
             - QPoint(frameSize.width() / 2, frameSize.height() / 2));
    } else {
        resize(1420, 1000);
    }

    auto *central = new QWidget(this);
    central->setObjectName(QStringLiteral("appCanvas"));
    central->setStyleSheet(
        QStringLiteral("#appCanvas { background: #E2EAF1; }"));
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    root->addWidget(createConnectionBar());
    operatorToolsPane_ = createOperatorTools();
    workspaceSplitter_ = new QSplitter(Qt::Vertical, central);
    workspaceSplitter_->setObjectName(QStringLiteral("workspaceSplitter"));
    workspaceSplitter_->setChildrenCollapsible(false);
    workspaceSplitter_->setHandleWidth(6);
    workspaceSplitter_->addWidget(createDashboard());
    workspaceSplitter_->addWidget(operatorToolsPane_);
    root->addWidget(workspaceSplitter_, 1);

    setCentralWidget(central);
    bindVisionUi();
    bindControllerUi();

    // Frozen Task 03 presentation tokens. Behavior and handlers remain local to
    // the existing controller/client objects; role properties are styling only.
    setStyleSheet(QStringLiteral(
        "QMainWindow { background: #E2EAF1; }"
        "QWidget { font-family: \"Segoe UI\"; font-size: 12px; color: #334A5C; }"
        "QTabWidget::pane { border: 1px solid #C5D4E0; border-radius: 8px; "
        "  background: #FFFFFF; top: -1px; }"
        "QTabBar::tab { background: #F5F8FA; border: 1px solid #C5D4E0; "
        "  border-bottom: none; border-top-left-radius: 5px; "
        "  border-top-right-radius: 5px; padding: 5px 12px; margin-right: 2px; "
        "  color: #667C8C; }"
        "QTabBar::tab:selected { background: #FFFFFF; color: #2F80C9; "
        "  font-weight: 700; border-top: 2px solid #2F80C9; }"
        "QTabBar::tab:hover:!selected { background: #E3EBF2; }"
        "QPushButton { background: #FFFFFF; border: 1px solid #C5D4E0; "
        "  border-radius: 5px; min-height: 30px; padding: 4px 12px; color: #334A5C; }"
        "QPushButton:hover { background: #F5F8FA; border-color: #81939F; }"
        "QPushButton:pressed { background: #E3EBF2; }"
        "QPushButton:disabled { color: #7F8F9C; background: #F1F4F6; "
        "  border-color: #CDD7E0; }"
        "QPushButton[consoleActionRole=primary] { background: #344B5B; color: #FFFFFF; border-color: #344B5B; font-weight: 700; }"
        "QPushButton[consoleActionRole=primary]:hover { background: #263B49; }"
        "QPushButton[consoleActionRole=primary]:pressed { background: #1B2E3A; }"
        "QPushButton[consoleActionRole=stop] { background: #E6EBEF; color: #334A5C; border-color: #81939F; font-weight: 700; }"
        "QPushButton[consoleActionRole=stop]:hover { background: #DCE4E9; }"
        "QPushButton[consoleActionRole=danger] { background: #C53F3F; color: #FFFFFF; border-color: #A93434; font-weight: 700; }"
        "QPushButton[consoleActionRole=danger]:disabled { background: #F1F4F6; color: #7F8F9C; border-color: #CDD7E0; }"
        "QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit { background: #FFFFFF; "
        "  border: 1px solid #C5D4E0; border-radius: 4px; padding: 3px 6px; "
        "  color: #334A5C; }"
        "QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus { "
        "  border-color: #2F80C9; }"
        "QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, "
        "QLineEdit:disabled { color: #7F8F9C; background: #F1F4F6; "
        "  border-color: #CDD7E0; }"
        "QSlider::groove:horizontal { height: 6px; background: #C5D4E0; "
        "  border-radius: 3px; }"
        "QSlider::handle:horizontal { width: 14px; margin: -4px 0; "
        "  background: #2F80C9; border-radius: 7px; }"
        "QSlider::handle:horizontal:disabled { background: #b8c1c9; }"
        "QPlainTextEdit { background: #fbfcfd; border: 1px solid #c4ccd4; "
        "  border-radius: 4px; color: #334A5C; }"));

    initializeWorkspaceSizes();

    setConnectedUi(false);
    setLeakUiState(controller_->leakState());
    setImuUiState(controller_->imuState());
    setDepthUiState(controller_->depthState());
    refreshMotionUi();
    refreshGaitSelectorsUi();
    refreshAuthorityUi();
    controller_->refreshSerialPorts();
}

QWidget *MainWindow::createConnectionBar()
{
    auto *bar = new QWidget(this);
    bar->setObjectName(QStringLiteral("topHeaderBar"));
    bar->setStyleSheet(QStringLiteral(
        "#topHeaderBar { background: #FFFFFF; border: 1px solid #C5D3DE; border-radius: 7px; }"));
    const bool remote =
        controller_->backendKind() == ConsoleBackendKind::RemoteRbrp;

    auto *row = new QHBoxLayout(bar);
    row->setContentsMargins(10, 6, 10, 6);
    row->setSpacing(3);

    auto *title = new QLabel(QStringLiteral("RoboBeetle"), bar);
    title->setObjectName(QStringLiteral("consoleTitle"));
    title->setVisible(false);

    auto addLabel = [bar, row](const QString &text) {
        auto *label = new QLabel(text, bar);
        label->setStyleSheet(QStringLiteral("color: #566B79; font-size: 11px; font-weight: 600;"));
        row->addWidget(label);
    };

    addLabel(QStringLiteral("Pi Host"));
    piHost_ = new QLineEdit(committedPiHost_, bar);
    piHost_->setObjectName(QStringLiteral("piHost"));
    piHost_->setPlaceholderText(QStringLiteral("Pi IP / hostname"));
    piHost_->setFixedWidth(130);
    piHost_->setFixedHeight(32);
    piHost_->installEventFilter(this);
    row->addWidget(piHost_);

    applyPiHostButton_ = new QPushButton(QStringLiteral("Applied"), bar);
    applyPiHostButton_->setObjectName(QStringLiteral("applyPiHostButton"));
    applyPiHostButton_->setProperty("consoleActionRole", "primary");
    applyPiHostButton_->setFixedSize(74, 32);
    applyPiHostButton_->setStyleSheet(QStringLiteral("min-height: 0px; max-height: 32px; padding: 2px 8px;"));
    row->addWidget(applyPiHostButton_);
    row->addSpacing(4);

    addLabel(QStringLiteral("Video"));
    visionPort_ = new QSpinBox(bar);
    visionPort_->setObjectName(QStringLiteral("visionPort"));
    visionPort_->setRange(1, 65535);
    visionPort_->setValue(47010);
    visionPort_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    visionPort_->setKeyboardTracking(false);
    visionPort_->setFixedSize(62, 32);
    row->addWidget(visionPort_);
    row->addSpacing(4);

    auto *refresh = new QPushButton(QStringLiteral("Refresh"), bar);
    refresh->setObjectName(QStringLiteral("serialRefreshButton"));
    refresh->setProperty("consoleActionRole", "secondary");
    refresh->setFixedHeight(32);
    refresh->setVisible(!remote);

    if (remote) {
        addLabel(QStringLiteral("Robot TCP"));
        robotTcpPort_ = new QSpinBox(bar);
        robotTcpPort_->setObjectName(QStringLiteral("robotTcpPort"));
        robotTcpPort_->setRange(1, 65535);
        robotTcpPort_->setValue(47000);
        robotTcpPort_->setButtonSymbols(QAbstractSpinBox::NoButtons);
        robotTcpPort_->setKeyboardTracking(false);
        robotTcpPort_->setFixedSize(62, 32);
        row->addWidget(robotTcpPort_);
    } else {
        addLabel(QStringLiteral("Serial"));
        serialPortCombo_ = new QComboBox(bar);
        serialPortCombo_->setObjectName(QStringLiteral("serialPortCombo"));
        serialPortCombo_->setEditable(true);
        serialPortCombo_->setMinimumWidth(110);
        serialPortCombo_->setFixedHeight(32);
        row->addWidget(serialPortCombo_);

        addLabel(QStringLiteral("Baud"));
        serialBaud_ = new QSpinBox(bar);
        serialBaud_->setObjectName(QStringLiteral("serialBaud"));
        serialBaud_->setRange(1200, 3000000);
        serialBaud_->setValue(9600);
        serialBaud_->setButtonSymbols(QAbstractSpinBox::NoButtons);
        serialBaud_->setKeyboardTracking(false);
        serialBaud_->setFixedSize(90, 32);
        row->addWidget(serialBaud_);

        refresh->setFixedWidth(70);
        row->addWidget(refresh);
    }

    connectButton_ = new QPushButton(QStringLiteral("Connect"), bar);
    connectButton_->setObjectName(QStringLiteral("connectRobotButton"));
    connectButton_->setProperty("consoleActionRole", "primary");
    connectButton_->setFixedSize(78, 32);
    connectButton_->setStyleSheet(QStringLiteral("min-height: 0px; max-height: 32px; padding: 2px 8px;"));
    row->addWidget(connectButton_);

    acquireButton_ = new QPushButton(QStringLiteral("Acquire"), bar);
    releaseButton_ = new QPushButton(QStringLiteral("Release"), bar);
    acquireButton_->setProperty("consoleActionRole", "secondary");
    releaseButton_->setProperty("consoleActionRole", "secondary");
    acquireButton_->setFixedSize(60, 32);
    releaseButton_->setFixedSize(60, 32);
    acquireButton_->setStyleSheet(QStringLiteral("min-height: 0px; max-height: 32px; padding: 2px 6px;"));
    releaseButton_->setStyleSheet(QStringLiteral("min-height: 0px; max-height: 32px; padding: 2px 6px;"));
    acquireButton_->setVisible(remote);
    releaseButton_->setVisible(remote);
    row->addWidget(acquireButton_);
    row->addWidget(releaseButton_);

    row->addStretch(1);
    auto *stateLabel = new QLabel(QStringLiteral("State"), bar);
    stateLabel->setStyleSheet(QStringLiteral(
        "color: #667C8C; font-size: 11px; font-weight: 600;"));
    row->addWidget(stateLabel);
    connectionStatus_ = new QLabel(QStringLiteral("Disconnected"), bar);
    connectionStatus_->setObjectName(QStringLiteral("connectionStatus"));
    connectionStatus_->setStyleSheet(
        QStringLiteral("font-size: 11px; font-weight: 700; color: #263238;"));
    row->addWidget(connectionStatus_);
    authorityStatus_ = new QLabel(
        remote ? QStringLiteral("Unowned") : QStringLiteral("Direct"), bar);
    authorityStatus_->setObjectName(QStringLiteral("authorityStatus"));
    authorityStatus_->setStyleSheet(
        QStringLiteral("font-size: 11px; font-weight: 700; color: #455A64;"));
    row->addWidget(authorityStatus_);

    // Kept as a non-visible compatibility/status helper only. The header no
    // longer spends a second row on "Active Pi" text.
    piHostHint_ = new QLabel(bar);
    piHostHint_->setObjectName(QStringLiteral("piHostHint"));
    piHostHint_->setVisible(false);

    connect(refresh, &QPushButton::clicked,
            controller_, &IConsoleController::refreshSerialPorts);
    connect(applyPiHostButton_, &QPushButton::clicked,
            this, &MainWindow::applyPiHost);
    connect(piHost_, &QLineEdit::returnPressed,
            this, &MainWindow::applyPiHost);
    connect(piHost_, &QLineEdit::textChanged,
            this, &MainWindow::refreshPiHostUi);
    connect(visionPort_, qOverload<int>(&QSpinBox::valueChanged),
            this, [this] { refreshVisionEndpointUi(); });

    if (robotTcpPort_ != nullptr) {
        connect(robotTcpPort_, qOverload<int>(&QSpinBox::valueChanged),
                this, [this] { refreshVisionEndpointUi(); });
    }
    connect(connectButton_, &QPushButton::clicked, this, [this] {
        if (controller_->isConnected()) {
            controller_->disconnectController();
            return;
        }
        ConsoleConnectionConfiguration configuration;
        configuration.endpoint =
            controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
                ? committedPiHost_
                : serialPortCombo_->currentText().trimmed();
        if (controller_->backendKind() == ConsoleBackendKind::RemoteRbrp) {
            configuration.tcpPort =
                static_cast<quint16>(robotTcpPort_->value());
        } else {
            configuration.baudRate = serialBaud_->value();
        }
        controller_->connectController(configuration);
    });
    connect(acquireButton_, &QPushButton::clicked,
            controller_, &IConsoleController::acquireControl);
    connect(releaseButton_, &QPushButton::clicked,
            controller_, &IConsoleController::releaseControl);
    return bar;
}

QWidget *MainWindow::createVideoPlaceholder()
{
    auto *box = new QGroupBox(QStringLiteral("Realtime Video"), this);
    box->setObjectName(QStringLiteral("videoCard"));
    videoCard_ = box;
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(8, 5, 8, 6);
    layout->setSpacing(5);

    auto *header = new QHBoxLayout;
    header->setContentsMargins(0, 0, 0, 0);
    auto *title = new QLabel(QStringLiteral("Realtime Video"), box);
    title->setStyleSheet(QStringLiteral(
        "color: #1F4058; font-size: 13px; font-weight: 700;"));
    header->addWidget(title);
    header->addStretch(1);
    layout->addLayout(header);

    videoContentHost_ = new QWidget(box);
    videoContentHost_->setObjectName(QStringLiteral("videoContentHost"));
    videoContentHost_->setMinimumHeight(180);
    videoContentHost_->installEventFilter(this);
    auto *content = new QHBoxLayout(videoContentHost_);
    content->setContentsMargins(0, 0, 0, 0);
    content->setSpacing(8);

    videoView_ = new vision::VideoView(videoContentHost_);
    videoView_->setObjectName(QStringLiteral("videoView"));
    videoView_->setStyleSheet(QStringLiteral(
        "#videoView { background: #101820; border: 1px solid #D5E0E8; border-radius: 6px; }"));
    videoView_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    content->addWidget(videoView_, 0, Qt::AlignLeft | Qt::AlignVCenter);

    auto *summary = new QGroupBox(QStringLiteral("Vision Status"), videoContentHost_);
    summary->setObjectName(QStringLiteral("visionSummaryPanel"));
    visionSummaryPanel_ = summary;
    summary->setMinimumWidth(210);
    summary->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    applySubpanelStyle(summary);
    auto *summaryLayout = new QVBoxLayout(summary);
    summaryLayout->setContentsMargins(8, 10, 8, 8);
    summaryLayout->setSpacing(4);

    auto *videoStateRow = new QHBoxLayout;
    videoStateRow->setContentsMargins(0, 0, 0, 0);
    videoStateRow->setSpacing(6);
    visionDot_ = new QLabel(summary);
    visionDot_->setObjectName(QStringLiteral("visionStatusDot"));
    visionDot_->setFixedSize(10, 10);
    visionDot_->setStyleSheet(
        QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
    visionState_ = new QLabel(
        visionClient_ != nullptr ? QStringLiteral("Disconnected")
                                 : QStringLiteral("Unavailable"),
        summary);
    visionState_->setObjectName(QStringLiteral("visionState"));
    visionState_->setStyleSheet(
        QStringLiteral("font-size: 14px; font-weight: 800; color: #1F4058;"));
    videoStateRow->addWidget(visionDot_);
    videoStateRow->addWidget(visionState_);
    videoStateRow->addStretch(1);
    summaryLayout->addLayout(videoStateRow);

    videoFpsSummary_ = new QLabel(QStringLiteral("Video FPS --"), summary);
    videoFpsSummary_->setObjectName(QStringLiteral("videoFpsSummary"));
    videoFpsSummary_->setToolTip(
        QStringLiteral("Received RBVS frame rate; not inference FPS or unique display FPS."));
    videoFpsSummary_->setStyleSheet(
        QStringLiteral("color: #257A9E; font-size: 13px; font-weight: 700;"));
    summaryLayout->addWidget(videoFpsSummary_);

    videoResolutionSummary_ =
        new QLabel(QStringLiteral("Resolution --"), summary);
    videoResolutionSummary_->setObjectName(
        QStringLiteral("videoResolutionSummary"));
    videoResolutionSummary_->setStyleSheet(
        QStringLiteral("color: #566B79; font-weight: 600;"));
    summaryLayout->addWidget(videoResolutionSummary_);

    storageFreeSummary_ = new QLabel(QStringLiteral("Storage Free --"), summary);
    storageFreeSummary_->setObjectName(QStringLiteral("storageFreeSummary"));
    storageFreeSummary_->setToolTip(QStringLiteral("Pi free disk space reported by the Vision status API; this is storage, not RAM."));
    storageFreeSummary_->setStyleSheet(
        QStringLiteral("color: #566B79; font-weight: 600;"));
    summaryLayout->addWidget(storageFreeSummary_);

    auto *divider1 = new QFrame(summary);
    divider1->setFrameShape(QFrame::HLine);
    divider1->setStyleSheet(QStringLiteral("color: #D5E0E8;"));
    summaryLayout->addWidget(divider1);

    const bool inferenceAvailable = visionControlClient_ != nullptr;
    auto *inferenceStateRow = new QHBoxLayout;
    inferenceStateRow->setContentsMargins(0, 0, 0, 0);
    inferenceStateRow->setSpacing(6);
    inferenceDot_ = new QLabel(summary);
    inferenceDot_->setObjectName(QStringLiteral("inferenceStatusDot"));
    inferenceDot_->setFixedSize(10, 10);
    inferenceDot_->setStyleSheet(
        QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
    inferenceState_ = new QLabel(
        inferenceAvailable
            ? inferenceStateText(visionControlClient_->status().inferenceState)
            : QStringLiteral("Inference Unavailable"),
        summary);
    inferenceState_->setObjectName(QStringLiteral("inferenceState"));
    inferenceState_->setStyleSheet(
        QStringLiteral("font-size: 14px; font-weight: 800; color: #1F4058;"));
    inferenceStateRow->addWidget(inferenceDot_);
    inferenceStateRow->addWidget(inferenceState_);
    inferenceStateRow->addStretch(1);
    summaryLayout->addLayout(inferenceStateRow);

    inferencePerformanceSummary_ = new QLabel(
        QStringLiteral("-- FPS / -- ms"), summary);
    inferencePerformanceSummary_->setObjectName(
        QStringLiteral("inferencePerformanceSummary"));
    inferencePerformanceSummary_->setToolTip(
        QStringLiteral("Camera capture to inference completion; not ORT-only duration."));
    inferencePerformanceSummary_->setStyleSheet(
        QStringLiteral("color: #6C5AAE; font-size: 13px; font-weight: 700;"));
    summaryLayout->addWidget(inferencePerformanceSummary_);

    inferenceDetectionSummary_ = new QLabel(
        QStringLiteral("Detections --"), summary);
    inferenceDetectionSummary_->setObjectName(
        QStringLiteral("inferenceDetectionSummary"));
    summaryLayout->addWidget(inferenceDetectionSummary_);

    inferenceMemorySummary_ = new QLabel(
        QStringLiteral("Memory -- / --"), summary);
    inferenceMemorySummary_->setObjectName(
        QStringLiteral("inferenceMemorySummary"));
    inferenceMemorySummary_->setToolTip(
        QStringLiteral("Vision process RSS / total system physical memory"));
    summaryLayout->addWidget(inferenceMemorySummary_);

    visionControlState_ = new QLabel(
        QStringLiteral("HTTP: Unavailable"), summary);
    visionControlState_->setObjectName(QStringLiteral("visionControlState"));
    visionControlState_->setStyleSheet(
        QStringLiteral("font-weight: 600; color: #566B79;"));
    summaryLayout->addWidget(visionControlState_);

    auto *divider2 = new QFrame(summary);
    divider2->setFrameShape(QFrame::HLine);
    divider2->setStyleSheet(QStringLiteral("color: #D5E0E8;"));
    summaryLayout->addWidget(divider2);

    captureState_ = new QLabel(
        visionControlClient_ != nullptr
            ? QStringLiteral("Capture Disconnected")
            : QStringLiteral("Capture Unavailable"),
        summary);
    captureState_->setObjectName(QStringLiteral("captureState"));
    captureState_->setStyleSheet(
        QStringLiteral("font-weight: 700; color: #566B79;"));
    summaryLayout->addWidget(captureState_);

    captureCountSummary_ = new QLabel(
        QStringLiteral("Recorded -- | Snapshots --"), summary);
    captureCountSummary_->setObjectName(QStringLiteral("captureCountSummary"));
    captureCountSummary_->setWordWrap(true);
    summaryLayout->addWidget(captureCountSummary_);

    visionControlMessage_ = new ui::ElidedLabel(summary);
    visionControlMessage_->setObjectName(QStringLiteral("visionControlMessage"));
    visionControlMessage_->setFullText(QString());
    visionControlMessage_->setAccessibleName(QStringLiteral("Vision notice"));
    visionControlMessage_->setStyleSheet(
        QStringLiteral("font-size: 10px; color: #7B8F9D;"));
    summaryLayout->addWidget(visionControlMessage_);
    summaryLayout->addStretch(1);

    auto *controlsLabel = new QLabel(QStringLiteral("Vision Controls"), summary);
    controlsLabel->setObjectName(QStringLiteral("visionControlsLabel"));
    controlsLabel->setStyleSheet(QStringLiteral(
        "color: #667C8C; font-size: 11px; font-weight: 700;"));
    summaryLayout->addWidget(controlsLabel);

    visionConnectButton_ = new QPushButton(QStringLiteral("Connect Video"), summary);
    visionConnectButton_->setObjectName(QStringLiteral("visionConnectButton"));
    visionConnectButton_->setProperty("consoleActionRole", "primary");

    // The visible inference action is a state-driven Start/Stop toggle.
    // The legacy stop button remains hidden except for Failed -> Clear Error,
    // preserving the frozen recovery semantics without showing duplicate
    // Start/Stop controls during normal operation.
    startInferenceButton_ =
        new QPushButton(QStringLiteral("Start Inference"), summary);
    startInferenceButton_->setObjectName(QStringLiteral("startInferenceButton"));
    startInferenceButton_->setProperty("consoleActionRole", "primary");
    stopInferenceButton_ =
        new QPushButton(QStringLiteral("Clear Error"), summary);
    stopInferenceButton_->setObjectName(QStringLiteral("stopInferenceButton"));
    stopInferenceButton_->setProperty("consoleActionRole", "secondary");
    stopInferenceButton_->setVisible(false);

    snapshotButton_ = new QPushButton(QStringLiteral("Snapshot"), summary);
    snapshotButton_->setObjectName(QStringLiteral("snapshotButton"));
    snapshotButton_->setProperty("consoleActionRole", "secondary");

    // Same pattern for capture: the visible button toggles Start/Stop from
    // authoritative capture status. The legacy stop action stays hidden for
    // compatibility tests and never creates a second operator-facing button.
    startRecordingButton_ =
        new QPushButton(QStringLiteral("Start Recording"), summary);
    startRecordingButton_->setObjectName(QStringLiteral("startRecordingButton"));
    startRecordingButton_->setProperty("consoleActionRole", "primary");
    stopRecordingButton_ =
        new QPushButton(QStringLiteral("Stop Recording"), summary);
    stopRecordingButton_->setObjectName(QStringLiteral("stopRecordingButton"));
    stopRecordingButton_->setProperty("consoleActionRole", "stop");
    stopRecordingButton_->setVisible(false);

    for (QPushButton *button :
         {visionConnectButton_, startInferenceButton_,
          stopInferenceButton_, snapshotButton_,
          startRecordingButton_}) {
        button->setMinimumHeight(34);
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        summaryLayout->addWidget(button);
    }

    content->addWidget(summary, 1);
    layout->addWidget(videoContentHost_, 1);

    // Hidden compatibility action: behavior tests and HTTP-only reconciliation
    // keep the same signal path, but the operator no longer sees a redundant
    // Refresh Status button.
    visionRefreshStatusButton_ = new QPushButton(box);
    visionRefreshStatusButton_->setObjectName(
        QStringLiteral("visionRefreshStatusButton"));
    visionRefreshStatusButton_->setVisible(false);

    const bool available = visionClient_ != nullptr;
    visionConnectButton_->setEnabled(available);
    visionRefreshStatusButton_->setEnabled(visionControlClient_ != nullptr);
    snapshotButton_->setEnabled(false);
    startRecordingButton_->setEnabled(false);
    stopRecordingButton_->setEnabled(false);
    startInferenceButton_->setEnabled(false);
    stopInferenceButton_->setEnabled(false);
    QTimer::singleShot(0, this, [this] { updateVideoSurfaceGeometry(); });
    return box;
}

QWidget *MainWindow::createLeakCard()
{
    auto *box = new QGroupBox(QStringLiteral("Leak Detection"), this);
    box->setObjectName(QStringLiteral("leakCard"));
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(7, 6, 7, 6);
    layout->setSpacing(7);
    addDashboardCardHeader(
        layout, box, QStringLiteral("Leak Detection"));

    auto *statusRow = new QHBoxLayout;
    statusRow->setContentsMargins(1, 1, 0, 0);
    statusRow->setSpacing(7);
    leakDot_ = new QLabel(box);
    leakDot_->setFixedSize(10, 10);
    leakDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
    leakStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    leakStatus_->setStyleSheet(QStringLiteral(
        "color: #566B79;"
        "font-size: 12px;"
        "font-weight: 600;"
    ));
    statusRow->addWidget(leakDot_);
    statusRow->addWidget(leakStatus_);
    statusRow->addStretch();
    layout->addLayout(statusRow);
    layout->addStretch();
    return box;
}

QWidget *MainWindow::createImuCard()
{
    auto *box = new QGroupBox(QStringLiteral("IMU"), this);
    box->setObjectName(QStringLiteral("imuCard"));
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(7, 6, 7, 6);
    layout->setSpacing(6);
    addDashboardCardHeader(
        layout, box, QStringLiteral("IMU"));

    auto *statusRow = new QHBoxLayout;
    statusRow->setContentsMargins(1, 0, 0, 0);
    statusRow->setSpacing(7);
    imuDot_ = new QLabel(box);
    imuDot_->setFixedSize(10, 10);
    imuDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
    imuStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    imuStatus_->setStyleSheet(QStringLiteral(
        "color: #566B79;"
        "font-size: 12px;"
        "font-weight: 600;"
    ));
    statusRow->addWidget(imuDot_);
    statusRow->addWidget(imuStatus_);
    statusRow->addStretch();
    layout->addLayout(statusRow);

    auto *metrics = new QGridLayout;
    metrics->setHorizontalSpacing(10);
    metrics->setVerticalSpacing(2);
    imuAcc_ = new QLabel(QStringLiteral("--"), box);
    imuGyro_ = new QLabel(QStringLiteral("--"), box);
    imuAngle_ = new QLabel(QStringLiteral("--"), box);
    for (QLabel *label : {imuAcc_, imuGyro_, imuAngle_}) {
        label->setStyleSheet(QStringLiteral("color: #405A6B; font-size: 11px; font-weight: 600;"));
    }
    for (int row = 0; row < 3; ++row) {
        auto *label = new QLabel(QStringList{QStringLiteral("Acc"), QStringLiteral("Gyro"), QStringLiteral("Angle")}.at(row), box);
        label->setStyleSheet(QStringLiteral("color: #7B8F9D; font-size: 11px; font-weight: 500;"));
        metrics->addWidget(label, row, 0);
    }
    metrics->addWidget(imuAcc_, 0, 1);
    metrics->addWidget(imuGyro_, 1, 1);
    metrics->addWidget(imuAngle_, 2, 1);
    metrics->setColumnStretch(1, 1);
    layout->addLayout(metrics);
    layout->addStretch();
    return box;
}

QWidget *MainWindow::createDepthCard()
{
    auto *box = new QGroupBox(QStringLiteral("Depth Sensor"), this);
    box->setObjectName(QStringLiteral("depthCard"));
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(7, 6, 7, 6);
    layout->setSpacing(6);
    addDashboardCardHeader(
        layout, box, QStringLiteral("Depth Sensor"));

    auto *statusRow = new QHBoxLayout;
    statusRow->setContentsMargins(1, 0, 0, 0);
    statusRow->setSpacing(7);
    depthDot_ = new QLabel(box);
    depthDot_->setFixedSize(10, 10);
    depthDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
    depthStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    depthStatus_->setStyleSheet(QStringLiteral(
        "color: #566B79;"
        "font-size: 12px;"
        "font-weight: 600;"
    ));
    statusRow->addWidget(depthDot_);
    statusRow->addWidget(depthStatus_);
    statusRow->addStretch();
    layout->addLayout(statusRow);

    auto *metrics = new QGridLayout;
    metrics->setHorizontalSpacing(10);
    metrics->setVerticalSpacing(2);
    depthValue_ = new QLabel(QStringLiteral("--"), box);
    depthTemperature_ = new QLabel(QStringLiteral("--"), box);
    depthAge_ = new QLabel(QStringLiteral("--"), box);
    for (QLabel *label : {depthValue_, depthTemperature_, depthAge_}) {
        label->setStyleSheet(QStringLiteral("color: #405A6B; font-size: 11px; font-weight: 600;"));
    }
    for (int row = 0; row < 3; ++row) {
        auto *label = new QLabel(QStringList{QStringLiteral("Depth"), QStringLiteral("Temp"), QStringLiteral("Age")}.at(row), box);
        label->setStyleSheet(QStringLiteral("color: #7B8F9D; font-size: 11px; font-weight: 500;"));
        metrics->addWidget(label, row, 0);
    }
    metrics->addWidget(depthValue_, 0, 1);
    metrics->addWidget(depthTemperature_, 1, 1);
    metrics->addWidget(depthAge_, 2, 1);
    metrics->setColumnStretch(1, 1);
    layout->addLayout(metrics);
    layout->addStretch();
    return box;
}

QWidget *MainWindow::createProtocolSummaryCard()
{
    auto *box = new QGroupBox(QStringLiteral("Protocol"), this);
    box->setObjectName(QStringLiteral("protocolSummaryCard"));
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(8, 7, 8, 7);
    layout->setSpacing(4);
    addDashboardCardHeader(
        layout, box, QStringLiteral("Protocol"));

    auto *metrics = new QGridLayout;
    metrics->setContentsMargins(0, 0, 0, 0);
    metrics->setHorizontalSpacing(6);
    metrics->setVerticalSpacing(2);

    txCount_ = new QLabel(QStringLiteral("0"), box);
    rxCount_ = new QLabel(QStringLiteral("0"), box);
    crcCount_ = new QLabel(QStringLiteral("0"), box);
    timeoutCount_ = new QLabel(QStringLiteral("0"), box);
    ackRtt_ = new QLabel(QStringLiteral("—"), box);
    for (QLabel *value : {txCount_, crcCount_, ackRtt_, rxCount_, timeoutCount_}) {
        value->setStyleSheet(QStringLiteral(
            "color: #405A6B; font-size: 11px; font-weight: 600;"));
    }

    const QStringList labels = {
        QStringLiteral("TX"), QStringLiteral("CRC"), QStringLiteral("ACK RTT"),
        QStringLiteral("RX"), QStringLiteral("Timeout"),
    };
    const QList<QLabel *> values = {
        txCount_, crcCount_, ackRtt_, rxCount_, timeoutCount_,
    };
    for (int row = 0; row < labels.size(); ++row) {
        auto *label = new QLabel(labels.at(row), box);
        label->setStyleSheet(QStringLiteral(
            "color: #7B8F9D; font-size: 11px; font-weight: 500;"));
        metrics->addWidget(label, row, 0);
        metrics->addWidget(values.at(row), row, 1);
    }
    metrics->setColumnStretch(1, 1);
    layout->addLayout(metrics);
    layout->addStretch(1);
    return box;
}

QWidget *MainWindow::createStatusColumn()
{
    auto *panel = new QWidget(this);
    panel->setObjectName(QStringLiteral("telemetrySidebar"));
    telemetrySidebar_ = panel;
    panel->setMinimumWidth(215);
    panel->setMaximumWidth(230);
    panel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);

    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(5);

    QWidget *leak = createLeakCard();
    QWidget *imu = createImuCard();
    QWidget *depth = createDepthCard();
    QWidget *protocol = createProtocolSummaryCard();
    QWidget *actuator = createActuatorPanel();

    leak->setMinimumHeight(60);
    imu->setMinimumHeight(96);
    depth->setMinimumHeight(92);
    protocol->setMinimumHeight(110);
    actuator->setMinimumHeight(145);

    const std::array<std::pair<QWidget *, int>, 5> cards = {{
        {leak, 1},
        {imu, 2},
        {depth, 2},
        {protocol, 2},
        {actuator, 3},
    }};
    for (const auto &[card, stretch] : cards) {
        card->setMinimumWidth(215);
        card->setMaximumWidth(230);
        card->setMaximumHeight(QWIDGETSIZE_MAX);
        card->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
        layout->addWidget(card, stretch);
    }
    return panel;
}

QWidget *MainWindow::createDashboard()
{
    auto *dashboard = new QWidget(this);
    dashboard->setObjectName(QStringLiteral("dashboard"));
    auto *layout = new QHBoxLayout(dashboard);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    QWidget *video = createVideoPlaceholder();
    video->setMinimumWidth(430);
    video->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(video, 1);

    QWidget *telemetry = createStatusColumn();
    layout->addWidget(telemetry, 0);

    actuatorScroll_ = nullptr;
    return dashboard;
}

QWidget *MainWindow::createServoPanel(int index, ServoId id)
{
    const ServoDescriptor *descriptor = servoDescriptor(id);
    Q_ASSERT(descriptor != nullptr);
    const QString semanticName = descriptor == nullptr
        ? QStringLiteral("Unknown")
        : QString::fromLatin1(descriptor->displayName);

    auto *row = new QWidget(this);
    row->setObjectName(QStringLiteral("servoStatusRow%1").arg(index));
    row->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    row->setFixedHeight(20);

    auto *layout = new QHBoxLayout(row);
    layout->setContentsMargins(2, 1, 2, 1);
    layout->setSpacing(5);

    auto *name = new QLabel(semanticName, row);
    name->setObjectName(QStringLiteral("servoStatusName%1").arg(index));
    name->setStyleSheet(QStringLiteral(
        "color: #263238; font-size: 11px; font-weight: 700;"));
    layout->addWidget(name);
    layout->addStretch(1);

    auto *statusDot = new QLabel(row);
    statusDot->setObjectName(QStringLiteral("servoStatusDot%1").arg(index));
    statusDot->setFixedSize(8, 8);
    statusDot->setStyleSheet(
        QStringLiteral("background: #9e9e9e; border-radius: 4px;"));
    layout->addWidget(statusDot);

    statusLabels_[index] = new QLabel(QStringLiteral("Disconnected"), row);
    statusLabels_[index]->setObjectName(
        QStringLiteral("servoStatusLabel%1").arg(index));
    statusLabels_[index]->setStyleSheet(QStringLiteral(
        "color: #6F7F8B; font-size: 10px; font-weight: 600;"));
    layout->addWidget(statusLabels_[index]);

    refreshServoUi(index);
    return row;
}

QWidget *MainWindow::createActuatorPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Actuator Control"), this);
    box->setObjectName(QStringLiteral("actuatorPage"));
    applyDashboardCardStyle(box);

    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(7, 6, 7, 6);
    layout->setSpacing(2);
    addDashboardCardHeader(
        layout, box, QStringLiteral("Actuator Control"));

    actuatorCardsHost_ = new QWidget(box);
    actuatorCardsHost_->setObjectName(QStringLiteral("actuatorCardsHost"));
    actuatorCardsHost_->setSizePolicy(
        QSizePolicy::Expanding, QSizePolicy::Expanding);
    actuatorGrid_ = new QGridLayout(actuatorCardsHost_);
    actuatorGrid_->setContentsMargins(0, 0, 0, 0);
    actuatorGrid_->setHorizontalSpacing(0);
    actuatorGrid_->setVerticalSpacing(0);

    const auto &descriptors = servoDescriptorTable();
    for (int index = 0; index < kServoCount; ++index) {
        servoPanels_[static_cast<std::size_t>(index)] =
            createServoPanel(index, descriptors.at(index).id);
        actuatorGrid_->addWidget(
            servoPanels_[static_cast<std::size_t>(index)], index, 0);
        actuatorGrid_->setRowStretch(index, 1);
    }
    actuatorColumnCount_ = 1;
    layout->addWidget(actuatorCardsHost_, 1);
    return box;
}

QWidget *MainWindow::createServoFineControlTab()
{
    auto *page = new QWidget(this);
    page->setObjectName(QStringLiteral("servoFineControlPage"));

    auto *grid = new QGridLayout(page);
    grid->setContentsMargins(8, 8, 8, 8);
    grid->setHorizontalSpacing(8);
    grid->setVerticalSpacing(6);

    const QStringList headers = {
        QStringLiteral("Servo"),
        QStringLiteral("PWM"),
        QStringLiteral("PWM Slider"),
        QStringLiteral(""),
        QStringLiteral("Angle"),
        QStringLiteral(""),
        QStringLiteral("Servo"),
        QStringLiteral(""),
    };
    for (int col = 0; col < headers.size(); ++col) {
        auto *label = new QLabel(headers.at(col), page);
        label->setStyleSheet(QStringLiteral(
            "color: #667C8C; font-size: 11px; font-weight: 700;"));
        grid->addWidget(label, 0, col);
    }

    const auto &descriptors = servoDescriptorTable();
    for (int index = 0; index < kServoCount; ++index) {
        const ServoDescriptor &descriptor = descriptors.at(index);
        const ServoId id = descriptor.id;
        const bool supported = controller_->isServoSupported(id);
        const QString semanticName =
            QString::fromLatin1(descriptor.displayName);

        auto *name = new QLabel(semanticName, page);
        name->setStyleSheet(QStringLiteral(
            "color: #263238; font-weight: 700;"));
        grid->addWidget(name, index + 1, 0);

        name->setToolTip(QStringLiteral("Manual raw PWM range: %1-%2 us")
                             .arg(descriptor.commandMinPwmUs)
                             .arg(descriptor.commandMaxPwmUs));

        const int previewMinPwm = std::min({
            static_cast<int>(descriptor.electricalMinPwmUs),
            static_cast<int>(descriptor.neutralPwmUs),
            static_cast<int>(descriptor.electricalMaxPwmUs)});
        const int previewMaxPwm = std::max({
            static_cast<int>(descriptor.electricalMinPwmUs),
            static_cast<int>(descriptor.neutralPwmUs),
            static_cast<int>(descriptor.electricalMaxPwmUs)});

        pwmSpins_[index] = new QSpinBox(page);
        pwmSpins_[index]->setObjectName(
            QStringLiteral("servoPwmSpin%1").arg(index));
        pwmSpins_[index]->setRange(previewMinPwm, previewMaxPwm);
        pwmSpins_[index]->setValue(descriptor.neutralPwmUs);
        pwmSpins_[index]->setSuffix(QStringLiteral(" μs"));
        pwmSpins_[index]->setKeyboardTracking(false);
        pwmSpins_[index]->setEnabled(
            supported && controller_->supportsRawPwm());
        grid->addWidget(pwmSpins_[index], index + 1, 1);

        pwmSliders_[index] = new QSlider(Qt::Horizontal, page);
        pwmSliders_[index]->setObjectName(
            QStringLiteral("servoPwmSlider%1").arg(index));
        pwmSliders_[index]->setRange(previewMinPwm, previewMaxPwm);
        pwmSliders_[index]->setValue(descriptor.neutralPwmUs);
        pwmSliders_[index]->setEnabled(
            supported && controller_->supportsRawPwm());
        grid->addWidget(pwmSliders_[index], index + 1, 2);

        angleSpins_[index] = new QDoubleSpinBox(page);
        angleSpins_[index]->setObjectName(
            QStringLiteral("servoAngleSpin%1").arg(index));
        angleSpins_[index]->setRange(
            static_cast<double>(descriptor.commandMinAngleCdeg) / 100.0,
            static_cast<double>(descriptor.commandMaxAngleCdeg) / 100.0);
        angleSpins_[index]->setDecimals(1);
        angleSpins_[index]->setSingleStep(0.1);
        angleSpins_[index]->setValue(0.0);
        angleSpins_[index]->setSuffix(QStringLiteral(" deg"));
        angleSpins_[index]->setButtonSymbols(
            QAbstractSpinBox::NoButtons);
        angleSpins_[index]->setKeyboardTracking(false);
        angleSpins_[index]->setEnabled(false);
        angleSpins_[index]->setToolTip(
            descriptor.angleSupported
                ? QStringLiteral(
                      "Edit the logical angle preview. PWM follows locally; "
                      "nothing is sent until Apply Angle is clicked.")
                : QStringLiteral(
                      "Angle control is unavailable for this servo."));
        grid->addWidget(angleSpins_[index], index + 1, 4);

        applyButtons_[index] =
            new QPushButton(QStringLiteral("Apply PWM"), page);
        applyButtons_[index]->setObjectName(
            QStringLiteral("servoApplyButton%1").arg(index));
        applyButtons_[index]->setProperty(
            "consoleActionRole", "secondary");
        applyButtons_[index]->setEnabled(false);
        applyButtons_[index]->setToolTip(
            controller_->supportsRawPwm()
                ? QStringLiteral("Apply the selected raw PWM value.")
                : QStringLiteral(
                      "Raw PWM is available only in Direct/APC maintenance mode."));
        grid->addWidget(applyButtons_[index], index + 1, 3);

        angleApplyButtons_[index] =
            new QPushButton(QStringLiteral("Apply Angle"), page);
        angleApplyButtons_[index]->setObjectName(
            QStringLiteral("servoAngleApplyButton%1").arg(index));
        angleApplyButtons_[index]->setProperty(
            "consoleActionRole", "secondary");
        angleApplyButtons_[index]->setEnabled(false);
        angleApplyButtons_[index]->setToolTip(QStringLiteral(
            "Apply the selected logical angle using SetServoAngle."));
        grid->addWidget(angleApplyButtons_[index], index + 1, 5);

        enableButtons_[index] =
            new QPushButton(QStringLiteral("Enable"), page);
        enableButtons_[index]->setObjectName(
            QStringLiteral("servoEnableButton%1").arg(index));
        enableButtons_[index]->setProperty(
            "consoleActionRole", "secondary");
        enableButtons_[index]->setEnabled(false);
        enableButtons_[index]->setToolTip(QStringLiteral(
            "Enable PWM drive and hold the calibrated neutral position."));
        grid->addWidget(enableButtons_[index], index + 1, 6);

        neutralButtons_[index] =
            new QPushButton(QStringLiteral("Neutral"), page);
        neutralButtons_[index]->setObjectName(
            QStringLiteral("servoNeutralButton%1").arg(index));
        neutralButtons_[index]->setProperty(
            "consoleActionRole", "secondary");
        neutralButtons_[index]->setEnabled(false);
        if (descriptor.calibrationPending) {
            neutralButtons_[index]->setToolTip(QStringLiteral(
                "Provisional center candidate; this action reuses the "
                "existing Protocol V2 Neutral command."));
        }
        grid->addWidget(neutralButtons_[index], index + 1, 7);

        connect(pwmSpins_[index],
                qOverload<int>(&QSpinBox::valueChanged),
                this, [this, id, index](int value) {
            const ServoDescriptor *descriptor = servoDescriptor(id);
            if (descriptor == nullptr) {
                return;
            }
            {
                const QSignalBlocker sliderBlock(pwmSliders_[index]);
                pwmSliders_[index]->setValue(value);
            }
            {
                const QSignalBlocker angleBlock(angleSpins_[index]);
                angleSpins_[index]->setValue(
                    static_cast<double>(servoPreviewPwmToAngle(
                        *descriptor, static_cast<quint16>(value))) / 100.0);
            }
            refreshServoUi(index);
        });
        connect(pwmSliders_[index], &QSlider::valueChanged,
                this, [this, id, index](int value) {
            const ServoDescriptor *descriptor = servoDescriptor(id);
            if (descriptor == nullptr) {
                return;
            }
            {
                const QSignalBlocker spinBlock(pwmSpins_[index]);
                pwmSpins_[index]->setValue(value);
            }
            {
                const QSignalBlocker angleBlock(angleSpins_[index]);
                angleSpins_[index]->setValue(
                    static_cast<double>(servoPreviewPwmToAngle(
                        *descriptor, static_cast<quint16>(value))) / 100.0);
            }
            refreshServoUi(index);
        });
        connect(angleSpins_[index],
                qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, [this, id, index](double degrees) {
            const ServoDescriptor *descriptor = servoDescriptor(id);
            if (descriptor == nullptr) {
                return;
            }
            const quint16 pulseUs = servoPreviewAngleToPwm(
                *descriptor, angleDegreesToCentidegrees(degrees));
            {
                const QSignalBlocker spinBlock(pwmSpins_[index]);
                pwmSpins_[index]->setValue(pulseUs);
            }
            {
                const QSignalBlocker sliderBlock(pwmSliders_[index]);
                pwmSliders_[index]->setValue(pulseUs);
            }
            refreshServoUi(index);
        });

        connect(applyButtons_[index], &QPushButton::clicked,
                this, [this, id, index] {
            if (pwmSpins_[index] == nullptr || applyButtons_[index] == nullptr) {
                return;
            }
            const bool submitted = controller_->setServoPwm(
                id, static_cast<quint16>(pwmSpins_[index]->value()));
            applyButtons_[index]->setText(
                submitted ? QStringLiteral("Sent ✓")
                          : QStringLiteral("Rejected"));
            QTimer::singleShot(900, applyButtons_[index],
                               [button = applyButtons_[index]] {
                if (button != nullptr) {
                    button->setText(QStringLiteral("Apply PWM"));
                }
            });
            refreshServoUi(index);
        });
        connect(angleApplyButtons_[index], &QPushButton::clicked,
                this, [this, id, index] {
            if (angleSpins_[index] == nullptr
                || angleApplyButtons_[index] == nullptr) {
                return;
            }
            const bool submitted = controller_->setServoAngle(
                id, angleDegreesToCentidegrees(angleSpins_[index]->value()));
            angleApplyButtons_[index]->setText(
                submitted ? QStringLiteral("Sent ✓")
                          : QStringLiteral("Rejected"));
            QTimer::singleShot(900, angleApplyButtons_[index],
                               [button = angleApplyButtons_[index]] {
                if (button != nullptr) {
                    button->setText(QStringLiteral("Apply Angle"));
                }
            });
            refreshServoUi(index);
        });
        connect(enableButtons_[index], &QPushButton::clicked,
                this, [this, id, index] {
            if (controller_->isServoEnabled(id)) {
                controller_->disableServo(id);
            } else {
                controller_->enableServo(id);
            }
            refreshServoUi(index);
        });
        connect(neutralButtons_[index], &QPushButton::clicked,
                this, [this, id, index] {
            controller_->neutralServo(id);
            refreshServoUi(index);
        });

    }

    grid->setColumnStretch(2, 1);
    grid->setRowStretch(kServoCount + 1, 1);
    return page;
}

QWidget *MainWindow::createMotionPanel()
{
    auto *page = new QWidget(this);
    page->setObjectName(QStringLiteral("motionPage"));
    auto *outer = new QVBoxLayout(page);
    outer->setContentsMargins(6, 5, 6, 5);
    outer->setSpacing(4);

    auto *content = new QHBoxLayout;
    content->setContentsMargins(0, 0, 0, 0);
    content->setSpacing(8);

    const QString commandButtonStyle = QStringLiteral(
        "QPushButton {"
        " background: #E1E7EC;"
        " border: 1px solid #AABAC6;"
        " border-radius: 6px;"
        " color: #294252;"
        " font-size: 13px;"
        " font-weight: 700;"
        " min-height: 26px;"
        " padding: 2px 8px;"
        "}"
        "QPushButton:hover { background: #D5DEE5; border-color: #7F929F; }"
        "QPushButton:pressed, QPushButton:checked { background: #C5D2DC; }"
        "QPushButton:disabled { background: #F0F3F5; color: #8A98A2; border-color: #D2DAE0; }");

    // --- Motion Control ---
    auto *motionGroup = new QGroupBox(QStringLiteral("Motion Control"), page);
    applySubpanelStyle(motionGroup);
    auto *dpad = new QGridLayout(motionGroup);
    dpad->setContentsMargins(6, 6, 6, 6);
    dpad->setSpacing(6);

    const MotionMode dpadModes[] = {
        MotionMode::Forward,
        MotionMode::TurnLeft,
        MotionMode::TurnRight,
        MotionMode::Backward,
    };
    for (const MotionMode mode : dpadModes) {
        auto *button = new QPushButton(motionModeText(mode), motionGroup);
        button->setCheckable(true);
        button->setAutoExclusive(false);
        button->setProperty("consoleActionRole", "secondary");
        button->setStyleSheet(commandButtonStyle);
        motionButtons_[static_cast<std::size_t>(mode)] = button;
        if (mode == MotionMode::Backward) {
            button->setEnabled(false);
            button->setToolTip(QStringLiteral(
                "Brake control is pending water-tank verification; no motion command is emitted."));
        }
        connect(button, &QPushButton::clicked, this, [this, mode] {
            controller_->startMotion(mode);
            refreshMotionUi();
        });
    }

    dpad->addWidget(
        motionButtons_[static_cast<std::size_t>(MotionMode::Forward)], 0, 1);
    dpad->addWidget(
        motionButtons_[static_cast<std::size_t>(MotionMode::TurnLeft)], 1, 0);

    motionStopButton_ = new QPushButton(QStringLiteral("STOP"), motionGroup);
    motionStopButton_->setObjectName(QStringLiteral("motionStopButton"));
    motionStopButton_->setProperty("consoleActionRole", "stop");
    motionStopButton_->setMinimumHeight(30);
    motionStopButton_->setStyleSheet(QStringLiteral(
        "QPushButton { background: #C8D0D6; border: 1px solid #788A96;"
        " border-radius: 6px; color: #253945; font-size: 13px;"
        " font-weight: 800; min-height: 26px; padding: 2px 8px; }"
        "QPushButton:hover { background: #BAC5CC; }"
        "QPushButton:pressed { background: #AAB7C0; }"));
    motionStopButton_->setToolTip(QStringLiteral("Stop the active motion."));
    connect(motionStopButton_, &QPushButton::clicked, this, [this] {
        controller_->stopMotion();
        refreshMotionUi();
    });
    dpad->addWidget(motionStopButton_, 1, 1);
    dpad->addWidget(
        motionButtons_[static_cast<std::size_t>(MotionMode::TurnRight)], 1, 2);
    dpad->addWidget(
        motionButtons_[static_cast<std::size_t>(MotionMode::Backward)], 2, 1);

    // --- Gait / Vertical ---
    auto *gaitGroup = new QGroupBox(QStringLiteral("Gait / Vertical"), page);
    applySubpanelStyle(gaitGroup);
    auto *gaitLayout = new QGridLayout(gaitGroup);
    gaitLayout->setContentsMargins(4, 2, 4, 2);
    gaitLayout->setSpacing(2);

    const MotionMode verticalModes[] = {
        MotionMode::Ascend,
        MotionMode::Descend,
    };
    for (const MotionMode mode : verticalModes) {
        auto *button = new QPushButton(motionModeText(mode), gaitGroup);
        button->setCheckable(true);
        button->setAutoExclusive(false);
        button->setProperty("consoleActionRole", "secondary");
        button->setStyleSheet(commandButtonStyle);
        motionButtons_[static_cast<std::size_t>(mode)] = button;
        connect(button, &QPushButton::clicked, this, [this, mode] {
            controller_->startMotion(mode);
            refreshMotionUi();
        });
    }

    gaitBackendCombo_ = new QComboBox(gaitGroup);
    gaitBackendCombo_->setObjectName(QStringLiteral("gaitBackendCombo"));
    gaitBackendCombo_->addItem(QStringLiteral("Unknown"), -1);
    gaitBackendCombo_->addItem(
        QStringLiteral("SimpleGait"), static_cast<int>(GaitBackend::SimpleGait));
    gaitBackendCombo_->addItem(
        QStringLiteral("CPG"), static_cast<int>(GaitBackend::CPG));
    gaitBackendCombo_->addItem(
        QStringLiteral("Experimental Flex"),
        static_cast<int>(GaitBackend::ExperimentalFlex));
    gaitBackendStatus_ = new QLabel(QStringLiteral("Unknown"), gaitGroup);
    gaitBackendStatus_->setObjectName(QStringLiteral("gaitBackendStatus"));
    gaitBackendStatus_->setStyleSheet(
        QStringLiteral("font-weight: 700; color: #405A6B;"));
    frontRearCoordinationCombo_ = new QComboBox(gaitGroup);
    frontRearCoordinationCombo_->setObjectName(
        QStringLiteral("frontRearCoordinationCombo"));
    frontRearCoordinationCombo_->addItem(QStringLiteral("Unknown"), -1);
    frontRearCoordinationCombo_->addItem(
        QStringLiteral("Same Direction"),
        static_cast<int>(FrontRearCoordination::SameDirection));
    frontRearCoordinationCombo_->addItem(
        QStringLiteral("Opposite Direction"),
        static_cast<int>(FrontRearCoordination::OppositeDirection));
    frontRearCoordinationStatus_ = new QLabel(QStringLiteral("Unknown"), gaitGroup);
    frontRearCoordinationStatus_->setObjectName(
        QStringLiteral("frontRearCoordinationStatus"));
    frontRearCoordinationStatus_->setStyleSheet(
        QStringLiteral("font-weight: 700; color: #405A6B;"));

    gaitLayout->addWidget(new QLabel(QStringLiteral("Gait"), gaitGroup), 0, 0);
    gaitLayout->addWidget(gaitBackendCombo_, 0, 1);
    auto *gaitCurrentLabel = new QLabel(QStringLiteral("Current"), gaitGroup);
    gaitCurrentLabel->setObjectName(QStringLiteral("gaitBackendCurrentLabel"));
    gaitLayout->addWidget(gaitCurrentLabel, 1, 0);
    gaitLayout->addWidget(gaitBackendStatus_, 1, 1);
    gaitLayout->addWidget(
        new QLabel(QStringLiteral("Front / Rear"), gaitGroup), 2, 0);
    gaitLayout->addWidget(frontRearCoordinationCombo_, 2, 1);
    auto *coordinationCurrentLabel =
        new QLabel(QStringLiteral("Current"), gaitGroup);
    coordinationCurrentLabel->setObjectName(
        QStringLiteral("frontRearCoordinationCurrentLabel"));
    gaitLayout->addWidget(coordinationCurrentLabel, 3, 0);
    gaitLayout->addWidget(frontRearCoordinationStatus_, 3, 1);
    gaitLayout->addWidget(
        motionButtons_[static_cast<std::size_t>(MotionMode::Ascend)], 4, 0);
    gaitLayout->addWidget(
        motionButtons_[static_cast<std::size_t>(MotionMode::Descend)], 4, 1);
    gaitLayout->setColumnStretch(0, 1);
    gaitLayout->setColumnStretch(1, 1);

    // --- Runtime Log ---
    auto *logGroup = new QGroupBox(QStringLiteral("Log"), page);
    applySubpanelStyle(logGroup);
    auto *logLayout = new QVBoxLayout(logGroup);
    logLayout->setContentsMargins(6, 8, 6, 6);
    logLayout->setSpacing(4);
    log_ = new QPlainTextEdit(logGroup);
    log_->setObjectName(QStringLiteral("motionLog"));
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(1000);
    log_->setPlaceholderText(
        QStringLiteral("Runtime events and controller messages will appear here."));
    log_->setFrameShape(QFrame::NoFrame);
    log_->setStyleSheet(QStringLiteral(
        "QPlainTextEdit { background: transparent; border: none;"
        " color: #405A6B; padding: 5px; font-size: 11px; }"));
    logLayout->addWidget(log_, 1);

    content->addWidget(motionGroup, 1);
    content->addWidget(gaitGroup, 1);
    content->addWidget(logGroup, 1);
    outer->addLayout(content, 1);

    connect(gaitBackendCombo_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int index) {
                if (index < 0 || gaitBackendCombo_ == nullptr
                    || !gaitBackendCombo_->isEnabled()) {
                    refreshGaitSelectorsUi();
                    return;
                }
                const QVariant value = gaitBackendCombo_->itemData(index);
                if (!value.isValid()
                    || !isValidGaitBackend(static_cast<quint8>(value.toInt()))) {
                    refreshGaitSelectorsUi();
                    return;
                }
                controller_->setGaitBackend(
                    static_cast<GaitBackend>(value.toInt()));
                refreshGaitSelectorsUi();
            });

    connect(frontRearCoordinationCombo_,
            qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int index) {
                if (index < 0 || frontRearCoordinationCombo_ == nullptr
                    || !frontRearCoordinationCombo_->isEnabled()) {
                    refreshGaitSelectorsUi();
                    return;
                }
                const QVariant value = frontRearCoordinationCombo_->itemData(index);
                if (!value.isValid()
                    || !isValidFrontRearCoordination(
                        static_cast<quint8>(value.toInt()))) {
                    refreshGaitSelectorsUi();
                    return;
                }
                controller_->setFrontRearCoordination(
                    static_cast<FrontRearCoordination>(value.toInt()));
                refreshGaitSelectorsUi();
            });

    refreshMotionUi();
    refreshGaitSelectorsUi();
    return page;
}

QWidget *MainWindow::createDataPlotsTab()
{
    auto *section = new QGroupBox(QStringLiteral("Data Plots"), this);
    section->setObjectName(QStringLiteral("dataPlotsPage"));
    applySectionStyle(section);

    auto *outer = new QVBoxLayout(section);
    outer->setContentsMargins(10, 8, 10, 10);
    outer->setSpacing(7);
    addSectionHeader(
        outer, section, QStringLiteral("Data Plots"));

    auto *tabs = new QTabWidget(section);
    tabs->setObjectName(QStringLiteral("dataPlotTabs"));
    dataPlotTabs_ = tabs;
    const QString titles[] = {
        QStringLiteral("IMU"),
        QStringLiteral("Depth"),
        QStringLiteral("Actuator"),
    };
    for (const QString &title : titles) {
        auto *page = new QWidget(tabs);
        auto *pageLayout = new QVBoxLayout(page);
        pageLayout->setContentsMargins(10, 10, 10, 10);
        pageLayout->setSpacing(8);

        auto *viewport = new QWidget(page);
        viewport->setObjectName(QStringLiteral("plotViewport"));
        viewport->setStyleSheet(QStringLiteral(
            "#plotViewport {"
            "  background: #F8FAFC;"
            "  border: 1px solid #E1E8EE;"
            "  border-radius: 6px;"
            "}"
        ));

        auto *viewportLayout = new QVBoxLayout(viewport);
        viewportLayout->setContentsMargins(12, 12, 12, 12);
        auto *label = new QLabel(QStringLiteral("Plot placeholder — no data buffer"), viewport);
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet(QStringLiteral(
            "QLabel {"
            "  background: transparent;"
            "  border: none;"
            "  color: #8497A5;"
            "  font-size: 11px;"
            "}"
        ));
        viewportLayout->addStretch();
        viewportLayout->addWidget(label);
        viewportLayout->addStretch();

        pageLayout->addWidget(viewport, 1);
        tabs->addTab(page, title);
    }
    outer->addWidget(tabs, 1);
    return section;
}

QWidget *MainWindow::createLogTab()
{
    auto *tab = new QWidget(this);
    tab->setObjectName(QStringLiteral("logPage"));
    auto *layout = new QVBoxLayout(tab);
    layout->setContentsMargins(8, 8, 8, 8);
    log_ = new QPlainTextEdit(tab);
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(1000);
    log_->setPlaceholderText(
        QStringLiteral("Runtime events and controller messages will appear here."));
    log_->setStyleSheet(QStringLiteral(
        "QPlainTextEdit {"
        "  background: #F8FAFC;"
        "  border: 1px solid #E1E8EE;"
        "  border-radius: 6px;"
        "  color: #405A6B;"
        "  padding: 6px;"
        "}"
    ));
    layout->addWidget(log_);
    return tab;
}

QWidget *MainWindow::createTelemetryDetailsTab()
{
    auto *tab = new QWidget(this);
    tab->setObjectName(QStringLiteral("telemetryDetailsPage"));
    auto *layout = new QVBoxLayout(tab);
    layout->setContentsMargins(8, 8, 8, 8);
    auto *form = new QFormLayout;
    imuDiagnostics_ = new QLabel(QStringLiteral("--"), tab);
    imuDiagnostics_->setWordWrap(true);
    depthDiagnostics_ = new QLabel(QStringLiteral("--"), tab);
    depthDiagnostics_->setWordWrap(true);
    form->addRow(QStringLiteral("IMU Diagnostics"), imuDiagnostics_);
    form->addRow(QStringLiteral("Depth Diagnostics"), depthDiagnostics_);
    layout->addLayout(form);
    layout->addStretch();
    return tab;
}

QWidget *MainWindow::createProtocolDetailsTab()
{
    auto *tab = new QWidget(this);
    tab->setObjectName(QStringLiteral("protocolDetailsPage"));
    auto *layout = new QVBoxLayout(tab);
    layout->setContentsMargins(8, 8, 8, 8);
    auto *form = new QFormLayout;
    txHex_ = new QLineEdit(tab);
    rxHex_ = new QLineEdit(tab);
    txHex_->setReadOnly(true);
    rxHex_->setReadOnly(true);
    form->addRow(QStringLiteral("TX Hex"), txHex_);
    form->addRow(QStringLiteral("RX Hex"), rxHex_);
    layout->addLayout(form);
    ackStatus_ = new QLabel(QStringLiteral("Idle"), tab);
    layout->addWidget(new QLabel(QStringLiteral("ACK status"), tab));
    layout->addWidget(ackStatus_);
    protocolCountersDetails_ = new QLabel(
        QStringLiteral("TX: 0\nRX: 0\nCRC: 0\nTimeout: 0\nACK RTT: unavailable"),
        tab);
    protocolCountersDetails_->setObjectName(QStringLiteral("protocolCountersDetails"));
    protocolCountersDetails_->setWordWrap(true);
    protocolCountersDetails_->setTextFormat(Qt::PlainText);
    protocolCountersDetails_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(protocolCountersDetails_);
    layout->addStretch();
    return tab;
}

QTabWidget *MainWindow::createLogDetailsTabs()
{
    auto *tabs = new QTabWidget(this);
    tabs->setObjectName(QStringLiteral("diagnosticsTabs"));
    tabs->addTab(createLogTab(), QStringLiteral("Log"));
    tabs->addTab(createTelemetryDetailsTab(), QStringLiteral("Telemetry Details"));
    tabs->addTab(createProtocolDetailsTab(), QStringLiteral("Protocol Details"));
    return tabs;
}

QWidget *MainWindow::createLowerDashboard()
{
    auto *lower = new QWidget(this);
    auto *layout = new QHBoxLayout(lower);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    auto *diagnostics = new QGroupBox(QStringLiteral("Diagnostics / Log"), lower);
    applySectionStyle(diagnostics);
    auto *diagLayout = new QVBoxLayout(diagnostics);
    diagLayout->setContentsMargins(10, 8, 10, 10);
    diagLayout->setSpacing(7);
    addSectionHeader(
        diagLayout,
        diagnostics,
        QStringLiteral("Diagnostics / Log"));

    QTabWidget *detailsTabs = createLogDetailsTabs();
    diagLayout->addWidget(detailsTabs, 1);

    layout->addWidget(createMotionPanel(), 1);
    layout->addWidget(createDataPlotsTab(), 1);
    layout->addWidget(diagnostics, 1);
    return lower;
}

QWidget *MainWindow::createVisionDetailsTab()
{
    auto *tab = new QWidget(this);
    tab->setObjectName(QStringLiteral("visionDetailsPage"));
    auto *scroll = new QScrollArea(tab);
    scroll->setObjectName(QStringLiteral("visionDetailsScroll"));
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    auto *content = new QWidget(scroll);
    auto *layout = new QVBoxLayout(content);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(7);

    auto *title = new QLabel(QStringLiteral("Vision Details"), content);
    title->setStyleSheet(QStringLiteral("color: #1F4058; font-size: 14px; font-weight: 700;"));
    layout->addWidget(title);

    auto *endpointForm = new QFormLayout;
    visionEndpointDetails_ = new QLabel(QStringLiteral("--"), content);
    visionEndpointDetails_->setObjectName(QStringLiteral("visionEndpointDetails"));
    visionEndpointDetails_->setWordWrap(true);
    visionEndpointDetails_->setTextFormat(Qt::PlainText);
    visionEndpointDetails_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    endpointForm->addRow(QStringLiteral("Endpoints"), visionEndpointDetails_);
    layout->addLayout(endpointForm);

    auto *videoForm = new QFormLayout;
    visionDiagnostics_ = new QLabel(QStringLiteral("Fresh\nRX FPS: -- | Display paint-event FPS: -- | Latest frame ID: --\nWire frames: -- | RX replaced: -- | JPEG decode drops: -- | UI replaced: --"), content);
    visionDiagnostics_->setObjectName(QStringLiteral("visionDiagnostics"));
    visionDiagnostics_->setWordWrap(true);
    visionDiagnostics_->setTextFormat(Qt::PlainText);
    visionDiagnostics_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    videoForm->addRow(QStringLiteral("Video diagnostics"), visionDiagnostics_);
    visualProposalDetails_ = new QLabel(QStringLiteral("VISION DRY_RUN - no motion output"), content);
    visualProposalDetails_->setObjectName(QStringLiteral("visualProposalDetails"));
    visualProposalDetails_->setWordWrap(true);
    visualProposalDetails_->setTextFormat(Qt::PlainText);
    visualProposalDetails_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    videoForm->addRow(QStringLiteral("Visual proposal"), visualProposalDetails_);
    layout->addLayout(videoForm);

    auto *csvForm = new QFormLayout;
    visualCsvEnabled_ = new QCheckBox(QStringLiteral("Record visual CSV (no motion output)"), content);
    visualCsvEnabled_->setObjectName(QStringLiteral("visualCsvEnabled"));
    csvForm->addRow(QStringLiteral("CSV"), visualCsvEnabled_);
    auto *csvDirectoryRow = new QWidget(content);
    auto *csvDirectoryLayout = new QHBoxLayout(csvDirectoryRow);
    csvDirectoryLayout->setContentsMargins(0, 0, 0, 0);
    visualCsvDirectory_ = new QLineEdit(vision::VisualCsvLogger::defaultDirectory(), csvDirectoryRow);
    visualCsvDirectory_->setObjectName(QStringLiteral("visualCsvDirectory"));
    visualCsvBrowse_ = new QPushButton(QStringLiteral("Browse..."), csvDirectoryRow);
    visualCsvBrowse_->setObjectName(QStringLiteral("visualCsvBrowse"));
    csvDirectoryLayout->addWidget(visualCsvDirectory_, 1);
    csvDirectoryLayout->addWidget(visualCsvBrowse_);
    csvForm->addRow(QStringLiteral("CSV directory"), csvDirectoryRow);
    visualCsvStatus_ = new QLabel(QStringLiteral("CSV OFF"), content);
    visualCsvStatus_->setObjectName(QStringLiteral("visualCsvStatus"));
    visualCsvStatus_->setWordWrap(true);
    visualCsvStatus_->setTextFormat(Qt::PlainText);
    visualCsvStatus_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    csvForm->addRow(QStringLiteral("CSV status"), visualCsvStatus_);
    layout->addLayout(csvForm);

    auto *inferenceForm = new QFormLayout;
    inferenceArtifactValue_ = new QLineEdit(content);
    inferenceArtifactValue_->setObjectName(QStringLiteral("inferenceArtifactValue"));
    inferenceArtifactValue_->setReadOnly(true);
    inferenceShaValue_ = new QLineEdit(content);
    inferenceShaValue_->setObjectName(QStringLiteral("inferenceShaValue"));
    inferenceShaValue_->setReadOnly(true);
    inferenceShaValue_->setMinimumWidth(360);
    inferenceThresholdValue_ = new QLabel(QStringLiteral("--"), content);
    inferenceOperationValue_ = new QLabel(QStringLiteral("--"), content);
    inferenceLastErrorValue_ = new QLabel(QStringLiteral("--"), content);
    inferenceLastErrorValue_->setObjectName(QStringLiteral("inferenceLastErrorValue"));
    inferenceLastErrorValue_->setWordWrap(true);
    inferenceLastErrorValue_->setTextFormat(Qt::PlainText);
    inferenceLastErrorValue_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    inferenceForm->addRow(QStringLiteral("Artifact"), inferenceArtifactValue_);
    inferenceForm->addRow(QStringLiteral("SHA-256"), inferenceShaValue_);
    inferenceForm->addRow(QStringLiteral("Threshold"), inferenceThresholdValue_);
    inferenceForm->addRow(QStringLiteral("Operation"), inferenceOperationValue_);
    inferenceForm->addRow(QStringLiteral("Inference last error"), inferenceLastErrorValue_);
    layout->addWidget(new QLabel(QStringLiteral("Inference"), content));
    layout->addLayout(inferenceForm);
    inferenceDiagnostics_ = new QLabel(QStringLiteral("Artifact: -- | SHA-256: --\nFPS: -- | Latency ms: --\nLatest Frame ID: -- | Detections: -- | Skipped: --"), content);
    inferenceDiagnostics_->setObjectName(QStringLiteral("inferenceDiagnostics"));
    inferenceDiagnostics_->setWordWrap(true);
    inferenceDiagnostics_->setTextFormat(Qt::PlainText);
    inferenceDiagnostics_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(inferenceDiagnostics_);

    auto *captureForm = new QFormLayout;
    captureDiagnostics_ = new QLabel(QStringLiteral("Session -- | Recorded 0 | Snapshots 0 | Queue 0 MiB | Free -- GiB"), content);
    captureDiagnostics_->setObjectName(QStringLiteral("captureDiagnostics"));
    captureDiagnostics_->setWordWrap(true);
    captureDiagnostics_->setTextFormat(Qt::PlainText);
    captureDiagnostics_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    captureForm->addRow(QStringLiteral("Capture"), captureDiagnostics_);
    layout->addLayout(captureForm);

    controlResponseDetails_ = new QLabel(QStringLiteral("--"), content);
    controlResponseDetails_->setObjectName(QStringLiteral("controlResponseDetails"));
    controlResponseDetails_->setWordWrap(true);
    controlResponseDetails_->setTextFormat(Qt::PlainText);
    controlResponseDetails_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    auto *controlForm = new QFormLayout;
    controlForm->addRow(QStringLiteral("Control response / error"), controlResponseDetails_);
    layout->addLayout(controlForm);
    layout->addStretch(1);
    scroll->setWidget(content);
    auto *outer = new QVBoxLayout(tab);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(scroll);
    return tab;
}

QWidget *MainWindow::createOperatorActionBar()
{
    auto *bar = new QWidget(this);
    bar->setObjectName(QStringLiteral("operatorActionBar"));
    auto *layout = new QHBoxLayout(bar);
    layout->setContentsMargins(10, 5, 10, 5);
    layout->setSpacing(8);
    auto *title = new QLabel(QStringLiteral("Operator tools"), bar);
    title->setObjectName(QStringLiteral("operatorToolsTitle"));
    title->setStyleSheet(QStringLiteral(
        "color: #1F4058; font-size: 15px; font-weight: 700;"));
    layout->addWidget(title);
    layout->addSpacing(12);
    motionStatus_ = new QLabel(QStringLiteral("Stopped"), bar);
    motionStatus_->setObjectName(QStringLiteral("motionStatus"));
    motionStatus_->setMinimumWidth(110);
    motionStatus_->setStyleSheet(
        QStringLiteral("color: #667C8C; font-weight: 700;"));
    layout->addWidget(motionStatus_);
    layout->addStretch(1);

    enableAllButton_ = new QPushButton(QStringLiteral("Enable All"), bar);
    enableAllButton_->setObjectName(QStringLiteral("enableAllButton"));
    enableAllButton_->setProperty("consoleActionRole", "secondary");
    layout->addWidget(enableAllButton_);
    connect(enableAllButton_, &QPushButton::clicked, this, [this] {
        if (!controller_->isControlActive() || controller_->isMotionActive()) {
            return;
        }
        const auto &descriptors = servoDescriptorTable();
        for (const auto &descriptor : descriptors) {
            if (controller_->isServoSupported(descriptor.id)
                && !controller_->isServoEnabled(descriptor.id)
                && !controller_->isServoDisablePending(descriptor.id)) {
                controller_->enableServo(descriptor.id);
            }
        }
        for (int index = 0; index < kServoCount; ++index) {
            refreshServoUi(index);
        }
        refreshMotionUi();
    });

    disableAllButton_ = new QPushButton(QStringLiteral("Disable All"), bar);
    disableAllButton_->setObjectName(QStringLiteral("disableAllButton"));

    disableAllButton_->setProperty("consoleActionRole", "stop");
    layout->addWidget(disableAllButton_);
    connect(disableAllButton_, &QPushButton::clicked, this, [this] {
        controller_->disableAll();
        for (int index = 0; index < kServoCount; ++index) {
            refreshServoUi(index);
        }
        refreshMotionUi();
    });

    emergencyStopButton_ = new QPushButton(QStringLiteral("Emergency Stop"), bar);
    emergencyStopButton_->setObjectName(QStringLiteral("emergencyStopButton"));
    emergencyStopButton_->setEnabled(false);
    emergencyStopButton_->setProperty("consoleActionRole", "danger");
    emergencyStopButton_->setToolTip(
        QStringLiteral("Protocol V2 has no Emergency Stop message in Phase 1"));
    layout->addWidget(emergencyStopButton_);
    return bar;
}

QTabWidget *MainWindow::createOperatorToolsTabs()
{
    operatorToolsTabs_ = new QTabWidget(this);
    operatorToolsTabs_->setObjectName(QStringLiteral("operatorToolsTabs"));
    operatorToolsTabs_->setMinimumHeight(0);
    operatorToolsTabs_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);

    motionScrollArea_ = new QScrollArea(this);
    motionScrollArea_->setObjectName(QStringLiteral("motionScrollArea"));
    motionScrollArea_->setWidgetResizable(true);
    motionScrollArea_->setFrameShape(QFrame::NoFrame);
    motionScrollArea_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    motionScrollArea_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    motionScrollArea_->setWidget(createMotionPanel());
    operatorToolsTabs_->addTab(
        motionScrollArea_, QStringLiteral("Motion / Gait"));

    auto *servoFineScroll = new QScrollArea(this);
    servoFineScroll->setObjectName(QStringLiteral("servoFineControlScrollArea"));
    servoFineScroll->setWidgetResizable(true);
    servoFineScroll->setFrameShape(QFrame::NoFrame);
    servoFineScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    servoFineScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    servoFineScroll->setWidget(createServoFineControlTab());
    operatorToolsTabs_->addTab(
        servoFineScroll, QStringLiteral("Servo Fine Control"));

    operatorToolsTabs_->addTab(
        createVisionDetailsTab(), QStringLiteral("Vision Details"));

    auto wrapDetailsPage = [this](QWidget *page, const QString &scrollName) {
        auto *scroll = new QScrollArea(this);
        scroll->setObjectName(scrollName);
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scroll->setWidget(page);
        return scroll;
    };

    operatorToolsTabs_->addTab(
        wrapDetailsPage(
            createTelemetryDetailsTab(),
            QStringLiteral("telemetryDetailsScroll")),
        QStringLiteral("Telemetry Details"));
    operatorToolsTabs_->addTab(
        wrapDetailsPage(
            createProtocolDetailsTab(),
            QStringLiteral("protocolDetailsScroll")),
        QStringLiteral("Protocol Details"));
    operatorToolsTabs_->addTab(
        wrapDetailsPage(
            createDataPlotsTab(),
            QStringLiteral("dataPlotsScroll")),
        QStringLiteral("Data Plots"));

    return operatorToolsTabs_;
}

QWidget *MainWindow::createOperatorTools()
{
    auto *pane = new QWidget(this);
    pane->setObjectName(QStringLiteral("operatorToolsPane"));
    pane->setMinimumHeight(0);
    pane->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    auto *layout = new QVBoxLayout(pane);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);
    layout->addWidget(createOperatorActionBar());
    layout->addWidget(createOperatorToolsTabs(), 1);
    return pane;
}

MainWindow::~MainWindow()
{
    if (visualCsvLogger_) { visualCsvLogger_->stop(); }
}

void MainWindow::refreshVisualCsvUi()
{
    const bool recording = visualCsvLogger_->isRecording();
    const QSignalBlocker blocker(visualCsvEnabled_);
    visualCsvEnabled_->setChecked(recording);
    visualCsvDirectory_->setEnabled(!recording);
    visualCsvBrowse_->setEnabled(!recording);
    if (!visualCsvLogger_->lastError().isEmpty()) {
        visualCsvStatus_->setText(QStringLiteral("CSV Error - stopped: %1\n%2")
            .arg(visualCsvLogger_->lastError(), visualCsvLogger_->filePath()));
    } else if (recording) {
        visualCsvStatus_->setText(QStringLiteral("CSV Recording (32 MiB limit):\n%1")
            .arg(visualCsvLogger_->filePath()));
    } else {
        visualCsvStatus_->setText(QStringLiteral("CSV OFF%1")
            .arg(visualCsvLogger_->filePath().isEmpty() ? QString{}
                 : QStringLiteral(" - saved:\n") + visualCsvLogger_->filePath()));
    }
}

void MainWindow::bindVisionUi()
{
    visualCsvLogger_ = new vision::VisualCsvLogger({}, this);
    connect(visualCsvLogger_, &vision::VisualCsvLogger::recordingChanged,
            this, &MainWindow::refreshVisualCsvUi);
    connect(visualCsvEnabled_, &QCheckBox::toggled, this, [this](bool enabled) {
        if (enabled) {
            if (visualCsvLogger_->start(visualCsvDirectory_->text())) {
                visualCsvLogger_->record(visualSession_->snapshot());
            }
        } else {
            visualCsvLogger_->stop();
        }
        refreshVisualCsvUi();
    });
    connect(visualCsvBrowse_, &QPushButton::clicked, this, [this] {
        const auto directory = QFileDialog::getExistingDirectory(
            this, QStringLiteral("Visual CSV directory"), visualCsvDirectory_->text());
        if (!directory.isEmpty()) { visualCsvDirectory_->setText(directory); }
    });
    visualSession_ = new vision::VisualDiagnosticSession({}, {}, this);
    connect(visualSession_, &vision::VisualDiagnosticSession::diagnosticChanged, this,
            [this](const vision::VisualDiagnosticSnapshot &snapshot) {
        visualCsvLogger_->record(snapshot);
        if (videoView_ == nullptr) { return; }
        if (snapshot.target && latestDetectionFrame_) {
            videoView_->setDetectionOverlay(*latestDetectionFrame_);
        } else {
            const auto reason = snapshot.state == vision::VisualState::InferenceOff
                ? vision::DetectionDisplayState::InferenceOff
                : snapshot.state == vision::VisualState::NoTarget
                    ? vision::DetectionDisplayState::NoTarget : vision::DetectionDisplayState::Stale;
            videoView_->clearDetectionOverlay(reason);
        }
        videoView_->setVisualDiagnostic(snapshot);
        if (visualProposalDetails_) {
            visualProposalDetails_->setText(videoView_->visualDiagnosticText());
        }
    });
    if (visionClient_ != nullptr) {
        connect(visionConnectButton_, &QPushButton::clicked, this, [this] {
            if (visionClient_->state() == vision::VisionConnectionState::Connected
                || visionClient_->state() == vision::VisionConnectionState::Connecting) {
                visionClient_->disconnectFromHost();
                return;
            }
            if (visionControlClient_ != nullptr) {
                if (!visionControlClient_->setEndpoint(
                        committedPiHost_, visionControlClient_->port())) {
                    visionNoticeText_ = QStringLiteral(
                        "Error: Vision action busy; wait before changing endpoint");
                    visionNoticeTooltip_ = visionNoticeText_;
                    appendLog(QStringLiteral(
                        "Vision capture: wait for the active action before changing host"));
                    refreshVisionUi();
                    return;
                }
                captureState_->setText(QStringLiteral("Capture Connecting"));
                visionControlClient_->startPolling();
            }
            visionClient_->connectToHost(
                committedPiHost_, static_cast<quint16>(visionPort_->value()));
        });

        connect(visionClient_, &vision::VisionClient::connectionStateChanged,
                this, [this](vision::VisionConnectionState state) {
            QString text = QStringLiteral("Disconnected");
            QString buttonText = QStringLiteral("Connect Video");
            switch (state) {
            case vision::VisionConnectionState::Disconnected:
                break;
            case vision::VisionConnectionState::Connecting:
                text = QStringLiteral("Connecting");
                buttonText = QStringLiteral("Disconnect Video");
                break;
            case vision::VisionConnectionState::Connected:
                text = QStringLiteral("Connected");
                buttonText = QStringLiteral("Disconnect Video");
                break;
            case vision::VisionConnectionState::Error:
                text = QStringLiteral("Error");
                break;
            }
            visionState_->setText(text);
            if (visionDot_ != nullptr) {
                QString color = QStringLiteral("#9e9e9e");
                if (state == vision::VisionConnectionState::Connecting) {
                    color = QStringLiteral("#D39B2A");
                } else if (state == vision::VisionConnectionState::Connected) {
                    color = QStringLiteral("#35A853");
                } else if (state == vision::VisionConnectionState::Error) {
                    color = QStringLiteral("#C53F3F");
                }
                visionDot_->setStyleSheet(
                    QStringLiteral("background: %1; border-radius: 5px;").arg(color));
            }
            visionConnectButton_->setText(buttonText);
            const bool videoConnected =
                state == vision::VisionConnectionState::Connected;
            if (!videoConnected) {
                haveVisionReceiveSample_ = false;
                visionFrameSize_ = QSize();
                videoView_->clearFrame();
                resetDetectionSession();
            } else {
                syncDetectionStream();
            }
            refreshDetectionOverlay();
            refreshVideoDiagnosticsUi();
            refreshPiHostUi();
        });

        connect(visionClient_, &vision::VisionClient::frameReady,
                this, [this](const QImage &image,
                             quint64 frameId,
                             quint64 captureTimestampNs) {
            haveVisionReceiveSample_ = true;
            if (!image.isNull()) {
                visionFrameSize_ = image.size();
            }
            videoView_->setFrame(image, frameId, captureTimestampNs);
            refreshDetectionOverlay();
            updateVideoSurfaceGeometry();
            refreshVideoDiagnosticsUi();
        });
        connect(visionClient_, &vision::VisionClient::diagnosticsChanged,
                this, [this](double, quint64, quint64, quint64,
                             quint64 jpegDecodeErrors) {
            visionJpegDecodeErrors_ = jpegDecodeErrors;
            refreshVideoDiagnosticsUi();
        });
        connect(visionClient_, &vision::VisionClient::logMessage,
                this, &MainWindow::appendLog);
        connect(visionClient_, &vision::VisionClient::protocolError,
                this, [this](const QString &message) {
            appendLog(QStringLiteral("Vision: %1").arg(message));
        });
        connect(visionClient_, &vision::VisionClient::endpointActivityChanged,
                this, &MainWindow::refreshPiHostUi);
    }

    if (visionControlClient_ != nullptr) {
        connect(snapshotButton_, &QPushButton::clicked,
                visionControlClient_, &vision::VisionControlClient::requestSnapshot);

        connect(startRecordingButton_, &QPushButton::clicked, this, [this] {
            if (visionControlClient_ == nullptr
                || !visionControlClient_->hasFreshStatus()
                || visionControlClient_->actionBusy()) {
                return;
            }
            const auto status = visionControlClient_->status();
            if (!status.haveCaptureStatus) {
                return;
            }
            if (status.recording) {
                visionControlClient_->stopRecording();
            } else if (status.state != QStringLiteral("stopping")
                       && !hostDirty()) {
                visionControlClient_->startRecording();
            }
        });
        connect(stopRecordingButton_, &QPushButton::clicked,
                visionControlClient_, &vision::VisionControlClient::stopRecording);

        connect(startInferenceButton_, &QPushButton::clicked, this, [this] {
            if (visionControlClient_ == nullptr
                || !visionControlClient_->hasFreshStatus()
                || visionControlClient_->actionBusy()
                || visionControlClient_->inferenceReconcilePending()) {
                return;
            }
            const auto status = visionControlClient_->status();
            const auto ui = vision::makeInferenceUiState(
                status, true, false, false);
            const bool stopIntent =
                status.haveInferenceState
                && (status.inferenceState == QStringLiteral("running")
                    || status.inferenceState == QStringLiteral("starting"));
            if (stopIntent && ui.stopEnabled) {
                visionControlClient_->stopInference();
            } else if (ui.startEnabled && !hostDirty()) {
                visionControlClient_->startInference();
            }
        });
        connect(stopInferenceButton_, &QPushButton::clicked,
                visionControlClient_, &vision::VisionControlClient::stopInference);
        connect(visionRefreshStatusButton_, &QPushButton::clicked, this, [this] {
            if (visionControlClient_ == nullptr || !hostCandidateValid(committedPiHost_)) {
                return;
            }
            visionControlClient_->setEndpoint(
                committedPiHost_, visionControlClient_->port());
            visionControlClient_->startPolling();
        });
        connect(visionControlClient_, &vision::VisionControlClient::statusChanged,
                this, [this](vision::VisionCaptureStatus) {
            refreshVisionUi();
            syncDetectionStream();
        });
        connect(visionControlClient_, &vision::VisionControlClient::actionSucceeded,
                this, [this](const QString &action) {
            const QString text = QStringLiteral("ACK: %1 succeeded").arg(action);
            visionNoticeText_ = text;
            visionNoticeTooltip_ = visionNoticeText_;
            lastControlResponseText_ = text;
            lastControlResponseDetail_ = text;
            appendLog(QStringLiteral("Vision capture: %1 succeeded").arg(action));
            refreshVisionUi();
        });
        connect(visionControlClient_, &vision::VisionControlClient::errorOccurred,
                this, [this](const QString &message) {
            const QString text = QStringLiteral("Error: %1").arg(message);
            visionNoticeText_ = text;
            visionNoticeTooltip_ = message;
            lastControlResponseText_ = text;
            lastControlResponseDetail_ = message;
            refreshVisionUi();
        });
        connect(visionControlClient_, &vision::VisionControlClient::requestStateChanged,
                this, [this] {
            refreshVisionUi();
            syncDetectionStream();
        });
        connect(visionControlClient_,
                &vision::VisionControlClient::authoritativeStatusRefreshed,
                this, [this] {
            visionOutcomeUncertain_ = false;
            visionNoticeText_.clear();
            visionNoticeTooltip_.clear();
            refreshVisionUi();
            refreshPiHostUi();
        });
        connect(visionControlClient_,
                &vision::VisionControlClient::inferenceActionAcknowledged,
                this, [this](const QString &action, const QString &outcome) {
            visionOutcomeUncertain_ = false;
            const QString text = QStringLiteral(
                "ACK: %1 accepted; confirming status").arg(action);
            visionNoticeText_ = text;
            visionNoticeTooltip_ = outcome;
            lastControlResponseText_ = text;
            lastControlResponseDetail_ = outcome;
            refreshVisionUi();
        });
        connect(visionControlClient_, &vision::VisionControlClient::requestFailed,
                this, [this](const QString &action, const QString &code,
                             const QString &message, bool uncertain) {
            visionOutcomeUncertain_ = uncertain;
            const QString text = QStringLiteral(
                "Failure: %1: %2").arg(action, message);
            const QString detail = QStringLiteral("%1 (%2)").arg(message, code);
            visionNoticeText_ = text;
            visionNoticeTooltip_ = detail;
            lastControlResponseText_ = text;
            lastControlResponseDetail_ = detail;
            appendLog(QStringLiteral("Vision %1 [%2]: %3")
                          .arg(action, code, message));
            refreshVisionUi();
        });
    }

    if (detectionClient_ != nullptr) {
        connect(detectionClient_, &vision::DetectionClient::metadataReady,
                this, [this](vision::DetectionFrame frame) {
            latestDetectionFrame_ = std::move(frame);
            visualSession_->onDetectionArrival(*latestDetectionFrame_, visualDisplayContext());
        });
        connect(detectionClient_, &vision::DetectionClient::connectionStateChanged,
                this, [this](vision::DetectionConnectionState state) {
            if (state == vision::DetectionConnectionState::Connected) {
                latestDetectionFrame_.reset();
                visualSession_->beginSession(++visualSessionId_);
                refreshDetectionOverlay();
            }
            if (state == vision::DetectionConnectionState::Disconnected
                || state == vision::DetectionConnectionState::Error) {
                latestDetectionFrame_.reset();
                visualSession_->beginSession(++visualSessionId_);
                refreshDetectionOverlay();
            }
        });
        connect(detectionClient_, &vision::DetectionClient::logMessage,
                this, [this](const QString &message) {
            appendLog(QStringLiteral("Detection: %1").arg(message));
        });
        connect(detectionClient_, &vision::DetectionClient::protocolError,
                this, [this](const QString &message) {
            appendLog(QStringLiteral("Detection: %1").arg(message));
        });
    }

    refreshVisionUi();
    syncDetectionStream();
    refreshPiHostUi();
    refreshVisionEndpointUi();
}

void MainWindow::bindControllerUi()
{
    connect(controller_, &IConsoleController::serialPortsChanged, this,
            [this](const QStringList &ports) {
        if (serialPortCombo_ == nullptr) {
            return;
        }
        const QString current = serialPortCombo_->currentText();
        serialPortCombo_->clear();
        serialPortCombo_->addItems(ports);
        if (!current.isEmpty() && serialPortCombo_->findText(current) < 0) {
            serialPortCombo_->addItem(current);
        }
        serialPortCombo_->setCurrentText(current);
    });
    connect(controller_, &IConsoleController::connectionStateChanged, this,
            [this](TransportState state) {
        transportState_ = state;
        connectionStatus_->setText(stateText(state));
        setConnectedUi(state == TransportState::Connected);
        refreshAuthorityUi();
        refreshPiHostUi();
    });
    connect(controller_, &IConsoleController::controlAvailabilityChanged,
            this, [this] { refreshAuthorityUi(); });
    connect(controller_, &IConsoleController::authorityStateChanged,
            this, [this](ControlAuthorityState, bool) {
        refreshAuthorityUi();
        for (int index = 0; index < kServoCount; ++index) {
            refreshServoUi(index);
        }
        refreshMotionUi();
        refreshGaitSelectorsUi();
    });
    connect(controller_, &IConsoleController::servoStateChanged,
            this, [this](int index, bool enabled) {
        if (index < 0 || index >= kServoCount) {
            return;
        }
        Q_UNUSED(enabled);
        refreshServoUi(index);
        refreshMotionUi();
    });
    connect(controller_, &IConsoleController::servoDisablePendingChanged,
            this, [this](int index, bool) {
        if (index < 0 || index >= kServoCount) {
            return;
        }
        refreshServoUi(index);
        refreshMotionUi();
    });
    connect(controller_, &IConsoleController::motionStateChanged,
            this, [this](MotionState, MotionMode) {
        refreshMotionUi();
        refreshGaitSelectorsUi();
        for (int index = 0; index < kServoCount; ++index) {
            refreshServoUi(index);
        }
    });
    connect(controller_, &IConsoleController::gaitBackendStateChanged,
            this, &MainWindow::refreshGaitSelectorsUi);
    connect(controller_, &IConsoleController::frontRearCoordinationStateChanged,
            this, &MainWindow::refreshGaitSelectorsUi);
    connect(controller_, &IConsoleController::leakStateChanged,
            this, &MainWindow::setLeakUiState);
    connect(controller_, &IConsoleController::imuStateChanged, this,
            [this] { setImuUiState(controller_->imuState()); });
    connect(controller_, &IConsoleController::depthStateChanged, this,
            [this] { setDepthUiState(controller_->depthState()); });
    connect(controller_, &IConsoleController::txHexChanged,
            txHex_, &QLineEdit::setText);
    connect(controller_, &IConsoleController::rxHexChanged,
            rxHex_, &QLineEdit::setText);
    connect(controller_, &IConsoleController::protocolMonitorChanged,
            this, [this](const ProtocolMonitor &monitor) {
        refreshProtocolUi(monitor);
    });
    connect(controller_, &IConsoleController::logMessage,
            this, &MainWindow::appendLog);
}

void MainWindow::refreshVideoDiagnosticsUi()
{
    if (visionClient_ == nullptr) {
        return;
    }
    const bool live = visionClient_->state() == vision::VisionConnectionState::Connected
        && haveVisionReceiveSample_;
    if (videoFpsSummary_ != nullptr) {
        videoFpsSummary_->setText(
            QStringLiteral("Video FPS %1")
                .arg(live ? QString::number(visionClient_->receivedFps(), 'f', 1)
                          : QStringLiteral("--")));
    }
    if (videoResolutionSummary_ != nullptr) {
        videoResolutionSummary_->setText(
            live && visionFrameSize_.isValid()
                ? QStringLiteral("Resolution %1 × %2")
                      .arg(visionFrameSize_.width())
                      .arg(visionFrameSize_.height())
                : QStringLiteral("Resolution --"));
    }
    if (visionDiagnostics_ != nullptr) {
        visionDiagnostics_->setText(
            QStringLiteral("%1\nRX FPS: %2 | Display paint-event FPS: %3 | Latest frame ID: %4\n"
                           "Wire frames: %5 | RX replaced: %6 | JPEG decode drops: %7 | UI replaced: %8")
                .arg(live ? QStringLiteral("Fresh") : QStringLiteral("Last received / stale"))
                .arg(live ? QString::number(visionClient_->receivedFps(), 'f', 1)
                          : QStringLiteral("--"))
                .arg(videoView_ != nullptr
                         ? QString::number(videoView_->displayedFps(), 'f', 1)
                         : QStringLiteral("--"))
                .arg(haveVisionReceiveSample_ ? QString::number(visionClient_->lastFrameId())
                                              : QStringLiteral("--"))
                .arg(visionClient_->totalWireFrames())
                .arg(visionClient_->replacedWireFrames())
                .arg(live ? QString::number(visionJpegDecodeErrors_)
                          : QStringLiteral("--"))
                .arg(videoView_ != nullptr
                         ? QString::number(videoView_->replacedPendingFrames())
                         : QStringLiteral("--")));
    }
}

void MainWindow::refreshProtocolUi(const ProtocolMonitor &monitor)
{
    if (txCount_ == nullptr) {
        return;
    }
    txCount_->setText(QString::number(monitor.txPacketCount));
    rxCount_->setText(QString::number(monitor.rxPacketCount));
    crcCount_->setText(QString::number(monitor.crcErrorCount));
    timeoutCount_->setText(QString::number(monitor.timeoutCount));
    ackRtt_->setText(monitor.lastAckRttMs < 0
                         ? QStringLiteral("—")
                         : QStringLiteral("%1 ms").arg(monitor.lastAckRttMs));
    ackStatus_->setText(monitor.ackStatus);
    if (protocolCountersDetails_ != nullptr) {
        protocolCountersDetails_->setText(
            QStringLiteral("TX: %1\nRX: %2\nCRC: %3\nTimeout: %4\nACK RTT: %5")
                .arg(monitor.txPacketCount)
                .arg(monitor.rxPacketCount)
                .arg(monitor.crcErrorCount)
                .arg(monitor.timeoutCount)
                .arg(monitor.lastAckRttMs < 0
                         ? QStringLiteral("unavailable")
                         : QStringLiteral("%1 ms").arg(monitor.lastAckRttMs)));
    }
}

void MainWindow::refreshVisionNoticeUi()
{
    const auto status = visionControlClient_ != nullptr
        ? visionControlClient_->status()
        : vision::VisionCaptureStatus{};
    QString notice;
    if (!status.inferenceLastError.trimmed().isEmpty()) {
        notice = QStringLiteral("Inference: %1").arg(status.inferenceLastError);
    } else if (!status.lastError.trimmed().isEmpty()) {
        notice = QStringLiteral("Capture: %1").arg(status.lastError);
    } else if (!visionNoticeText_.isEmpty()) {
        notice = visionNoticeText_.startsWith(QStringLiteral("Control:"))
            ? visionNoticeText_
            : QStringLiteral("Control: %1").arg(visionNoticeText_);
    } else if (visionOutcomeUncertain_) {
        notice = QStringLiteral(
            "Control: Vision action outcome uncertain; refresh status before acting again");
    }
    if (visionControlMessage_ != nullptr) {
        visionControlMessage_->setFullText(notice);
        visionControlMessage_->setToolTip(
            visionNoticeTooltip_.isEmpty() ? notice : visionNoticeTooltip_);
    }
    if (controlResponseDetails_ != nullptr) {
        if (!lastControlResponseText_.isEmpty()) {
            controlResponseDetails_->setText(
                QStringLiteral("%1\nTooltip / detail: %2")
                    .arg(lastControlResponseText_, lastControlResponseDetail_.isEmpty()
                                                ? QStringLiteral("--")
                                                : lastControlResponseDetail_));
        } else if (visionOutcomeUncertain_) {
            controlResponseDetails_->setText(
                QStringLiteral("Uncertain result: refresh status before acting again"));
        } else {
            controlResponseDetails_->setText(QStringLiteral("--"));
        }
    }
}

void MainWindow::refreshVisionEndpointUi()
{
    if (visionEndpointDetails_ == nullptr) {
        return;
    }
    const QString host = bracketedEndpointHost(committedPiHost_);
    const bool remote = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp;
    const QString robot = remote
        ? QStringLiteral("Robot TCP: %1:%2")
              .arg(host)
              .arg(robotTcpPort_ != nullptr ? robotTcpPort_->value() : 47000)
        : QStringLiteral("Robot: Direct serial mode");
    const quint16 httpPort = visionControlClient_ != nullptr
        ? visionControlClient_->port()
        : vision::kVisionControlDefaultPort;
    const quint16 videoPort = visionPort_ != nullptr
        ? static_cast<quint16>(visionPort_->value())
        : 47010;
    visionEndpointDetails_->setText(
        QStringLiteral("Committed Pi Host: %1\n%2\nRBVS endpoint: tcp://%3:%4\n"
                       "Vision HTTP endpoint: http://%3:%5")
            .arg(committedPiHost_)
            .arg(robot)
            .arg(host)
            .arg(videoPort)
            .arg(httpPort));
}

void MainWindow::resetDetectionSession()
{
    latestDetectionFrame_.reset();
    if (visualSession_) { visualSession_->beginSession(++visualSessionId_); }
    detectionConnectAttempted_ = false;
    detectionAttemptHost_.clear();
    detectionAttemptPort_ = 0U;
    refreshDetectionOverlay();
    if (detectionClient_ != nullptr
        && (detectionClient_->endpointBusy()
            || detectionClient_->state()
                != vision::DetectionConnectionState::Disconnected)) {
        detectionClient_->disconnectFromHost();
    }
}

void MainWindow::syncDetectionStream()
{
    if (detectionClient_ == nullptr) {
        return;
    }

    const bool videoConnected =
        visionClient_ != nullptr && visionClient_->isConnected();
    const bool statusFresh =
        visionControlClient_ != nullptr
        && visionControlClient_->hasFreshStatus();

    if (!videoConnected) {
        resetDetectionSession();
        return;
    }

    if (!statusFresh) {
        refreshDetectionOverlay();
        return;
    }

    const auto status = visionControlClient_->status();
    const bool supported =
        status.detectionStreamSupported.has_value()
        && *status.detectionStreamSupported
        && status.detectionStreamPort.has_value()
        && status.detectionStreamVersion.has_value()
        && *status.detectionStreamVersion == vision::kDetectionStreamVersion;

    if (!supported) {
        latestDetectionFrame_.reset();
        refreshDetectionOverlay();
        if (detectionClient_->endpointBusy()
            || detectionClient_->state()
                != vision::DetectionConnectionState::Disconnected) {
            detectionClient_->disconnectFromHost();
        }
        detectionConnectAttempted_ = false;
        detectionAttemptHost_.clear();
        detectionAttemptPort_ = 0U;
        return;
    }

    const QString host = normalizedHostCandidate(committedPiHost_);
    const quint16 port = *status.detectionStreamPort;
    if (!hostCandidateValid(host) || port == 0U) {
        return;
    }

    const bool sameAttempt =
        detectionConnectAttempted_
        && detectionAttemptHost_ == host
        && detectionAttemptPort_ == port;
    const bool sameLiveEndpoint =
        detectionClient_->host() == host
        && detectionClient_->port() == port
        && (detectionClient_->state()
                == vision::DetectionConnectionState::Connecting
            || detectionClient_->state()
                == vision::DetectionConnectionState::Connected);

    if (sameLiveEndpoint || sameAttempt) {
        return;
    }

    if (detectionClient_->endpointBusy()
        || detectionClient_->state()
            != vision::DetectionConnectionState::Disconnected) {
        detectionClient_->disconnectFromHost();
    }

    detectionConnectAttempted_ = true;
    detectionAttemptHost_ = host;
    detectionAttemptPort_ = port;
    detectionClient_->connectToHost(host, port);
}

void MainWindow::refreshDetectionOverlay()
{
    if (visualSession_) { visualSession_->refresh(visualDisplayContext()); }
}

vision::VisualViewContext MainWindow::visualDisplayContext() const
{
    vision::VisualViewContext context;
    if (!videoView_ || !visionControlClient_) { return context; }

    const bool fresh = visionControlClient_->hasFreshStatus();
    const auto status = visionControlClient_->status();
    const bool knownWorkerState = status.inferenceState == QStringLiteral("disabled")
        || status.inferenceState == QStringLiteral("starting")
        || status.inferenceState == QStringLiteral("running")
        || status.inferenceState == QStringLiteral("failed")
        || status.inferenceState == QStringLiteral("error");
    if (!fresh || !status.haveInferenceState || !status.inferenceOperationValid
        || !knownWorkerState || visionControlClient_->inferenceReconcilePending()) {
        return context;
    }
    const bool inferenceRunning = status.inferenceState == QStringLiteral("running")
        && status.inferenceOperation.isEmpty();
    if (!inferenceRunning) {
        context.gate = vision::DetectionDisplayState::InferenceOff;
        return context;
    }
    if (detectionClient_ == nullptr || !detectionClient_->isConnected()
        || !latestDetectionFrame_.has_value()) {
        return context;
    }

    context.gate = vision::detectionOverlayState(*latestDetectionFrame_,
        videoView_->currentFrameSize(), videoView_->currentCaptureTimestampNs(), inferenceRunning, fresh);
    if (vision::detectionOverlayRenderable(
            *latestDetectionFrame_,
            videoView_->currentFrameSize(),
            videoView_->currentCaptureTimestampNs(),
            inferenceRunning,
            fresh)) {
        context.selected = vision::selectTargetState(*latestDetectionFrame_);
    }

    return context;
}

void MainWindow::reflowActuatorCards()
{
    if (actuatorGrid_ == nullptr || actuatorCardsHost_ == nullptr) {
        return;
    }
    constexpr int columns = 1;
    int actualColumns = 0;
    for (int index = 0; index < actuatorGrid_->count(); ++index) {
        int row = 0;
        int column = 0;
        int rowSpan = 0;
        int columnSpan = 0;
        actuatorGrid_->getItemPosition(
            index, &row, &column, &rowSpan, &columnSpan);
        actualColumns = qMax(actualColumns, column + columnSpan);
    }
    if (actuatorColumnCount_ == columns
        && actualColumns == columns) {
        return;
    }
    while (QLayoutItem *item = actuatorGrid_->takeAt(0)) {
        delete item;
    }
    for (int index = 0; index < kServoCount; ++index) {
        actuatorGrid_->addWidget(
            servoPanels_[static_cast<std::size_t>(index)],
            index, 0);
    }
    actuatorColumnCount_ = columns;
}

void MainWindow::updateVideoSurfaceGeometry()
{
    if (videoView_ == nullptr || videoContentHost_ == nullptr) {
        return;
    }
    const QSize frameSize = visionFrameSize_.isValid()
        ? visionFrameSize_ : QSize(640, 480);
    const int frameWidth = qMax(1, frameSize.width());
    const int frameHeight = qMax(1, frameSize.height());
    const int ratioDivisor = std::gcd(frameWidth, frameHeight);
    const int ratioWidth = frameWidth / ratioDivisor;
    const int ratioHeight = frameHeight / ratioDivisor;

    const int hostHeight =
        qMax(160, videoContentHost_->contentsRect().height());
    const int hostWidth =
        qMax(240, videoContentHost_->contentsRect().width());

    // Reserve a bounded share for Vision Status. Then choose an exact
    // integer multiple of the source aspect ratio. Using rounded width and
    // height independently can make a nominal 4:3 surface a few pixels off,
    // which VideoView::KeepAspectRatio exposes as a black edge.
    const int reservedSummaryWidth =
        qBound(210, qRound(hostWidth * 0.28), 360);
    const int availableVideoWidth =
        qMax(240, hostWidth - reservedSummaryWidth - 8);
    const int ratioUnits = qMax(
        1,
        qMin(availableVideoWidth / ratioWidth,
             hostHeight / ratioHeight));
    const int targetWidth = ratioWidth * ratioUnits;
    const int targetHeight = ratioHeight * ratioUnits;
    videoView_->setFixedSize(targetWidth, targetHeight);
}

void MainWindow::initializeWorkspaceSizes()
{
    if (workspaceSplitter_ == nullptr) {
        return;
    }
    if (workspaceSplitter_->height() <= 0) {
        if (!workspaceSizeInitPending_) {
            workspaceSizeInitPending_ = true;
            QTimer::singleShot(0, this, [this] {
                workspaceSizeInitPending_ = false;
                initializeWorkspaceSizes();
            });
        }
        return;
    }
    const int availableHeight = qMax(0, height() - 100);
    const int toolsHeight = availableHeight > 760 ? 260 : 240;
    const int total = qMax(400, workspaceSplitter_->height());
    workspaceSplitter_->setSizes({qMax(1, total - toolsHeight), toolsHeight});
    workspaceSplitter_->setStretchFactor(0, 1);
    workspaceSplitter_->setStretchFactor(1, 0);
    for (int index = 0; index < workspaceSplitter_->count(); ++index) {
        workspaceSplitter_->setCollapsible(index, false);
    }
}

void MainWindow::setLeakUiState(LeakState state)
{
    if (leakStatus_ == nullptr || leakDot_ == nullptr) {
        return;
    }
    const QString stateText = leakStateDisplayText(state);
    leakStatus_->setText(stateText.startsWith(QStringLiteral("Leak: "))
                             ? stateText.mid(6)
                             : stateText);
    switch (state) {
    case LeakState::Unknown:
        leakStatus_->setStyleSheet(QStringLiteral("color: #666666; font-size: 12px; font-weight: 600;"));
        leakDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
        break;
    case LeakState::Dry:
        leakStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-size: 12px; font-weight: 600;"));
        leakDot_->setStyleSheet(QStringLiteral("background: #43A047; border-radius: 5px;"));
        break;
    case LeakState::Wet:
        leakStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-size: 12px; font-weight: 600;"));
        leakDot_->setStyleSheet(QStringLiteral("background: #E53935; border-radius: 5px;"));
        break;
    }
}

void MainWindow::setImuUiState(const ImuMonitorState &state)
{
    if (imuStatus_ == nullptr) {
        return;
    }

    imuStatus_->setText(imuStatusText(state.status));
    switch (state.status) {
    case ImuStatus::Unknown:
        imuStatus_->setStyleSheet(QStringLiteral("color: #666666; font-size: 12px; font-weight: 600;"));
        imuDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
        break;
    case ImuStatus::Receiving:
        imuStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-size: 12px; font-weight: 600;"));
        imuDot_->setStyleSheet(QStringLiteral("background: #43A047; border-radius: 5px;"));
        break;
    case ImuStatus::Stale:
        imuStatus_->setStyleSheet(QStringLiteral("color: #b35c00; font-size: 12px; font-weight: 600;"));
        imuDot_->setStyleSheet(QStringLiteral("background: #FB8C00; border-radius: 5px;"));
        break;
    case ImuStatus::Error:
        imuStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-size: 12px; font-weight: 600;"));
        imuDot_->setStyleSheet(QStringLiteral("background: #E53935; border-radius: 5px;"));
        break;
    }

    const bool live = state.status == ImuStatus::Receiving && state.snapshot.has_value();
    if (!live) {
        imuAcc_->setText(QStringLiteral("--"));
        imuGyro_->setText(QStringLiteral("--"));
        imuAngle_->setText(QStringLiteral("--"));
        imuDiagnostics_->setText(QStringLiteral("--"));
        return;
    }

    const ImuSnapshot &snapshot = *state.snapshot;
    imuAcc_->setText(snapshot.accValid() ? formatAcc(snapshot.accMg) : QStringLiteral("--"));
    imuGyro_->setText(snapshot.gyroValid()
                          ? formatGyro(snapshot.gyroDecidps)
                          : QStringLiteral("--"));
    imuAngle_->setText(snapshot.angleValid()
                           ? formatAngle(snapshot.angleCentidegrees)
                           : QStringLiteral("--"));
    const ImuDiagnostics &diagnostics = snapshot.diagnostics;
    imuDiagnostics_->setText(
        QStringLiteral("RX %1 | headers %2 | valid %3 | checksum %4 | overflow %5 | "
                       "re-arm %6 | UART %7 | Mag %8 | unsupported %9")
            .arg(diagnostics.rxByteCount)
            .arg(diagnostics.headerCount)
            .arg(diagnostics.validFrameCount)
            .arg(diagnostics.checksumErrorCount)
            .arg(diagnostics.ringOverflowCount)
            .arg(diagnostics.rearmFailureCount)
            .arg(diagnostics.uartErrorCount)
            .arg(diagnostics.magFrameCount)
            .arg(diagnostics.unsupportedFrameCount));
}

void MainWindow::setDepthUiState(const DepthMonitorState &state)
{
    if (depthStatus_ == nullptr) {
        return;
    }

    depthStatus_->setText(depthStatusText(state.status));
    switch (state.status) {
    case DepthStatus::Unknown:
        depthStatus_->setStyleSheet(QStringLiteral("color: #666666; font-size: 12px; font-weight: 600;"));
        depthDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
        break;
    case DepthStatus::Receiving:
        depthStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-size: 12px; font-weight: 600;"));
        depthDot_->setStyleSheet(QStringLiteral("background: #43A047; border-radius: 5px;"));
        break;
    case DepthStatus::Stale:
        depthStatus_->setStyleSheet(QStringLiteral("color: #b35c00; font-size: 12px; font-weight: 600;"));
        depthDot_->setStyleSheet(QStringLiteral("background: #FB8C00; border-radius: 5px;"));
        break;
    case DepthStatus::Error:
        depthStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-size: 12px; font-weight: 600;"));
        depthDot_->setStyleSheet(QStringLiteral("background: #E53935; border-radius: 5px;"));
        break;
    }

    const bool live = state.status == DepthStatus::Receiving && state.snapshot.has_value();
    if (!live) {
        depthValue_->setText(QStringLiteral("--"));
        depthTemperature_->setText(QStringLiteral("--"));
        depthAge_->setText(QStringLiteral("--"));
        if (!state.snapshot.has_value()) {
            depthDiagnostics_->setText(QStringLiteral("--"));
        } else {
            const DepthDiagnostics &diagnostics = state.snapshot->diagnostics;
            depthDiagnostics_->setText(
                QStringLiteral("RX %1 | valid lines %2 | parse errors %3 | overlong %4 | "
                               "overflow %5 | hard re-arm %6 | UART errors %7")
                    .arg(diagnostics.rxByteCount)
                    .arg(diagnostics.validLineCount)
                    .arg(diagnostics.parseErrorCount)
                    .arg(diagnostics.overlongLineCount)
                    .arg(diagnostics.ringOverflowCount)
                    .arg(diagnostics.hardRearmFailureCount)
                    .arg(diagnostics.uartErrorCount));
        }
        return;
    }

    const DepthSnapshot &snapshot = *state.snapshot;
    depthValue_->setText(snapshot.depthValid()
                              ? formatDepth(snapshot.depthMm)
                              : QStringLiteral("--"));
    depthTemperature_->setText(snapshot.temperatureValid()
                                    ? formatTemperature(snapshot.temperatureCentiC)
                                    : QStringLiteral("--"));
    depthAge_->setText(snapshot.sampleAgeMs == DepthSnapshot::UnknownSampleAgeMs
                           ? QStringLiteral("--")
                           : QStringLiteral("%1 ms").arg(snapshot.sampleAgeMs));
    const DepthDiagnostics &diagnostics = snapshot.diagnostics;
    depthDiagnostics_->setText(
        QStringLiteral("RX %1 | valid lines %2 | parse errors %3 | overlong %4 | "
                       "overflow %5 | hard re-arm %6 | UART errors %7")
            .arg(diagnostics.rxByteCount)
            .arg(diagnostics.validLineCount)
            .arg(diagnostics.parseErrorCount)
            .arg(diagnostics.overlongLineCount)
            .arg(diagnostics.ringOverflowCount)
            .arg(diagnostics.hardRearmFailureCount)
            .arg(diagnostics.uartErrorCount));
}

void MainWindow::setConnectedUi(bool connected)
{
    connectButton_->setText(connected ? QStringLiteral("Disconnect") : QStringLiteral("Connect"));
    if (serialPortCombo_ != nullptr) {
        serialPortCombo_->setEnabled(!connected);
    }
    if (serialBaud_ != nullptr) {
        serialBaud_->setEnabled(!connected);
    }
    if (robotTcpPort_ != nullptr) {
        robotTcpPort_->setEnabled(!connected);
    }
    for (int index = 0; index < kServoCount; ++index) {
        refreshServoUi(index);
    }
    refreshMotionUi();
    refreshGaitSelectorsUi();
    refreshAuthorityUi();
}

void MainWindow::refreshServoUi(int index)
{
    if (index < 0 || index >= kServoCount) {
        return;
    }
    const ServoDescriptor *descriptor = servoDescriptor(static_cast<quint8>(index));
    if (descriptor == nullptr) {
        return;
    }
    const ServoId id = descriptor->id;
    const bool connected = controller_->isConnected();
    const bool active = controller_->isControlActive();
    const bool supported = controller_->isServoSupported(id);
    const bool enabled = controller_->isServoEnabled(id);
    const bool pendingDisable = controller_->isServoDisablePending(id);
    const bool motionActive = controller_->isMotionActive();
    if (enableButtons_[index] != nullptr) {
        enableButtons_[index]->setText(
            enabled ? QStringLiteral("Release") : QStringLiteral("Enable"));
        enableButtons_[index]->setToolTip(
            enabled
                ? QStringLiteral(
                      "Release PWM drive without sending Neutral; "
                      "the servo is no longer actively held.")
                : QStringLiteral(
                      "Enable PWM drive and hold the calibrated neutral position."));
        enableButtons_[index]->setEnabled(
            active && supported && !motionActive);
    }
    if (neutralButtons_[index] != nullptr) {
        neutralButtons_[index]->setEnabled(
            active && supported && enabled
            && !pendingDisable && !motionActive);
    }
    if (applyButtons_[index] != nullptr) {
        const int previewPwm = pwmSpins_[index] == nullptr
            ? static_cast<int>(descriptor->neutralPwmUs)
            : pwmSpins_[index]->value();
        const bool rawPwmInRange =
            previewPwm >= descriptor->commandMinPwmUs
            && previewPwm <= descriptor->commandMaxPwmUs;
        applyButtons_[index]->setEnabled(
            active && controller_->supportsRawPwm()
            && supported && enabled && rawPwmInRange
            && !pendingDisable && !motionActive);
        applyButtons_[index]->setToolTip(
            rawPwmInRange
                ? QStringLiteral("Apply this raw PWM value. Slider edits do not transmit.")
                : QStringLiteral("Equivalent PWM is outside the manual raw PWM range %1-%2 us; use Apply Angle instead.")
                      .arg(descriptor->commandMinPwmUs)
                      .arg(descriptor->commandMaxPwmUs));
    }
    QString statusColor = QStringLiteral("#6F7F8B");
    if (!connected) {
        statusLabels_[index]->setText(QStringLiteral("Disconnected"));
    } else if (!active) {
        statusLabels_[index]->setText(QStringLiteral("No control authority"));
    } else if (!supported) {
        statusLabels_[index]->setText(QStringLiteral("Unsupported"));
    } else if (pendingDisable) {
        statusLabels_[index]->setText(QStringLiteral("Disable pending ACK"));
        statusColor = QStringLiteral("#b35c00");
    } else if (enabled) {
        statusLabels_[index]->setText(QStringLiteral("Enabled / ACKed"));
        statusColor = QStringLiteral("#2F80ED");
    } else if (descriptor->calibrationPending) {
        statusLabels_[index]->setText(QStringLiteral("Disabled — Calibration Pending"));
    } else {
        statusLabels_[index]->setText(QStringLiteral("Disabled"));
    }
    statusLabels_[index]->setStyleSheet(
        QStringLiteral("color: %1; font-size: 11px; font-weight: 600;").arg(statusColor));
    if (auto *statusDot = findChild<QLabel *>(QStringLiteral("servoStatusDot%1").arg(index))) {
        statusDot->setStyleSheet(
            QStringLiteral("background: %1; border-radius: 4px;").arg(statusColor));
    }
    setAngleUiEnabled(index, enabled);
}

void MainWindow::setAngleUiEnabled(int index, bool enabled)
{
    if (index < 0 || index >= kServoCount) {
        return;
    }
    const ServoDescriptor *descriptor = servoDescriptor(static_cast<quint8>(index));
    if (descriptor == nullptr) {
        return;
    }
    const ServoId id = descriptor->id;
    const bool actionable = enabled && controller_->isControlActive()
        && controller_->isServoSupported(id) && descriptor->angleSupported
        && !descriptor->calibrationPending && controller_->isServoEnabled(id)
        && !controller_->isServoDisablePending(id)
        && !controller_->isMotionActive();
    if (angleSpins_[index] != nullptr) {
        angleSpins_[index]->setEnabled(actionable);
    }
    if (angleApplyButtons_[index] != nullptr) {
        angleApplyButtons_[index]->setEnabled(actionable);
    }
}

void MainWindow::refreshMotionUi()
{
    if (motionStopButton_ == nullptr || motionStatus_ == nullptr) {
        return;
    }

    const MotionState state = controller_->motionState();
    switch (state) {
    case MotionState::Stopped:
        motionStatus_->setText(QStringLiteral("Stopped"));
        break;
    case MotionState::Running:
        motionStatus_->setText(QStringLiteral("Running — %1")
                                   .arg(motionModeText(controller_->motionMode())));
        break;
    case MotionState::Stopping:
        motionStatus_->setText(QStringLiteral("Stopping"));
        break;
    case MotionState::Faulted:
        motionStatus_->setText(QStringLiteral("Faulted"));
        break;
    }

    const bool connected = controller_->isControlActive();
    if (disableAllButton_ != nullptr) {
        disableAllButton_->setEnabled(connected);
    }
    if (enableAllButton_ != nullptr) {
        bool haveEnableCandidate = false;
        const auto &descriptors = servoDescriptorTable();
        for (const auto &descriptor : descriptors) {
            haveEnableCandidate = haveEnableCandidate
                || (controller_->isServoSupported(descriptor.id)
                    && !controller_->isServoEnabled(descriptor.id)
                    && !controller_->isServoDisablePending(descriptor.id));
        }
        enableAllButton_->setEnabled(connected && !controller_->isMotionActive()
                                     && haveEnableCandidate);
    }
    const bool transitioning = controller_->isMotionTransitioning();
    for (int index = 0; index < static_cast<int>(motionButtons_.size()); ++index) {
        QPushButton *button = motionButtons_[static_cast<std::size_t>(index)];
        if (button == nullptr) {
            continue;
        }
        const MotionMode mode = static_cast<MotionMode>(index);
        const bool pendingMode = mode == MotionMode::Backward;
        button->setChecked(
            !pendingMode && state == MotionState::Running
            && controller_->motionMode() == mode);
        button->setEnabled(
            !pendingMode && connected && state != MotionState::Stopping
            && !transitioning && controller_->isMotionReady(mode));
    }
    motionStopButton_->setEnabled(connected && controller_->isMotionActive());
}

void MainWindow::refreshGaitSelectorsUi()
{
    if (gaitBackendCombo_ == nullptr || gaitBackendStatus_ == nullptr
        || frontRearCoordinationCombo_ == nullptr
        || frontRearCoordinationStatus_ == nullptr) {
        return;
    }

    const bool connected = controller_->isControlActive();
    const bool backendPending = controller_->isGaitBackendChangePending();
    const bool coordinationPending =
        controller_->isFrontRearCoordinationChangePending();
    const std::optional<GaitBackend> displayBackend = backendPending
        ? controller_->requestedGaitBackend()
        : controller_->confirmedGaitBackend();
    const std::optional<FrontRearCoordination> displayCoordination =
        coordinationPending
        ? controller_->requestedFrontRearCoordination()
        : controller_->confirmedFrontRearCoordination();
    int displayIndex = gaitBackendCombo_->findData(-1);
    if (displayBackend.has_value()) {
        displayIndex = gaitBackendCombo_->findData(
            static_cast<int>(*displayBackend));
    }
    int coordinationIndex = frontRearCoordinationCombo_->findData(-1);
    if (displayCoordination.has_value()) {
        coordinationIndex = frontRearCoordinationCombo_->findData(
            static_cast<int>(*displayCoordination));
    }

    const bool selectorsEnabled = connected && !controller_->isMotionActive()
        && !backendPending && !coordinationPending;
    {
        const QSignalBlocker blocker(gaitBackendCombo_);
        gaitBackendCombo_->setCurrentIndex(displayIndex);
    }
    {
        const QSignalBlocker blocker(frontRearCoordinationCombo_);
        frontRearCoordinationCombo_->setCurrentIndex(coordinationIndex);
    }
    gaitBackendCombo_->setEnabled(selectorsEnabled);
    frontRearCoordinationCombo_->setEnabled(selectorsEnabled);

    const std::optional<GaitBackend> confirmedBackend =
        controller_->confirmedGaitBackend();
    const std::optional<FrontRearCoordination> confirmedCoordination =
        controller_->confirmedFrontRearCoordination();
    const QString backendStatus = connected && confirmedBackend.has_value()
        ? QStringLiteral("Confirmed — %1").arg(
              gaitBackendCombo_->itemText(gaitBackendCombo_->findData(
                  static_cast<int>(*confirmedBackend))))
        : QStringLiteral("Unknown");
    const QString coordinationStatus =
        connected && confirmedCoordination.has_value()
        ? QStringLiteral("Confirmed — %1").arg(
              frontRearCoordinationCombo_->itemText(
                  frontRearCoordinationCombo_->findData(
                      static_cast<int>(*confirmedCoordination))))
        : QStringLiteral("Unknown");
    gaitBackendStatus_->setText(backendStatus);
    frontRearCoordinationStatus_->setText(coordinationStatus);
}

void MainWindow::refreshAuthorityUi()
{
    if (authorityStatus_ == nullptr || acquireButton_ == nullptr
        || releaseButton_ == nullptr) {
        return;
    }

    if (controller_->backendKind() == ConsoleBackendKind::DirectSerial) {
        authorityStatus_->setText(QStringLiteral("Direct/APC"));
        acquireButton_->setEnabled(false);
        releaseButton_->setEnabled(false);
        return;
    }

    QString text;
    switch (controller_->authorityState()) {
    case ControlAuthorityState::Unowned:
        text = QStringLiteral("Unowned");
        break;
    case ControlAuthorityState::Acquiring:
        text = QStringLiteral("Acquiring");
        break;
    case ControlAuthorityState::Owned:
        text = controller_->isControlActive()
            ? QStringLiteral("Owned + Active")
            : QStringLiteral("Owned / Waiting Link");
        break;
    }
    authorityStatus_->setText(text);
    acquireButton_->setEnabled(controller_->canAcquireControl());
    releaseButton_->setEnabled(
        controller_->isConnected()
        && controller_->authorityState() == ControlAuthorityState::Owned);
}

void MainWindow::appendLog(const QString &message)
{
    log_->appendPlainText(message);
}

bool MainWindow::hostDirty() const
{
    return piHost_ != nullptr
        && normalizedHostCandidate(piHost_->text()) != committedPiHost_;
}

bool MainWindow::hasRemoteVisionWorkThatMayContinue() const
{
    if (visionControlClient_ == nullptr) {
        return false;
    }
    const auto status = visionControlClient_->status();
    return visionControlClient_->inferenceReconcilePending()
        || status.recording
        || status.state == QStringLiteral("stopping")
        || status.inferenceState == QStringLiteral("running")
        || status.inferenceState == QStringLiteral("starting")
        || status.inferenceOperation == QStringLiteral("stopping")
        || status.inferenceOperation == QStringLiteral("retrying")
        || visionOutcomeUncertain_
        || (visionControlClient_->isPolling() && !visionControlClient_->hasFreshStatus());
}

bool MainWindow::hostCandidateValid(const QString &candidate) const
{
    return validHostSyntax(candidate);
}

void MainWindow::refreshPiHostUi()
{
    if (piHost_ == nullptr) {
        return;
    }
    const bool dirty = hostDirty();
    const bool remoteBusy = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
        && (transportState_ == TransportState::Opening
            || transportState_ == TransportState::Connected
            || transportState_ == TransportState::Closing);
    const bool socketBusy = visionClient_ != nullptr && visionClient_->endpointBusy();
    const bool controlBusy = visionControlClient_ != nullptr
        && visionControlClient_->actionBusy();
    const bool safe = !remoteBusy && !socketBusy && !controlBusy && !updatingEndpoints_;
    applyPiHostButton_->setText(dirty ? QStringLiteral("Apply Host") : QStringLiteral("Applied"));
    applyPiHostButton_->setEnabled(dirty && hostCandidateValid(piHost_->text()) && safe);
    piHostHint_->setVisible(false);
    piHostHint_->clear();
    piHost_->setToolTip(
        dirty
            ? QStringLiteral("Edited value is not applied. Current endpoint: %1").arg(committedPiHost_)
            : QStringLiteral("Applied endpoint: %1").arg(committedPiHost_));
    applyPiHostButton_->setToolTip(
        dirty
            ? QStringLiteral("Apply this Pi Host to Robot, RBVS, and Vision HTTP endpoints.")
            : QStringLiteral("Pi Host is applied."));
    if (visionRefreshStatusButton_ != nullptr) {
        visionRefreshStatusButton_->setEnabled(
            visionControlClient_ != nullptr && !dirty);
        visionRefreshStatusButton_->setToolTip(
            QStringLiteral("Refresh HTTP status from the committed endpoint http://%1:%2")
                .arg(committedPiHost_)
                .arg(visionControlClient_ != nullptr
                         ? visionControlClient_->port()
                         : vision::kVisionControlDefaultPort));
    }
    if (visionConnectButton_ != nullptr) {
        visionConnectButton_->setEnabled(visionClient_ != nullptr && !dirty);
    }
    if (connectButton_ != nullptr) {
        // Keep an existing Disconnect available while a candidate is dirty,
        // but do not start a new Robot connection against either host value.
        connectButton_->setEnabled(controller_->isConnected() || !dirty);
    }
    if (visionControlClient_ != nullptr) {
        refreshInferenceUi();
        refreshCaptureUi();
    }
    refreshVisionEndpointUi();
}

void MainWindow::applyPiHost()
{
    if (piHost_ == nullptr) {
        return;
    }
    const QString candidate = normalizedHostCandidate(piHost_->text());
    if (!hostCandidateValid(candidate)) {
        visionNoticeText_ = QStringLiteral("Invalid Pi Host");
        visionNoticeTooltip_ = visionNoticeText_;
        refreshVisionNoticeUi();
        return;
    }
    if (candidate == committedPiHost_) {
        piHost_->setText(committedPiHost_);
        refreshPiHostUi();
        return;
    }
    const bool remoteBusy = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
        && (transportState_ == TransportState::Opening
            || transportState_ == TransportState::Connected
            || transportState_ == TransportState::Closing);
    const bool socketBusy = visionClient_ != nullptr && visionClient_->endpointBusy();
    const bool controlBusy = visionControlClient_ != nullptr
        && visionControlClient_->actionBusy();
    if (remoteBusy || socketBusy || controlBusy) {
        visionNoticeText_ = QStringLiteral("Pi Host is busy; wait for the active endpoint");
        visionNoticeTooltip_ = visionNoticeText_;
        refreshVisionNoticeUi();
        refreshPiHostUi();
        return;
    }
    if (hasRemoteVisionWorkThatMayContinue()) {
        const QString oldCommittedHost = committedPiHost_;
        updatingEndpoints_ = true;
        const auto answer = QMessageBox::question(
            this,
            QStringLiteral("Switch Pi Host"),
            QStringLiteral("Switch monitoring from %1 to %2? This does not stop inference or recording on %1.")
                .arg(committedPiHost_, candidate),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        updatingEndpoints_ = false;
        const bool remoteBusyAfter = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
            && (transportState_ == TransportState::Opening
                || transportState_ == TransportState::Connected
                || transportState_ == TransportState::Closing);
        const bool socketBusyAfter = visionClient_ != nullptr && visionClient_->endpointBusy();
        const bool controlBusyAfter = visionControlClient_ != nullptr
            && visionControlClient_->actionBusy();
        const bool candidateChanged = normalizedHostCandidate(piHost_->text()) != candidate;
        if (answer != QMessageBox::Yes || candidateChanged || committedPiHost_ != oldCommittedHost
            || remoteBusyAfter || socketBusyAfter || controlBusyAfter || !hostDirty()) {
            piHost_->setText(committedPiHost_);
            refreshPiHostUi();
            return;
        }
    }
    if (visionControlClient_ != nullptr
        && !visionControlClient_->setEndpoint(candidate, visionControlClient_->port())) {
        piHost_->setText(committedPiHost_);
        refreshPiHostUi();
        return;
    }
    updatingEndpoints_ = true;
    committedPiHost_ = candidate;
    piHost_->setText(committedPiHost_);
    // An explicit host-switch acknowledgement also acknowledges any prior
    // ambiguous POST outcome; the new endpoint is a fresh operator choice.
    visionOutcomeUncertain_ = false;
    visionNoticeText_.clear();
    visionNoticeTooltip_.clear();
    updatingEndpoints_ = false;
    resetDetectionSession();
    refreshPiHostUi();
}

void MainWindow::refreshVisionUi()
{
    refreshInferenceUi();
    refreshCaptureUi();
    refreshDetectionOverlay();
}

void MainWindow::refreshInferenceUi()
{
    if (inferenceState_ == nullptr || visionControlClient_ == nullptr) {
        return;
    }
    const auto rawStatus = visionControlClient_->status();
    const bool fresh = visionControlClient_->hasFreshStatus();
    const auto ui = vision::makeInferenceUiState(
        rawStatus,
        fresh,
        visionControlClient_->actionBusy(),
        visionControlClient_->inferenceReconcilePending());
    inferenceState_->setText(ui.stateText);

    if (inferenceDot_ != nullptr) {
        QString inferenceColor = QStringLiteral("#9e9e9e");
        const QString operation =
            rawStatus.inferenceOperationValid
                ? rawStatus.inferenceOperation
                : QString();
        if (!fresh || !rawStatus.haveInferenceStatus) {
            inferenceColor = QStringLiteral("#9e9e9e");
        } else if (operation == QStringLiteral("stopping")
                   || operation == QStringLiteral("retrying")
                   || rawStatus.inferenceState == QStringLiteral("starting")) {
            inferenceColor = QStringLiteral("#D39B2A");
        } else if (rawStatus.inferenceState == QStringLiteral("running")) {
            inferenceColor = QStringLiteral("#2F80ED");
        } else if (rawStatus.inferenceState == QStringLiteral("failed")) {
            inferenceColor = QStringLiteral("#C53F3F");
        }
        inferenceDot_->setStyleSheet(
            QStringLiteral("background: %1; border-radius: 5px;")
                .arg(inferenceColor));
    }

    const auto setActionRole = [](QPushButton *button, const char *role) {
        if (button == nullptr) {
            return;
        }
        const QString next = QString::fromLatin1(role);
        if (button->property("consoleActionRole").toString() == next) {
            return;
        }
        button->setProperty("consoleActionRole", next);
        button->style()->unpolish(button);
        button->style()->polish(button);
        button->update();
    };

    const QString operation =
        rawStatus.inferenceOperationValid
            ? rawStatus.inferenceOperation
            : QString();
    const bool failed =
        rawStatus.haveInferenceState
        && rawStatus.inferenceState == QStringLiteral("failed");
    const bool stopMode =
        rawStatus.haveInferenceState
        && (rawStatus.inferenceState == QStringLiteral("running")
            || rawStatus.inferenceState == QStringLiteral("starting"))
        && operation != QStringLiteral("stopping")
        && operation != QStringLiteral("retrying");

    if (operation == QStringLiteral("stopping")) {
        startInferenceButton_->setText(QStringLiteral("Stopping Inference..."));
        startInferenceButton_->setEnabled(false);
        setActionRole(startInferenceButton_, "stop");
    } else if (operation == QStringLiteral("retrying")) {
        startInferenceButton_->setText(QStringLiteral("Retrying Inference..."));
        startInferenceButton_->setEnabled(false);
        setActionRole(startInferenceButton_, "primary");
    } else if (stopMode) {
        startInferenceButton_->setText(ui.stopText);
        startInferenceButton_->setEnabled(ui.stopEnabled);
        setActionRole(startInferenceButton_, "stop");
    } else {
        startInferenceButton_->setText(ui.startText);
        startInferenceButton_->setEnabled(ui.startEnabled && !hostDirty());
        setActionRole(startInferenceButton_, "primary");
    }

    // Preserve Failed -> Clear Error as the only exceptional second action.
    // Normal Start/Stop is always one operator-facing toggle.
    stopInferenceButton_->setText(ui.stopText);
    stopInferenceButton_->setVisible(failed && operation.isEmpty());
    stopInferenceButton_->setEnabled(
        failed && operation.isEmpty() && ui.stopEnabled);
    setActionRole(stopInferenceButton_, "secondary");

    const auto displayStatus = rawStatus;
    if (!ui.showActiveMetrics) {
        if (inferencePerformanceSummary_ != nullptr) {
            inferencePerformanceSummary_->setText(QStringLiteral("-- FPS / -- ms"));
        }
        if (inferenceDetectionSummary_ != nullptr) {
            inferenceDetectionSummary_->setText(QStringLiteral("Detections --"));
        }
    } else {
        if (inferencePerformanceSummary_ != nullptr) {
            inferencePerformanceSummary_->setText(
                inferencePerformanceText(displayStatus, true));
        }
        if (inferenceDetectionSummary_ != nullptr) {
            inferenceDetectionSummary_->setText(
                inferenceDetectionText(displayStatus, true));
        }
    }
    if (inferenceMemorySummary_ != nullptr) {
        inferenceMemorySummary_->setText(
            inferenceMemoryText(displayStatus, ui.showActiveMetrics));
    }
    if (inferenceDiagnostics_ != nullptr) {
        inferenceDiagnostics_->setText(inferenceDetailsText(rawStatus, fresh, ui.reason));
    }
    if (inferenceArtifactValue_ != nullptr) {
        inferenceArtifactValue_->setText(displayStatus.inferenceArtifactName.isEmpty()
                                             ? QStringLiteral("--")
                                             : displayStatus.inferenceArtifactName);
    }
    if (inferenceShaValue_ != nullptr) {
        inferenceShaValue_->setText(displayStatus.inferenceModelSha256.isEmpty()
                                        ? QStringLiteral("--")
                                        : displayStatus.inferenceModelSha256);
        inferenceShaValue_->setToolTip(displayStatus.inferenceModelSha256);
    }
    if (inferenceThresholdValue_ != nullptr) {
        inferenceThresholdValue_->setText(displayStatus.haveInferenceConfidenceThreshold
                                              ? QString::number(displayStatus.inferenceConfidenceThreshold, 'f', 2)
                                              : QStringLiteral("--"));
    }
    if (inferenceOperationValue_ != nullptr) {
        inferenceOperationValue_->setText(displayStatus.inferenceOperationValid
                                              ? displayStatus.inferenceOperation
                                              : QStringLiteral("--"));
    }
    if (inferenceLastErrorValue_ != nullptr) {
        inferenceLastErrorValue_->setText(displayStatus.inferenceLastError.isEmpty()
                                              ? QStringLiteral("--")
                                              : displayStatus.inferenceLastError);
    }
    if (!visionControlClient_->hasFreshStatus()) {
        visionControlState_->setText(QStringLiteral("HTTP: Stale"));
    } else if (!visionControlClient_->status().haveInferenceStatus) {
        visionControlState_->setText(QStringLiteral("HTTP: Unavailable"));
    } else {
        visionControlState_->setText(QStringLiteral("HTTP: Reachable"));
    }
    refreshVisionNoticeUi();
}

void MainWindow::refreshCaptureUi()
{
    if (visionControlClient_ == nullptr || captureState_ == nullptr) {
        return;
    }
    const auto status = visionControlClient_->status();
    const bool fresh = visionControlClient_->hasFreshStatus();
    const bool busy = visionControlClient_->actionBusy();
    const bool usable = fresh && status.haveCameraStatus && status.haveCaptureStatus
        && status.cameraRunning;
    const bool captureFresh = fresh && status.haveCaptureStatus;
    captureState_->setText(
        captureFresh
            ? (status.recording
                   ? QStringLiteral("Capture Recording")
                   : QStringLiteral("Capture %1").arg(status.state))
            : QStringLiteral("Capture Unknown"));
    if (captureCountSummary_ != nullptr) {
        captureCountSummary_->setText(
            captureFresh
                ? QStringLiteral("Recorded %1 | Snapshots %2")
                      .arg(status.recordedFrames)
                      .arg(status.snapshotCount)
                : QStringLiteral("Recorded -- | Snapshots --"));
    }
    if (storageFreeSummary_ != nullptr) {
        storageFreeSummary_->setText(
            fresh && status.haveFreeDisk
                ? QStringLiteral("Storage Free %1 GiB")
                      .arg(static_cast<double>(status.freeDiskBytes)
                               / (1024.0 * 1024.0 * 1024.0),
                           0, 'f', 1)
                : QStringLiteral("Storage Free --"));
    }
    snapshotButton_->setEnabled(usable && !busy && !hostDirty());

    const auto setRecordingRole = [](QPushButton *button, const char *role) {
        if (button == nullptr) {
            return;
        }
        const QString next = QString::fromLatin1(role);
        if (button->property("consoleActionRole").toString() == next) {
            return;
        }
        button->setProperty("consoleActionRole", next);
        button->style()->unpolish(button);
        button->style()->polish(button);
        button->update();
    };

    if (captureFresh && status.state == QStringLiteral("stopping")) {
        startRecordingButton_->setText(QStringLiteral("Stopping Recording..."));
        startRecordingButton_->setEnabled(false);
        setRecordingRole(startRecordingButton_, "stop");
    } else if (captureFresh && status.recording) {
        startRecordingButton_->setText(QStringLiteral("Stop Recording"));
        startRecordingButton_->setEnabled(!busy);
        setRecordingRole(startRecordingButton_, "stop");
    } else {
        startRecordingButton_->setText(QStringLiteral("Start Recording"));
        startRecordingButton_->setEnabled(
            usable && !busy && !hostDirty());
        setRecordingRole(startRecordingButton_, "primary");
    }

    // Compatibility-only stop action: never presented as a second recording
    // control in the operator console.
    stopRecordingButton_->setEnabled(
        fresh && status.haveCaptureStatus && status.recording && !busy);
    stopRecordingButton_->setVisible(false);

    if (captureDiagnostics_ != nullptr) {
        captureDiagnostics_->setText(captureDetailsText(status, fresh));
    }
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == piHost_ && event->type() == QEvent::KeyPress) {
        auto *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Escape) {
            piHost_->setText(committedPiHost_);
            refreshPiHostUi();
            return true;
        }
    }
    if (watched == videoContentHost_ && event->type() == QEvent::Resize) {
        if (!videoGeometryUpdatePending_) {
            videoGeometryUpdatePending_ = true;
            QTimer::singleShot(0, this, [this] {
                videoGeometryUpdatePending_ = false;
                updateVideoSurfaceGeometry();
            });
        }
    }
    if ((watched == actuatorCardsHost_ || watched == actuatorScroll_
         || (actuatorScroll_ != nullptr && watched == actuatorScroll_->viewport()))
        && event->type() == QEvent::Resize) {
        // Reflow synchronously against the new viewport width so the operator
        // never sees a stale five-column layout after narrowing the window.
        reflowActuatorCards();
    }
    return QMainWindow::eventFilter(watched, event);
}

QString MainWindow::stateText(TransportState state)
{
    switch (state) {
    case TransportState::Disconnected: return QStringLiteral("Disconnected");
    case TransportState::Opening: return QStringLiteral("Opening");
    case TransportState::Connected: return QStringLiteral("Connected");
    case TransportState::Closing: return QStringLiteral("Closing");
    case TransportState::Error: return QStringLiteral("Error");
    }
    return QStringLiteral("Unknown");
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    appendLog(QStringLiteral("Application close requested"));
    if (visionControlClient_ != nullptr
        && visionControlClient_->actionBusy()) {
        visionNoticeText_ = QStringLiteral("Vision action busy; close deferred");
        visionNoticeTooltip_ = visionNoticeText_;
        refreshVisionNoticeUi();
        appendLog(QStringLiteral(
            "Vision control: application close deferred until the active action completes"));
        event->ignore();
        return;
    }
    if (visionControlClient_ != nullptr) {
        if (hasRemoteVisionWorkThatMayContinue()) {
            const auto answer = QMessageBox::question(
                this,
                QStringLiteral("Close RoboBeetle Console"),
                QStringLiteral("Vision work may continue on the committed Pi host. Close without sending Stop?"),
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No);
            if (answer != QMessageBox::Yes) {
                event->ignore();
                return;
            }
            if (visionControlClient_->actionBusy()) {
                event->ignore();
                return;
            }
        }
    }
    if (detectionClient_ != nullptr) {
        detectionClient_->shutdown();
    }
    if (visionControlClient_ != nullptr) {
        visionControlClient_->shutdown();
    }
    if (visionClient_ != nullptr) {
        visionClient_->shutdown();
    }
    controller_->shutdown();
    if (visualCsvLogger_) { visualCsvLogger_->stop(); }
    event->accept();
}

} // namespace rb
