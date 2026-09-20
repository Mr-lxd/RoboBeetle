#include "ui/MainWindow.h"
#include "ui/ElidedLabel.h"

#include "robot/ServoDescriptor.h"
#include "vision/VideoView.h"
#include "vision/VisionClient.h"
#include "vision/VisionControlClient.h"
#include "vision/InferenceUiState.h"

#include <QCloseEvent>
#include <QAbstractSpinBox>
#include <QComboBox>
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
#include <QScrollArea>
#include <QScreen>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QUrl>
#include <QTimer>

#include <array>

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
    case MotionMode::Backward: return QStringLiteral("Backward (Pending)");
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
    QWidget *parent)
    : QMainWindow(parent),
      controller_(controller),
      visionClient_(visionClient),
      visionControlClient_(visionControlClient)
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
        const int initialWidth = qMax(1100, qMin(1600, available.width() - 32));
        const int initialHeight = qMax(720, qMin(1000, available.height() - 80));
        resize(initialWidth, initialHeight);
        const QSize frameSize = frameGeometry().size();
        move(available.center()
             - QPoint(frameSize.width() / 2, frameSize.height() / 2));
    } else {
        resize(1600, 1000);
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
    refreshGaitBackendUi();
    refreshAuthorityUi();
    controller_->refreshSerialPorts();
}

QWidget *MainWindow::createConnectionBar()
{
    auto *bar = new QWidget(this);
    bar->setObjectName(QStringLiteral("topHeaderBar"));
    bar->setStyleSheet(QStringLiteral(
        "#topHeaderBar {"
        "  background: #FFFFFF;"
        "  border: 1px solid #C5D3DE;"
        "  border-radius: 7px;"
        "}"
    ));
    const bool remote =
        controller_->backendKind() == ConsoleBackendKind::RemoteRbrp;

    auto *layout = new QVBoxLayout(bar);
    layout->setContentsMargins(10, 7, 10, 7);
    layout->setSpacing(4);

    auto *row1 = new QHBoxLayout;
    row1->setContentsMargins(0, 0, 0, 0);
    row1->setSpacing(6);
    auto *title = new QLabel(QStringLiteral("RoboBeetle Console"), bar);
    title->setObjectName(QStringLiteral("consoleTitle"));
    title->setStyleSheet(QStringLiteral("font-weight: 700; font-size: 15px; color: #1F4058;"));
    row1->addWidget(title);
    row1->addSpacing(10);
    row1->addWidget(new QLabel(QStringLiteral("Pi Host"), bar));
    piHost_ = new QLineEdit(committedPiHost_, bar);
    piHost_->setObjectName(QStringLiteral("piHost"));
    piHost_->setPlaceholderText(QStringLiteral("Pi IP / hostname"));
    piHost_->setMinimumWidth(200);
    piHost_->installEventFilter(this);
    row1->addWidget(piHost_, 1);
    applyPiHostButton_ = new QPushButton(QStringLiteral("Apply Host"), bar);
    applyPiHostButton_->setObjectName(QStringLiteral("applyPiHostButton"));
    applyPiHostButton_->setProperty("consoleActionRole", "primary");
    row1->addWidget(applyPiHostButton_);
    row1->addWidget(new QLabel(QStringLiteral("Video"), bar));
    visionPort_ = new QSpinBox(bar);
    visionPort_->setObjectName(QStringLiteral("visionPort"));
    visionPort_->setRange(1, 65535);
    visionPort_->setValue(47010);
    visionPort_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    visionPort_->setKeyboardTracking(false);
    row1->addWidget(visionPort_);
    layout->addLayout(row1);

    auto *row2 = new QHBoxLayout;
    row2->setContentsMargins(0, 0, 0, 0);
    row2->setSpacing(6);
    piHostHint_ = new QLabel(bar);
    piHostHint_->setObjectName(QStringLiteral("piHostHint"));
    piHostHint_->setStyleSheet(QStringLiteral("font-size: 10px; color: #667C8C;"));
    piHostHint_->setVisible(false);

    if (remote) {
        row2->addWidget(new QLabel(QStringLiteral("Robot TCP"), bar));
        robotTcpPort_ = new QSpinBox(bar);
        robotTcpPort_->setObjectName(QStringLiteral("robotTcpPort"));
        robotTcpPort_->setRange(1, 65535);
        robotTcpPort_->setValue(47000);
        robotTcpPort_->setButtonSymbols(QAbstractSpinBox::NoButtons);
        robotTcpPort_->setKeyboardTracking(false);
        row2->addWidget(robotTcpPort_);
    } else {
        row2->addWidget(new QLabel(QStringLiteral("Serial Port"), bar));
        serialPortCombo_ = new QComboBox(bar);
        serialPortCombo_->setObjectName(QStringLiteral("serialPortCombo"));
        serialPortCombo_->setEditable(true);
        row2->addWidget(serialPortCombo_);
        row2->addWidget(new QLabel(QStringLiteral("Baud"), bar));
        serialBaud_ = new QSpinBox(bar);
        serialBaud_->setObjectName(QStringLiteral("serialBaud"));
        serialBaud_->setRange(1200, 3000000);
        serialBaud_->setValue(9600);
        serialBaud_->setButtonSymbols(QAbstractSpinBox::NoButtons);
        serialBaud_->setKeyboardTracking(false);
        row2->addWidget(serialBaud_);
    }

    auto *refresh = new QPushButton(QStringLiteral("Refresh"), bar);
    refresh->setObjectName(QStringLiteral("serialRefreshButton"));
    refresh->setProperty("consoleActionRole", "secondary");
    connectButton_ = new QPushButton(QStringLiteral("Connect"), bar);
    connectButton_->setObjectName(QStringLiteral("connectRobotButton"));
    connectButton_->setProperty("consoleActionRole", "primary");
    refresh->setVisible(!remote);
    row2->addWidget(refresh);
    row2->addWidget(connectButton_);

    acquireButton_ = new QPushButton(QStringLiteral("Acquire"), bar);
    releaseButton_ = new QPushButton(QStringLiteral("Release"), bar);
    acquireButton_->setProperty("consoleActionRole", "secondary");
    releaseButton_->setProperty("consoleActionRole", "secondary");
    acquireButton_->setVisible(remote);
    releaseButton_->setVisible(remote);
    row2->addWidget(acquireButton_);
    row2->addWidget(releaseButton_);

    row2->addStretch();
    row2->addWidget(new QLabel(QStringLiteral("State"), bar));
    connectionStatus_ = new QLabel(QStringLiteral("Disconnected"), bar);
    connectionStatus_->setStyleSheet(QStringLiteral("font-weight: 700; color: #263238;"));
    row2->addWidget(connectionStatus_);
    authorityStatus_ = new QLabel(remote ? QStringLiteral("Unowned")
                                          : QStringLiteral("Direct"), bar);
    authorityStatus_->setStyleSheet(
        QStringLiteral("font-weight: 700; color: #455A64;"));
    row2->addWidget(authorityStatus_);
    layout->addLayout(row2);
    layout->addWidget(piHostHint_);

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
        configuration.endpoint = controller_->backendKind() == ConsoleBackendKind::RemoteRbrp
            ? committedPiHost_
            : serialPortCombo_->currentText().trimmed();
        if (controller_->backendKind() == ConsoleBackendKind::RemoteRbrp) {
            configuration.tcpPort = static_cast<quint16>(robotTcpPort_->value());
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
    layout->setContentsMargins(8, 6, 8, 6);
    layout->setSpacing(4);
    auto *header = new QHBoxLayout;
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(6);
    auto *title = new QLabel(QStringLiteral("Realtime Video"), box);
    title->setStyleSheet(QStringLiteral(
        "color: #1F4058; font-size: 13px; font-weight: 700;"));
    header->addWidget(title);

    visionConnectButton_ = new QPushButton(QStringLiteral("Connect Video"), box);
    visionConnectButton_->setObjectName(QStringLiteral("visionConnectButton"));
    visionConnectButton_->setProperty("consoleActionRole", "primary");
    visionRefreshStatusButton_ = new QPushButton(QStringLiteral("Refresh Status"), box);
    visionRefreshStatusButton_->setObjectName(QStringLiteral("visionRefreshStatusButton"));
    visionRefreshStatusButton_->setProperty("consoleActionRole", "secondary");
    visionRefreshStatusButton_->setToolTip(
        QStringLiteral("Refresh HTTP status from the committed endpoint http://%1:%2")
            .arg(committedPiHost_)
            .arg(visionControlClient_ != nullptr
                     ? visionControlClient_->port()
                     : vision::kVisionControlDefaultPort));
    auto *detailsButton = new QPushButton(QStringLiteral("Details"), box);
    detailsButton->setObjectName(QStringLiteral("visionDetailsButton"));
    detailsButton->setProperty("consoleActionRole", "secondary");
    detailsButton->setToolTip(QStringLiteral("Show Vision Details without issuing a request"));
    connect(detailsButton, &QPushButton::clicked, this, [this] {
        if (operatorToolsTabs_ != nullptr) {
            operatorToolsTabs_->setCurrentIndex(2);
        }
    });

    visionState_ = new QLabel(
        visionClient_ != nullptr ? QStringLiteral("Video: Disconnected")
                                 : QStringLiteral("Video: Unavailable"),
        box);
    visionState_->setObjectName(QStringLiteral("visionState"));
    visionState_->setStyleSheet(QStringLiteral("font-weight: 600; color: #566B79;"));
    header->addWidget(visionState_);

    videoFpsSummary_ = new QLabel(QStringLiteral("FPS --"), box);
    videoFpsSummary_->setObjectName(QStringLiteral("videoFpsSummary"));
    videoFpsSummary_->setToolTip(
        QStringLiteral("Received RBVS frame rate; not inference FPS or unique display FPS."));
    videoFpsSummary_->setStyleSheet(QStringLiteral("color: #566B79; font-weight: 600;"));
    header->addWidget(videoFpsSummary_);
    header->addStretch(1);
    header->addWidget(visionConnectButton_);
    header->addWidget(visionRefreshStatusButton_);
    header->addWidget(detailsButton);
    layout->addLayout(header);

    videoView_ = new vision::VideoView(box);
    videoView_->setObjectName(QStringLiteral("videoView"));
    videoView_->setStyleSheet(QStringLiteral(
        "#videoView { background: #101820; border: 1px solid #D5E0E8; "
        "border-radius: 6px; }"));
    layout->addWidget(videoView_, 1);

    const bool inferenceAvailable = visionControlClient_ != nullptr;
    inferenceState_ = new QLabel(
        inferenceAvailable
            ? inferenceStateText(visionControlClient_->status().inferenceState)
            : QStringLiteral("Inference Unavailable"),
        box);
    inferenceState_->setObjectName(QStringLiteral("inferenceState"));
    inferenceState_->setStyleSheet(
        QStringLiteral("font-weight: 600; color: #566B79;"));
    auto *inferenceControls = new QHBoxLayout;
    inferenceControls->setContentsMargins(0, 0, 0, 0);
    inferenceControls->setSpacing(5);
    startInferenceButton_ = new QPushButton(QStringLiteral("Start Inference"), box);
    startInferenceButton_->setObjectName(QStringLiteral("startInferenceButton"));
    startInferenceButton_->setProperty("consoleActionRole", "primary");
    stopInferenceButton_ = new QPushButton(QStringLiteral("Stop Inference"), box);
    stopInferenceButton_->setObjectName(QStringLiteral("stopInferenceButton"));
    stopInferenceButton_->setProperty("consoleActionRole", "stop");
    inferenceControls->addWidget(startInferenceButton_);
    inferenceControls->addWidget(stopInferenceButton_);
    inferenceControls->addWidget(inferenceState_);
    inferencePerformanceSummary_ = new QLabel(QStringLiteral("-- FPS / -- ms"), box);
    inferencePerformanceSummary_->setObjectName(QStringLiteral("inferencePerformanceSummary"));
    inferencePerformanceSummary_->setToolTip(
        QStringLiteral("Camera capture to inference completion; not ORT-only duration."));
    inferenceControls->addWidget(inferencePerformanceSummary_);
    inferenceDetectionSummary_ = new QLabel(QStringLiteral("Detections --"), box);
    inferenceDetectionSummary_->setObjectName(QStringLiteral("inferenceDetectionSummary"));
    inferenceControls->addWidget(inferenceDetectionSummary_);
    inferenceControls->addStretch(1);
    visionControlState_ = new QLabel(QStringLiteral("Unavailable"), box);
    visionControlState_->setObjectName(QStringLiteral("visionControlState"));
    visionControlState_->setStyleSheet(QStringLiteral("font-weight: 600; color: #566B79;"));
    visionControlState_->setText(QStringLiteral("HTTP: Unavailable"));
    inferenceControls->addWidget(visionControlState_);
    layout->addLayout(inferenceControls);
    visionControlMessage_ = new ui::ElidedLabel(box);
    visionControlMessage_->setObjectName(QStringLiteral("visionControlMessage"));
    visionControlMessage_->setFullText(QString());
    visionControlMessage_->setAccessibleName(QStringLiteral("Vision notice"));
    visionControlMessage_->setToolTip(QStringLiteral("Vision control acknowledgements and errors"));
    visionControlMessage_->setStyleSheet(QStringLiteral("font-size: 10px; color: #7B8F9D;"));
    layout->addWidget(visionControlMessage_);

    auto *captureControls = new QHBoxLayout;
    captureControls->setContentsMargins(0, 0, 0, 0);
    captureControls->setSpacing(5);

    snapshotButton_ = new QPushButton(QStringLiteral("Snapshot"), box);
    snapshotButton_->setObjectName(QStringLiteral("snapshotButton"));
    snapshotButton_->setProperty("consoleActionRole", "secondary");
    startRecordingButton_ =
        new QPushButton(QStringLiteral("Start Recording"), box);
    startRecordingButton_->setObjectName(
        QStringLiteral("startRecordingButton"));
    startRecordingButton_->setProperty("consoleActionRole", "primary");
    stopRecordingButton_ =
        new QPushButton(QStringLiteral("Stop Recording"), box);
    stopRecordingButton_->setObjectName(
        QStringLiteral("stopRecordingButton"));
    stopRecordingButton_->setProperty("consoleActionRole", "stop");
    captureState_ = new QLabel(
        visionControlClient_ != nullptr
            ? QStringLiteral("Capture Disconnected")
            : QStringLiteral("Capture Unavailable"),
        box);
    captureState_->setObjectName(QStringLiteral("captureState"));
    captureState_->setStyleSheet(
        QStringLiteral("font-weight: 600; color: #566B79;"));

    captureControls->addWidget(snapshotButton_);
    captureControls->addWidget(startRecordingButton_);
    captureControls->addWidget(stopRecordingButton_);
    captureControls->addStretch();
    captureControls->addWidget(captureState_);
    captureCountSummary_ = new QLabel(QStringLiteral("Recorded -- | Snapshots --"), box);
    captureCountSummary_->setObjectName(QStringLiteral("captureCountSummary"));
    captureControls->addWidget(captureCountSummary_);
    layout->addLayout(captureControls);

    const bool available = visionClient_ != nullptr;
    visionConnectButton_->setEnabled(available);
    visionRefreshStatusButton_->setEnabled(visionControlClient_ != nullptr);
    snapshotButton_->setEnabled(false);
    startRecordingButton_->setEnabled(false);
    stopRecordingButton_->setEnabled(false);
    startInferenceButton_->setEnabled(false);
    stopInferenceButton_->setEnabled(false);

    return box;
}

QWidget *MainWindow::createLeakCard()
{
    auto *box = new QGroupBox(QStringLiteral("Leak Detection"), this);
    box->setObjectName(QStringLiteral("leakCard"));
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(10, 9, 10, 9);
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
    auto *box = new QGroupBox(QStringLiteral("IMU — JY901S"), this);
    box->setObjectName(QStringLiteral("imuCard"));
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(10, 9, 10, 9);
    layout->setSpacing(6);
    addDashboardCardHeader(
        layout, box, QStringLiteral("IMU — JY901S"));

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
    layout->setContentsMargins(10, 9, 10, 9);
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
    auto *box = new QGroupBox(QStringLiteral("Protocol / Link"), this);
    box->setObjectName(QStringLiteral("protocolSummaryCard"));
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(10, 9, 10, 9);
    layout->setSpacing(7);
    addDashboardCardHeader(
        layout, box, QStringLiteral("Protocol / Link"));

    auto *metrics = new QGridLayout;
    metrics->setContentsMargins(1, 0, 0, 0);
    metrics->setHorizontalSpacing(10);
    metrics->setVerticalSpacing(5);
    txCount_ = new QLabel(QStringLiteral("0"), box);
    rxCount_ = new QLabel(QStringLiteral("0"), box);
    crcCount_ = new QLabel(QStringLiteral("0"), box);
    timeoutCount_ = new QLabel(QStringLiteral("0"), box);
    ackRtt_ = new QLabel(QStringLiteral("—"), box);
    for (QLabel *value : {txCount_, rxCount_, crcCount_, timeoutCount_, ackRtt_}) {
        value->setStyleSheet(QStringLiteral("color: #405A6B; font-size: 11px; font-weight: 600;"));
    }
    const QStringList labels = {
        QStringLiteral("TX"), QStringLiteral("RX"), QStringLiteral("CRC"),
        QStringLiteral("Timeout"), QStringLiteral("ACK RTT"),
    };
    for (int index = 0; index < labels.size(); ++index) {
        auto *label = new QLabel(labels.at(index), box);
        label->setStyleSheet(QStringLiteral("color: #7B8F9D; font-size: 11px; font-weight: 500;"));
        const int row = index < 4 ? index / 2 : 2;
        const int column = index < 4 ? (index % 2) * 2 : 0;
        metrics->addWidget(label, row, column);
    }
    metrics->addWidget(txCount_, 0, 1);
    metrics->addWidget(rxCount_, 0, 3);
    metrics->addWidget(crcCount_, 1, 1);
    metrics->addWidget(timeoutCount_, 1, 3);
    metrics->addWidget(ackRtt_, 2, 1);
    metrics->setColumnStretch(1, 1);
    metrics->setColumnStretch(3, 1);
    layout->addLayout(metrics);
    layout->addStretch();
    return box;
}

QWidget *MainWindow::createStatusColumn()
{
    auto *column = new QWidget(this);
    column->setObjectName(QStringLiteral("telemetrySidebar"));
    telemetrySidebar_ = column;
    auto *layout = new QVBoxLayout(column);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);
    QWidget *leak = createLeakCard();
    QWidget *imu = createImuCard();
    QWidget *depth = createDepthCard();
    QWidget *protocol = createProtocolSummaryCard();
    leak->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    imu->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    depth->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    protocol->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    layout->addWidget(leak);
    layout->addWidget(imu);
    layout->addWidget(depth);
    layout->addWidget(protocol);
    layout->addStretch(1);
    return column;
}

QWidget *MainWindow::createDashboard()
{
    auto *dashboard = new QWidget(this);
    dashboard->setObjectName(QStringLiteral("dashboard"));
    auto *layout = new QHBoxLayout(dashboard);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    layout->addWidget(createVideoPlaceholder(), 4);
    layout->addWidget(createStatusColumn(), 1);
    if (auto *sidebar = qobject_cast<QWidget *>(telemetrySidebar_)) {
        sidebar->setMinimumWidth(272);
        sidebar->setMaximumWidth(320);
    }
    return dashboard;
}

QWidget *MainWindow::createServoPanel(int index, ServoId id)
{
    const ServoDescriptor *descriptor = servoDescriptor(id);
    Q_ASSERT(descriptor != nullptr);
    const bool supported = descriptor != nullptr && controller_->isServoSupported(id);
    const QString semanticName = descriptor == nullptr
        ? QStringLiteral("Unknown")
        : QString::fromLatin1(descriptor->displayName);
    auto *box = new QGroupBox(semanticName, this);
    box->setMinimumWidth(248);
    applyCardStyle(box);
    box->setStyleSheet(box->styleSheet() + QStringLiteral(
        "QGroupBox { margin-top: 5px; padding-top: 3px; }"
        "QGroupBox::title { color: transparent; background: transparent; }"));
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(3);

    const bool angleSupported = supported && descriptor != nullptr && descriptor->angleSupported;

    auto *header = new QHBoxLayout;
    header->setSpacing(4);
    auto *name = new QLabel(semanticName, box);
    name->setStyleSheet(QStringLiteral("color: #263238; font-size: 13px; font-weight: 700;"));
    header->addWidget(name);
    header->addStretch();
    auto *statusDot = new QLabel(box);
    statusDot->setObjectName(QStringLiteral("servoStatusDot%1").arg(index));
    statusDot->setFixedSize(8, 8);
    statusDot->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 4px;"));
    header->addWidget(statusDot);
    statusLabels_[index] = new QLabel(QStringLiteral("Disconnected"), box);
    statusLabels_[index]->setStyleSheet(QStringLiteral("color: #6F7F8B; font-size: 11px; font-weight: 600;"));
    header->addWidget(statusLabels_[index]);
    layout->addLayout(header);

    const QString warningText = descriptor == nullptr || !supported
        ? QStringLiteral("UNSUPPORTED — NO HARDWARE CHANNEL")
        : descriptor->calibrationPending
            ? QStringLiteral("PWM bring-up only: %1–%2 μs")
                  .arg(descriptor->commandMinPwmUs)
                  .arg(descriptor->commandMaxPwmUs)
            : QStringLiteral("PWM: %1–%2 μs; angle: %3–%4°")
                  .arg(descriptor->commandMinPwmUs)
                  .arg(descriptor->commandMaxPwmUs)
                  .arg(static_cast<double>(descriptor->commandMinAngleCdeg) / 100.0, 0, 'f', 1)
                  .arg(static_cast<double>(descriptor->commandMaxAngleCdeg) / 100.0, 0, 'f', 1);
    auto *warning = new QLabel(warningText, box);
    warning->setWordWrap(true);
    warning->setStyleSheet(QStringLiteral(
        "color: #756451;"
        "font-size: 10px;"
        "font-weight: 600;"
    ));
    layout->addWidget(warning);

    // PWM label + spin + slider share one compact row.
    auto *pwmRow = new QHBoxLayout;
    pwmRow->setSpacing(2);
    pwmSpins_[index] = new QSpinBox(box);
    pwmSpins_[index]->setRange(descriptor == nullptr ? 0 : descriptor->commandMinPwmUs,
                               descriptor == nullptr ? 0 : descriptor->commandMaxPwmUs);
    pwmSpins_[index]->setValue(descriptor == nullptr ? 0 : descriptor->neutralPwmUs);
    pwmSpins_[index]->setSuffix(QStringLiteral(" μs"));
    pwmSpins_[index]->setEnabled(supported);
    pwmRow->addWidget(new QLabel(QStringLiteral("PWM"), box));
    pwmRow->addWidget(pwmSpins_[index]);
    pwmSliders_[index] = new QSlider(Qt::Horizontal, box);
    pwmSliders_[index]->setRange(descriptor == nullptr ? 0 : descriptor->commandMinPwmUs,
                                 descriptor == nullptr ? 0 : descriptor->commandMaxPwmUs);
    pwmSliders_[index]->setValue(descriptor == nullptr ? 0 : descriptor->neutralPwmUs);
    pwmSliders_[index]->setEnabled(supported);
    pwmRow->addWidget(pwmSliders_[index], 1);
    layout->addLayout(pwmRow);

    // Angle + Set Angle share one compact row.
    auto *angleRow = new QHBoxLayout;
    angleRow->setSpacing(2);
    angleSpins_[index] = new QDoubleSpinBox(box);
    angleSpins_[index]->setRange(descriptor == nullptr ? 0.0
                                                      : static_cast<double>(descriptor->commandMinAngleCdeg) / 100.0,
                                 descriptor == nullptr ? 0.0
                                                      : static_cast<double>(descriptor->commandMaxAngleCdeg) / 100.0);
    angleSpins_[index]->setDecimals(1);
    angleSpins_[index]->setSingleStep(0.1);
    angleSpins_[index]->setValue(0.0);
    angleSpins_[index]->setSuffix(QStringLiteral(" deg"));
    angleSpins_[index]->setEnabled(false);
    angleRow->addWidget(new QLabel(QStringLiteral("Angle"), box));
    angleRow->addWidget(angleSpins_[index], 1);
    angleButtons_[index] = new QPushButton(
        angleSupported
            ? QStringLiteral("Set Angle")
            : QStringLiteral("Set Angle — PWM Only / Planned"),
        box);
    angleButtons_[index]->setEnabled(false);
    angleButtons_[index]->setToolTip(
        angleSupported
            ? QStringLiteral("Send %1 as Protocol V2 centidegrees after Enable ACK")
                  .arg(semanticName)
            : QStringLiteral("%1 has no angle capability; use PWM only while calibration is pending")
                  .arg(semanticName));
    angleRow->addWidget(angleButtons_[index]);
    layout->addLayout(angleRow);

    // Enable / Neutral / Apply on a single row.
    enableButtons_[index] = new QPushButton(QStringLiteral("Enable PWM"), box);
    enableButtons_[index]->setToolTip(QStringLiteral(
        "Enable PWM drive and hold the calibrated neutral position."));
    neutralButtons_[index] = new QPushButton(
        descriptor != nullptr && descriptor->calibrationPending
            ? QStringLiteral("Center %1 us — Provisional").arg(descriptor->neutralPwmUs)
            : QStringLiteral("Neutral"),
        box);
    if (descriptor != nullptr && descriptor->calibrationPending) {
        neutralButtons_[index]->setToolTip(QStringLiteral(
            "Provisional center candidate only; not a calibrated Neutral. "
            "This action reuses the Protocol V2 Neutral command."));
    }
    applyButtons_[index] = new QPushButton(QStringLiteral("Apply PWM"), box);
    if (!controller_->supportsRawPwm()) {
        pwmSpins_[index]->setEnabled(false);
        pwmSliders_[index]->setEnabled(false);
        applyButtons_[index]->setEnabled(false);
        applyButtons_[index]->setToolTip(
            QStringLiteral("Raw PWM is available only in Direct/APC maintenance mode."));
    }

    for (QPushButton *button : {enableButtons_[index], neutralButtons_[index], applyButtons_[index]}) {
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        button->setMinimumWidth(0);
        button->setProperty("consoleActionRole", "secondary");
    }
    auto *actionsRow = new QHBoxLayout;
    actionsRow->setSpacing(2);
    actionsRow->addWidget(enableButtons_[index], 1);
    actionsRow->addWidget(neutralButtons_[index], 1);
    actionsRow->addWidget(applyButtons_[index], 1);
    layout->addLayout(actionsRow);

    connect(pwmSpins_[index], qOverload<int>(&QSpinBox::valueChanged),
            pwmSliders_[index], &QSlider::setValue);
    connect(pwmSliders_[index], &QSlider::valueChanged,
            pwmSpins_[index], &QSpinBox::setValue);
    connect(enableButtons_[index], &QPushButton::clicked, this, [this, id, index] {
        if (controller_->isServoEnabled(id)) {
            controller_->disableServo(id);
        } else {
            controller_->enableServo(id);
        }
    });
    connect(neutralButtons_[index], &QPushButton::clicked, this, [this, id] {
        controller_->neutralServo(id);
    });
    connect(applyButtons_[index], &QPushButton::clicked, this, [this, id, index] {
        controller_->setServoPwm(id, static_cast<quint16>(pwmSpins_[index]->value()));
    });
    connect(angleButtons_[index], &QPushButton::clicked, this, [this, id, index] {
        if (!controller_->isControlActive() || !controller_->isServoSupported(id)
            || !controller_->isServoEnabled(id)
            || controller_->isServoDisablePending(id)
            || servoDescriptor(id) == nullptr
            || !servoDescriptor(id)->angleSupported) {
            refreshServoUi(index);
            return;
        }
        controller_->setServoAngle(id, angleDegreesToCentidegrees(angleSpins_[index]->value()));
    });
    refreshServoUi(index);
    return box;
}

QWidget *MainWindow::createActuatorPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Actuator Control"), this);
    box->setObjectName(QStringLiteral("actuatorPage"));
    applySectionStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(10, 8, 10, 10);
    layout->setSpacing(7);

    auto *header = new QHBoxLayout;
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(7);

    auto *accent = new QWidget(box);
    accent->setFixedSize(4, 18);
    accent->setStyleSheet(QStringLiteral(
        "background: #1976D2;"
        "border-radius: 2px;"
    ));
    header->addWidget(accent);

    auto *title = new QLabel(QStringLiteral("Actuator Control"), box);
    title->setStyleSheet(QStringLiteral(
        "color: #18364F;"
        "font-size: 14px;"
        "font-weight: 700;"
    ));
    header->addWidget(title);

    header->addStretch();
    layout->addLayout(header);

    actuatorCardsHost_ = new QWidget(box);
    actuatorCardsHost_->setObjectName(QStringLiteral("actuatorCardsHost"));
    actuatorCardsHost_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    actuatorCardsHost_->setMinimumWidth(0);
    actuatorCardsHost_->installEventFilter(this);
    actuatorGrid_ = new QGridLayout(actuatorCardsHost_);
    actuatorGrid_->setContentsMargins(0, 0, 0, 0);
    actuatorGrid_->setHorizontalSpacing(8);
    actuatorGrid_->setVerticalSpacing(8);
    const auto &descriptors = servoDescriptorTable();
    for (int index = 0; index < kServoCount; ++index) {
        servoPanels_[static_cast<std::size_t>(index)] =
            createServoPanel(index, descriptors.at(index).id);
    }
    layout->addWidget(actuatorCardsHost_, 1);
    reflowActuatorCards();
    return box;
}

QWidget *MainWindow::createMotionPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Motion / Gait — Bench"), this);
    box->setObjectName(QStringLiteral("motionPage"));
    applySectionStyle(box);
    auto *outer = new QVBoxLayout(box);
    outer->setContentsMargins(10, 8, 10, 10);
    outer->setSpacing(7);

    auto *sectionHeader = new QHBoxLayout;
    sectionHeader->setContentsMargins(0, 0, 0, 0);
    sectionHeader->setSpacing(7);

    auto *accent = new QWidget(box);
    accent->setFixedSize(4, 18);
    accent->setStyleSheet(QStringLiteral(
        "background: #1976D2;"
        "border-radius: 2px;"
    ));
    sectionHeader->addWidget(accent);

    auto *sectionTitle = new QLabel(QStringLiteral("Motion / Gait — Bench"), box);
    sectionTitle->setStyleSheet(QStringLiteral(
        "color: #18364F;"
        "font-size: 14px;"
        "font-weight: 700;"
    ));
    sectionHeader->addWidget(sectionTitle);
    sectionHeader->addStretch();
    outer->addLayout(sectionHeader);

    auto *content = new QHBoxLayout;
    content->setContentsMargins(0, 0, 0, 0);
    content->setSpacing(8);

    // --- Motion Control (left): D-pad-like arrangement ---
    auto *motionGroup = new QGroupBox(QStringLiteral("Motion Control"), box);
    applySubpanelStyle(motionGroup);
    auto *dpad = new QGridLayout(motionGroup);
    dpad->setContentsMargins(8, 12, 8, 8);
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
        motionButtons_[static_cast<std::size_t>(mode)] = button;
        if (mode == MotionMode::Backward) {
            button->setEnabled(false);
            button->setToolTip(
                QStringLiteral("Pending water-tank verification; no BACKWARD START is emitted."));
        }
        connect(button,
                &QPushButton::clicked,
                this,
                [this, mode] {
                    controller_->startMotion(mode);
                    refreshMotionUi();
                });
    }
    dpad->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::Forward)], 0, 1);
    dpad->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::TurnLeft)], 1, 0);
    auto *manual = new QLabel(QStringLiteral("Manual"), motionGroup);
    manual->setObjectName(QStringLiteral("motionManualCenter"));
    manual->setAlignment(Qt::AlignCenter);
    manual->setStyleSheet(QStringLiteral("color: #667C8C; font-size: 11px; font-weight: 600;"));
    dpad->addWidget(manual, 1, 1);
    dpad->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::TurnRight)], 1, 2);
    dpad->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::Backward)], 2, 1);

    // --- Gait / Vertical (right) ---
    auto *gaitGroup = new QGroupBox(QStringLiteral("Gait / Vertical"), box);
    applySubpanelStyle(gaitGroup);
    auto *gaitLayout = new QGridLayout(gaitGroup);
    gaitLayout->setContentsMargins(8, 12, 8, 8);
    gaitLayout->setSpacing(6);

    const MotionMode verticalModes[] = {
        MotionMode::Ascend,
        MotionMode::Descend,
    };
    for (const MotionMode mode : verticalModes) {
        auto *button = new QPushButton(motionModeText(mode), gaitGroup);
        button->setCheckable(true);
        button->setAutoExclusive(false);
        button->setProperty("consoleActionRole", "secondary");
        motionButtons_[static_cast<std::size_t>(mode)] = button;
        connect(button,
                &QPushButton::clicked,
                this,
                [this, mode] {
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
    gaitBackendStatus_ = new QLabel(QStringLiteral("Unknown"), gaitGroup);
    auto *provisional = new QLabel(
        QStringLiteral("Bench Provisional / Pending Water Verification"),
        gaitGroup);
    provisional->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));

    gaitLayout->addWidget(new QLabel(QStringLiteral("Gait Backend"), gaitGroup), 0, 0, 1, 3);
    gaitLayout->addWidget(gaitBackendCombo_, 1, 0, 1, 3);
    gaitLayout->addWidget(new QLabel(QStringLiteral("Current:"), gaitGroup), 2, 0);
    gaitLayout->addWidget(gaitBackendStatus_, 2, 1, 1, 2);
    auto *motionStatusHint = new QLabel(QStringLiteral("Motion status is shown in Operator tools"), gaitGroup);
    motionStatusHint->setStyleSheet(QStringLiteral("color: #667C8C; font-size: 11px;"));
    gaitLayout->addWidget(motionStatusHint, 3, 0, 1, 3);
    gaitLayout->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::Ascend)], 4, 0);
    gaitLayout->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::Descend)], 4, 1);
    gaitLayout->addWidget(provisional, 5, 0, 1, 3);

    content->addWidget(motionGroup, 1);
    content->addWidget(gaitGroup, 1);
    outer->addLayout(content, 1);

    connect(gaitBackendCombo_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int index) {
                if (index < 0 || gaitBackendCombo_ == nullptr
                    || !gaitBackendCombo_->isEnabled()) {
                    refreshGaitBackendUi();
                    return;
                }
                const QVariant value = gaitBackendCombo_->itemData(index);
                if (!value.isValid()
                    || !isValidGaitBackend(static_cast<quint8>(value.toInt()))) {
                    refreshGaitBackendUi();
                    return;
                }
                controller_->setGaitBackend(
                    static_cast<GaitBackend>(value.toInt()));
                refreshGaitBackendUi();
            });

    refreshMotionUi();
    refreshGaitBackendUi();
    return box;
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
    layout->addLayout(videoForm);

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
    title->setStyleSheet(QStringLiteral("color: #1F4058; font-size: 15px; font-weight: 700;"));
    layout->addWidget(title);
    layout->addSpacing(12);
    motionStatus_ = new QLabel(QStringLiteral("Stopped"), bar);
    motionStatus_->setObjectName(QStringLiteral("motionStatus"));
    motionStatus_->setMinimumWidth(110);
    motionStatus_->setStyleSheet(QStringLiteral("color: #667C8C; font-weight: 700;"));
    layout->addWidget(motionStatus_);
    layout->addStretch(1);

    motionStopButton_ = new QPushButton(QStringLiteral("Motion Stop"), bar);
    motionStopButton_->setObjectName(QStringLiteral("motionStopButton"));
    motionStopButton_->setProperty("consoleActionRole", "stop");
    layout->addWidget(motionStopButton_);
    connect(motionStopButton_, &QPushButton::clicked, this, [this] {
        controller_->stopMotion();
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
    });

    emergencyStopButton_ = new QPushButton(QStringLiteral("Emergency Stop"), bar);
    emergencyStopButton_->setObjectName(QStringLiteral("emergencyStopButton"));
    emergencyStopButton_->setEnabled(false);
    emergencyStopButton_->setProperty("consoleActionRole", "danger");
    emergencyStopButton_->setToolTip(QStringLiteral("Protocol V2 has no Emergency Stop message in Phase 1"));
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
    operatorToolsTabs_->addTab(motionScrollArea_, QStringLiteral("Motion / Gait"));

    actuatorScroll_ = new QScrollArea(this);
    actuatorScroll_->setObjectName(QStringLiteral("actuatorScrollArea"));
    actuatorScroll_->setWidgetResizable(true);
    actuatorScroll_->setFrameShape(QFrame::NoFrame);
    actuatorScroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    actuatorScroll_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    actuatorScroll_->installEventFilter(this);
    actuatorScroll_->viewport()->installEventFilter(this);
    actuatorScroll_->setWidget(createActuatorPanel());
    operatorToolsTabs_->addTab(actuatorScroll_, QStringLiteral("Actuators"));
    operatorToolsTabs_->addTab(createVisionDetailsTab(), QStringLiteral("Vision Details"));
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
        wrapDetailsPage(createTelemetryDetailsTab(), QStringLiteral("telemetryDetailsScroll")),
        QStringLiteral("Telemetry Details"));
    operatorToolsTabs_->addTab(
        wrapDetailsPage(createProtocolDetailsTab(), QStringLiteral("protocolDetailsScroll")),
        QStringLiteral("Protocol Details"));
    operatorToolsTabs_->addTab(createLogTab(), QStringLiteral("Log"));
    operatorToolsTabs_->addTab(
        wrapDetailsPage(createDataPlotsTab(), QStringLiteral("dataPlotsScroll")),
        QStringLiteral("Data Plots"));
    connect(operatorToolsTabs_, &QTabWidget::currentChanged, this, [this] {
        if (actuatorGrid_ != nullptr) {
            reflowActuatorCards();
        }
    });
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

void MainWindow::bindVisionUi()
{
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
            visionState_->setText(QStringLiteral("Video: %1").arg(text));
            visionConnectButton_->setText(buttonText);
            const bool videoConnected =
                state == vision::VisionConnectionState::Connected;
            if (!videoConnected) {
                haveVisionReceiveSample_ = false;
                videoView_->clearFrame();
            }
            refreshVideoDiagnosticsUi();
            refreshPiHostUi();
        });

        connect(visionClient_, &vision::VisionClient::frameReady,
                this, [this](const QImage &image, quint64 frameId, quint64) {
            haveVisionReceiveSample_ = true;
            videoView_->setFrame(image, frameId);
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
        connect(startRecordingButton_, &QPushButton::clicked,
                visionControlClient_, &vision::VisionControlClient::startRecording);
        connect(stopRecordingButton_, &QPushButton::clicked,
                visionControlClient_, &vision::VisionControlClient::stopRecording);
        connect(startInferenceButton_, &QPushButton::clicked,
                visionControlClient_, &vision::VisionControlClient::startInference);
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
                this, &MainWindow::refreshVisionUi);
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
    refreshVisionUi();
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
        refreshGaitBackendUi();
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
        for (int index = 0; index < kServoCount; ++index) {
            refreshServoUi(index);
        }
    });
    connect(controller_, &IConsoleController::gaitBackendStateChanged,
            this, &MainWindow::refreshGaitBackendUi);
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
            QStringLiteral("FPS %1")
                .arg(live ? QString::number(visionClient_->receivedFps(), 'f', 1)
                          : QStringLiteral("--")));
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

void MainWindow::reflowActuatorCards()
{
    if (actuatorGrid_ == nullptr || actuatorCardsHost_ == nullptr) {
        return;
    }
    const int width = actuatorCardsHost_->width();
    const int columns = width >= 5 * 248 + 4 * 8 ? 5
        : width >= 3 * 248 + 2 * 8 ? 3
        : width >= 2 * 248 + 8 ? 2 : 1;
    int actualColumns = 0;
    for (int index = 0; index < actuatorGrid_->count(); ++index) {
        int row = 0;
        int column = 0;
        int rowSpan = 0;
        int columnSpan = 0;
        actuatorGrid_->getItemPosition(index, &row, &column, &rowSpan, &columnSpan);
        actualColumns = qMax(actualColumns, column + columnSpan);
    }
    if (columns == actuatorColumnCount_ && actualColumns == columns) {
        return;
    }
    while (QLayoutItem *item = actuatorGrid_->takeAt(0)) {
        delete item;
    }
    for (int index = 0; index < kServoCount; ++index) {
        actuatorGrid_->addWidget(servoPanels_[static_cast<std::size_t>(index)],
                                 index / columns, index % columns);
    }
    actuatorColumnCount_ = columns;
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
    const int toolsHeight = availableHeight > 760 ? 240 : 190;
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
    refreshGaitBackendUi();
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
    enableButtons_[index]->setText(enabled
                                       ? QStringLiteral("Release PWM")
                                       : QStringLiteral("Enable PWM"));
    enableButtons_[index]->setToolTip(enabled
                                          ? QStringLiteral(
                                                "Release PWM: stop PWM drive without sending Neutral; the servo is no longer actively held.")
                                          : QStringLiteral(
                                                "Enable PWM drive and hold the calibrated neutral position."));
    enableButtons_[index]->setEnabled(active && supported && !motionActive);
    neutralButtons_[index]->setEnabled(active && supported && enabled
                                       && !pendingDisable && !motionActive);
    applyButtons_[index]->setEnabled(
        active && controller_->supportsRawPwm() && supported && enabled
        && !pendingDisable && !motionActive);
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
        statusColor = QStringLiteral("#43a047");
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
    angleSpins_[index]->setEnabled(actionable);
    angleButtons_[index]->setEnabled(actionable);
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

void MainWindow::refreshGaitBackendUi()
{
    if (gaitBackendCombo_ == nullptr || gaitBackendStatus_ == nullptr) {
        return;
    }

    const bool connected = controller_->isControlActive();
    const bool pending = controller_->isGaitBackendChangePending();
    const std::optional<GaitBackend> displayBackend = pending
        ? controller_->requestedGaitBackend()
        : controller_->confirmedGaitBackend();
    int displayIndex = gaitBackendCombo_->findData(-1);
    QString status = QStringLiteral("Unknown");
    if (displayBackend.has_value()) {
        displayIndex = gaitBackendCombo_->findData(
            static_cast<int>(*displayBackend));
        const QString name = gaitBackendCombo_->itemText(displayIndex);
        status = pending
            ? QStringLiteral("Requested — %1 (awaiting ACK)").arg(name)
            : QStringLiteral("Confirmed — %1").arg(name);
    }

    const QSignalBlocker blocker(gaitBackendCombo_);
    gaitBackendCombo_->setCurrentIndex(displayIndex);
    gaitBackendCombo_->setEnabled(connected && !pending);
    gaitBackendStatus_->setText(connected ? status : QStringLiteral("Unknown"));
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
    applyPiHostButton_->setEnabled(dirty && hostCandidateValid(piHost_->text()) && safe);
    piHostHint_->setVisible(dirty);
    piHostHint_->setText(
        QStringLiteral("Active Pi: %1 | Host edit not applied").arg(committedPiHost_));
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
    refreshPiHostUi();
}

void MainWindow::refreshVisionUi()
{
    refreshInferenceUi();
    refreshCaptureUi();
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
    startInferenceButton_->setText(ui.startText);
    stopInferenceButton_->setText(ui.stopText);
    startInferenceButton_->setEnabled(ui.startEnabled && !hostDirty());
    stopInferenceButton_->setEnabled(ui.stopEnabled);
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
    snapshotButton_->setEnabled(usable && !busy && !hostDirty());
    startRecordingButton_->setEnabled(
        usable && !busy && !status.recording
        && status.state != QStringLiteral("stopping") && !hostDirty());
    stopRecordingButton_->setEnabled(
        fresh && status.haveCaptureStatus && status.recording && !busy);
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
    if ((watched == actuatorCardsHost_ || watched == actuatorScroll_
         || (actuatorScroll_ != nullptr && watched == actuatorScroll_->viewport()))
        && event->type() == QEvent::Resize) {
        if (!actuatorReflowPending_) {
            actuatorReflowPending_ = true;
            QTimer::singleShot(0, this, [this] {
                actuatorReflowPending_ = false;
                reflowActuatorCards();
            });
        }
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
    if (visionControlClient_ != nullptr) {
        visionControlClient_->shutdown();
    }
    if (visionClient_ != nullptr) {
        visionClient_->shutdown();
    }
    controller_->shutdown();
    event->accept();
}

} // namespace rb
