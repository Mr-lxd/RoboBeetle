#include "ui/MainWindow.h"

#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

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
        "  background: #EEF4F8;"
        "  border: 1px solid #B8C9D8;"
        "  border-radius: 9px;"
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
        "  background: #FAFCFE;"
        "  border: 1px solid #D6E0E8;"
        "  border-radius: 7px;"
        "  margin-top: 14px;"
        "  padding-top: 6px;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  left: 9px;"
        "  top: 1px;"
        "  padding: 0 5px;"
        "  background: #FAFCFE;"
        "  color: #49657A;"
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

MainWindow::MainWindow(RobotController *controller, QWidget *parent)
    : QMainWindow(parent), controller_(controller)
{
    Q_ASSERT(controller_ != nullptr);
    setWindowTitle(QStringLiteral("RoboBeetle Console"));
    resize(1420, 880);
    setMinimumSize(1100, 720);

    auto *central = new QWidget(this);
    central->setObjectName(QStringLiteral("appCanvas"));
    central->setStyleSheet(
        QStringLiteral("#appCanvas { background: #E2EAF1; }"));
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    root->addWidget(createConnectionBar());
    root->addWidget(createDashboard(), 1);
    root->addWidget(createActuatorPanel(), 1);
    root->addWidget(createLowerDashboard(), 1);

    setCentralWidget(central);

    // Application-wide styling system (Slice A): a subtle page background tint,
    // a shared base font, and product-style tabs/inputs. Cards and section
    // titles are styled locally via applyCardStyle(). Per-widget stylesheets
    // (motion buttons, header actions) override these app-wide defaults.
    setStyleSheet(QStringLiteral(
        "QMainWindow { background: #E2EAF1; }"
        "QWidget { font-family: \"Segoe UI\"; font-size: 12px; color: #334A5C; }"
        "QTabWidget::pane { border: 1px solid #cfd6dd; border-radius: 6px; "
        "  background: #ffffff; top: -1px; }"
        "QTabBar::tab { background: #eef2f6; border: 1px solid #d5dce3; "
        "  border-bottom: none; border-top-left-radius: 5px; "
        "  border-top-right-radius: 5px; padding: 5px 14px; margin-right: 2px; "
        "  color: #455a64; }"
        "QTabBar::tab:selected { background: #ffffff; color: #1565c0; "
        "  font-weight: bold; border-top: 2px solid #1976D2; }"
        "QTabBar::tab:hover:!selected { background: #e2e8ee; }"
        "QPushButton { background: #f2f5f8; border: 1px solid #c4ccd4; "
        "  border-radius: 5px; padding: 6px 14px; color: #37474f; }"
        "QPushButton:hover { background: #e3e9ef; border-color: #aeb7c0; }"
        "QPushButton:pressed { background: #d7dfe7; }"
        "QPushButton:disabled { color: #7F8F9C; background: #F1F4F6; "
        "  border-color: #CDD7E0; }"
        "QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit { background: #ffffff; "
        "  border: 1px solid #c4ccd4; border-radius: 4px; padding: 3px 6px; "
        "  color: #37474f; }"
        "QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus { "
        "  border-color: #1976D2; }"
        "QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, "
        "QLineEdit:disabled { color: #7F8F9C; background: #F1F4F6; "
        "  border-color: #CDD7E0; }"
        "QSlider::groove:horizontal { height: 6px; background: #d3dae1; "
        "  border-radius: 3px; }"
        "QSlider::handle:horizontal { width: 14px; margin: -4px 0; "
        "  background: #1976D2; border-radius: 7px; }"
        "QSlider::handle:horizontal:disabled { background: #b8c1c9; }"
        "QPlainTextEdit { background: #fbfcfd; border: 1px solid #c4ccd4; "
        "  border-radius: 4px; color: #37474f; }"));

    connect(controller_, &RobotController::serialPortsChanged, this, [this](const QStringList &ports) {
        const QString current = portCombo_->currentText();
        portCombo_->clear();
        portCombo_->addItems(ports);
        if (!current.isEmpty() && portCombo_->findText(current) < 0) {
            portCombo_->addItem(current);
        }
        portCombo_->setCurrentText(current);
    });
    connect(controller_, &RobotController::connectionStateChanged, this, [this](TransportState state) {
        connectionStatus_->setText(stateText(state));
        setConnectedUi(state == TransportState::Connected);
    });
    connect(controller_, &RobotController::servoStateChanged, this, [this](int index, bool enabled) {
        if (index < 0 || index >= kServoCount) {
            return;
        }
        Q_UNUSED(enabled);
        refreshServoUi(index);
        refreshMotionUi();
    });
    connect(controller_, &RobotController::servoDisablePendingChanged,
            this, [this](int index, bool) {
        if (index < 0 || index >= kServoCount) {
            return;
        }
        refreshServoUi(index);
        refreshMotionUi();
    });
    connect(controller_, &RobotController::motionStateChanged,
            this, [this](MotionState, MotionMode) {
                refreshMotionUi();
                for (int index = 0; index < kServoCount; ++index) {
                    refreshServoUi(index);
                }
            });
    connect(controller_, &RobotController::gaitBackendStateChanged,
            this, &MainWindow::refreshGaitBackendUi);
    connect(controller_, &RobotController::leakStateChanged,
            this, &MainWindow::setLeakUiState);
    connect(controller_->imuMonitor(), &ImuMonitor::changed, this, [this] {
        setImuUiState(controller_->imuState());
    });
    connect(controller_->depthMonitor(), &DepthMonitor::changed, this, [this] {
        setDepthUiState(controller_->depthState());
    });
    connect(controller_, &RobotController::txHexChanged, txHex_, &QLineEdit::setText);
    connect(controller_, &RobotController::rxHexChanged, rxHex_, &QLineEdit::setText);
    connect(controller_, &RobotController::protocolMonitorChanged, this, [this](const ProtocolMonitor &monitor) {
        txCount_->setText(QString::number(monitor.txPacketCount));
        rxCount_->setText(QString::number(monitor.rxPacketCount));
        crcCount_->setText(QString::number(monitor.crcErrorCount));
        timeoutCount_->setText(QString::number(monitor.timeoutCount));
        ackRtt_->setText(monitor.lastAckRttMs < 0
                             ? QStringLiteral("—")
                             : QStringLiteral("%1 ms").arg(monitor.lastAckRttMs));
        ackStatus_->setText(monitor.ackStatus);
    });
    connect(controller_, &RobotController::logMessage, this, &MainWindow::appendLog);

    setConnectedUi(false);
    setLeakUiState(controller_->leakState());
    setImuUiState(controller_->imuState());
    setDepthUiState(controller_->depthState());
    refreshMotionUi();
    refreshGaitBackendUi();
    controller_->refreshSerialPorts();
}

QWidget *MainWindow::createConnectionBar()
{
    auto *bar = new QWidget(this);
    bar->setFixedHeight(48);
    bar->setObjectName(QStringLiteral("topHeaderBar"));
    bar->setStyleSheet(QStringLiteral(
        "#topHeaderBar {"
        "  background: #FFFFFF;"
        "  border: 1px solid #C5D3DE;"
        "  border-radius: 7px;"
        "}"
    ));
    auto *layout = new QHBoxLayout(bar);
    layout->setContentsMargins(10, 4, 10, 4);
    layout->setSpacing(6);

    auto *title = new QLabel(QStringLiteral("RoboBeetle Console"), bar);
    title->setStyleSheet(QStringLiteral(
        "font-weight: 700;"
        "font-size: 15px;"
        "color: #145DA0;"
        "letter-spacing: 0.3px;"
    ));
    layout->addWidget(title);

    layout->addSpacing(6);

    layout->addWidget(new QLabel(QStringLiteral("Serial Port"), bar));
    portCombo_ = new QComboBox(bar);
    portCombo_->setEditable(true);
    layout->addWidget(portCombo_);

    layout->addWidget(new QLabel(QStringLiteral("Baud Rate"), bar));
    baudSpin_ = new QSpinBox(bar);
    baudSpin_->setRange(1200, 3000000);
    baudSpin_->setValue(9600);
    layout->addWidget(baudSpin_);

    // Secondary action: Refresh.
    auto *refresh = new QPushButton(QStringLiteral("Refresh"), bar);
    refresh->setStyleSheet(QStringLiteral(
        "QPushButton { background: #ffffff; border: 1px solid #90a4ae; "
        "  border-radius: 5px; padding: 6px 14px; color: #455a64; }"
        "QPushButton:hover { background: #eef2f6; border-color: #78909c; }"
        "QPushButton:pressed { background: #e0e6ec; }"));

    // Primary action: Connect.
    connectButton_ = new QPushButton(QStringLiteral("Connect"), bar);
    connectButton_->setStyleSheet(QStringLiteral(
        "QPushButton { background: #1976D2; border: 1px solid #1565c0; "
        "  border-radius: 5px; padding: 6px 16px; color: #ffffff; "
        "  font-weight: 700; }"
        "QPushButton:hover { background: #1565c0; }"
        "QPushButton:pressed { background: #11519b; }"
        "QPushButton:disabled { background: #9eb6cf; color: #e6edf3; "
        "  border-color: #8aa6c0; }"));
    layout->addWidget(refresh);
    layout->addWidget(connectButton_);

    // Danger action: Emergency Stop.
    auto *emergencyStop = new QPushButton(QStringLiteral("Emergency Stop"), bar);
    emergencyStop->setEnabled(false);
    emergencyStop->setToolTip(QStringLiteral("Protocol V2 has no Emergency Stop message in Phase 1"));
    emergencyStop->setStyleSheet(QStringLiteral(
        "QPushButton { background: #E53935; border: 1px solid #c62828; "
        "  border-radius: 5px; padding: 6px 14px; color: #ffffff; "
        "  font-weight: 700; }"
        "QPushButton:hover { background: #d32f2f; }"
        "QPushButton:pressed { background: #b71c1c; }"
        "QPushButton:disabled { background: #f2c2c1; color: #fdeaea; "
        "  border-color: #e5a5a3; }"));
    layout->addWidget(emergencyStop);

    layout->addStretch();
    layout->addWidget(new QLabel(QStringLiteral("State"), bar));
    connectionStatus_ = new QLabel(QStringLiteral("Disconnected"), bar);
    connectionStatus_->setStyleSheet(QStringLiteral("font-weight: 700; color: #263238;"));
    layout->addWidget(connectionStatus_);

    connect(refresh, &QPushButton::clicked, controller_, &RobotController::refreshSerialPorts);
    connect(connectButton_, &QPushButton::clicked, this, [this] {
        if (controller_->isConnected()) {
            controller_->disconnectTransport();
            return;
        }
        controller_->connectTransport({portCombo_->currentText().trimmed(), baudSpin_->value()});
    });
    return bar;
}

QWidget *MainWindow::createVideoPlaceholder()
{
    auto *box = new QGroupBox(QStringLiteral("Realtime Video"), this);
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(10, 9, 10, 10);
    layout->setSpacing(8);
    addDashboardCardHeader(
        layout, box, QStringLiteral("Realtime Video"));

    auto *stage = new QWidget(box);
    stage->setObjectName(QStringLiteral("videoEmptyStage"));
    stage->setStyleSheet(QStringLiteral(
        "#videoEmptyStage {"
        "  background: #F7FAFC;"
        "  border: 1px solid #E1E9F0;"
        "  border-radius: 6px;"
        "}"
    ));

    auto *stageLayout = new QVBoxLayout(stage);
    stageLayout->setContentsMargins(16, 16, 16, 16);
    stageLayout->setSpacing(5);
    auto *title = new QLabel(QStringLiteral("Waiting for video stream"), stage);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet(QStringLiteral(
        "font-size: 15px;"
        "font-weight: 700;"
        "color: #294B63;"
    ));
    auto *subtitle = new QLabel(QStringLiteral("Video backend not connected"), stage);
    subtitle->setAlignment(Qt::AlignCenter);
    subtitle->setStyleSheet(QStringLiteral(
        "font-size: 11px;"
        "color: #7B8F9D;"
    ));
    stageLayout->addStretch();
    stageLayout->addWidget(title);
    stageLayout->addWidget(subtitle);
    stageLayout->addStretch();
    layout->addWidget(stage, 1);
    return box;
}

QWidget *MainWindow::createLeakCard()
{
    auto *box = new QGroupBox(QStringLiteral("Leak Detection"), this);
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
    auto *box = new QGroupBox(QStringLiteral("Depth Sensor — ROVMAKER"), this);
    applyDashboardCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(10, 9, 10, 9);
    layout->setSpacing(6);
    addDashboardCardHeader(
        layout, box, QStringLiteral("Depth Sensor — ROVMAKER"));

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
    auto *layout = new QGridLayout(column);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);
    // Keep the dashboard telemetry as one balanced 2x2 grid alongside video.
    QWidget *leak = createLeakCard();
    QWidget *imu = createImuCard();
    QWidget *depth = createDepthCard();
    QWidget *protocol = createProtocolSummaryCard();
    leak->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    imu->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    depth->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    protocol->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    layout->addWidget(leak, 0, 0);
    layout->addWidget(imu, 0, 1);
    layout->addWidget(depth, 1, 0);
    layout->addWidget(protocol, 1, 1);
    layout->setColumnStretch(0, 1);
    layout->setColumnStretch(1, 1);
    layout->setRowStretch(0, 1);
    layout->setRowStretch(1, 1);
    return column;
}

QWidget *MainWindow::createDashboard()
{
    auto *dashboard = new QWidget(this);
    auto *layout = new QHBoxLayout(dashboard);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    layout->addWidget(createVideoPlaceholder(), 3);
    layout->addWidget(createStatusColumn(), 2);
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
    statusLabels_[index]->setStyleSheet(QStringLiteral("color: #666666; font-size: 11px; font-weight: 600;"));
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
    warning->setStyleSheet(QStringLiteral("color: #8b6f4e; font-size: 10px; font-weight: 500;"));
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

    for (QPushButton *button : {enableButtons_[index], neutralButtons_[index], applyButtons_[index]}) {
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        button->setMinimumWidth(0);
        button->setStyleSheet(QStringLiteral("QPushButton { padding: 4px 3px; }"));
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
        if (!controller_->isConnected() || !controller_->isServoSupported(id)
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
    auto *disableAll = new QPushButton(QStringLiteral("Disable All"), box);
    disableAll->setStyleSheet(QStringLiteral(
        "QPushButton { background: #E53935; border: 1px solid #c62828; "
        "  border-radius: 5px; padding: 6px 16px; color: #ffffff; "
        "  font-weight: 700; }"
        "QPushButton:hover { background: #d32f2f; }"
        "QPushButton:pressed { background: #b71c1c; }"
        "QPushButton:disabled { background: #f2c2c1; color: #fdeaea; "
        "  border-color: #e5a5a3; }"));
    header->addWidget(disableAll);
    layout->addLayout(header);

    auto *cards = new QHBoxLayout;
    cards->setSpacing(6);
    const auto &descriptors = servoDescriptorTable();
    for (int index = 0; index < kServoCount; ++index) {
        cards->addWidget(createServoPanel(index, descriptors.at(index).id), 1);
    }
    layout->addLayout(cards);

    connect(disableAll, &QPushButton::clicked, this, [this] {
        controller_->disableAll();
        for (int index = 0; index < kServoCount; ++index) {
            refreshServoUi(index);
        }
    });
    return box;
}

QWidget *MainWindow::createMotionPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Motion / Gait — Bench"), this);
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
        button->setStyleSheet(QStringLiteral(
            "QPushButton { background: #ffffff; border: 1px solid #b0bbc4; "
            "  border-radius: 6px; padding: 8px 10px; color: #37474f; "
            "  font-weight: 600; }"
            "QPushButton:hover { background: #eef2f6; border-color: #90a4ae; }"
            "QPushButton:pressed { background: #e0e6ec; }"
            "QPushButton:disabled { color: #7F8F9C; background: #F1F4F6; "
            "  border-color: #C9D4DD; }"
            "QPushButton:checked { background-color: #1976D2; color: white; "
            "  font-weight: bold; border-color: #1565c0; }"));
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
    motionStopButton_ = new QPushButton(QStringLiteral("Stop"), motionGroup);
    motionStopButton_->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background: #455A64;"
        "  border: 1px solid #37474F;"
        "  border-radius: 6px;"
        "  padding: 8px 14px;"
        "  color: #FFFFFF;"
        "  font-weight: 700;"
        "  font-size: 13px;"
        "}"
        "QPushButton:hover { background: #37474F; }"
        "QPushButton:pressed { background: #263238; }"
        "QPushButton:disabled {"
        "  background: #D5DDE2;"
        "  color: #71818D;"
        "  border-color: #C1CBD2;"
        "}"
    ));
    connect(motionStopButton_, &QPushButton::clicked, this, [this] {
        controller_->stopMotion();
        refreshMotionUi();
    });

    dpad->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::Forward)], 0, 1);
    dpad->addWidget(motionButtons_[static_cast<std::size_t>(MotionMode::TurnLeft)], 1, 0);
    dpad->addWidget(motionStopButton_, 1, 1);
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
        button->setStyleSheet(QStringLiteral(
            "QPushButton { background: #ffffff; border: 1px solid #b0bbc4; "
            "  border-radius: 6px; padding: 8px 10px; color: #37474f; "
            "  font-weight: 600; }"
            "QPushButton:hover { background: #eef2f6; border-color: #90a4ae; }"
            "QPushButton:pressed { background: #e0e6ec; }"
            "QPushButton:checked { background-color: #1976D2; color: white; "
            "  font-weight: bold; border-color: #1565c0; }"));
        motionButtons_[static_cast<std::size_t>(mode)] = button;
        connect(button,
                &QPushButton::clicked,
                this,
                [this, mode] {
                    controller_->startMotion(mode);
                    refreshMotionUi();
                });
    }

    motionStatus_ = new QLabel(QStringLiteral("Stopped"), gaitGroup);
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
    gaitLayout->addWidget(new QLabel(QStringLiteral("Motion Status"), gaitGroup), 3, 0);
    gaitLayout->addWidget(motionStatus_, 3, 1, 1, 2);
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
    applySectionStyle(section);

    auto *outer = new QVBoxLayout(section);
    outer->setContentsMargins(10, 8, 10, 10);
    outer->setSpacing(7);
    addSectionHeader(
        outer, section, QStringLiteral("Data Plots"));

    auto *tabs = new QTabWidget(section);
    const QString titles[] = {
        QStringLiteral("IMU"),
        QStringLiteral("Depth"),
        QStringLiteral("Actuator"),
    };
    for (const QString &title : titles) {
        auto *page = new QWidget(tabs);
        page->setStyleSheet(QStringLiteral(
            "background: #FFFFFF;"
        ));

        auto *pageLayout = new QVBoxLayout(page);
        pageLayout->setContentsMargins(10, 10, 10, 10);
        pageLayout->setSpacing(8);

        auto *pageTitle = new QLabel(QStringLiteral("%1 Plots").arg(title), page);
        pageTitle->setStyleSheet(QStringLiteral(
            "color: #49657A;"
            "font-size: 12px;"
            "font-weight: 600;"
        ));
        pageLayout->addWidget(pageTitle);

        auto *viewport = new QWidget(page);
        viewport->setStyleSheet(QStringLiteral(
            "background: #F8FAFC;"
            "border: 1px solid #E1E8EE;"
            "border-radius: 6px;"
        ));

        auto *viewportLayout = new QVBoxLayout(viewport);
        viewportLayout->setContentsMargins(12, 12, 12, 12);
        auto *label = new QLabel(QStringLiteral("Plot placeholder — no data buffer"), viewport);
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet(QStringLiteral(
            "color: #8497A5;"
            "font-size: 11px;"
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

void MainWindow::setLeakUiState(LeakState state)
{
    if (leakStatus_ == nullptr || leakDot_ == nullptr) {
        return;
    }
    const QString stateText = leakStateDisplayText(state);
    leakStatus_->setText(stateText.startsWith(QStringLiteral("Leak: "))
                             ? stateText.mid(6)
                             : QStringLiteral("Wet"));
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
    portCombo_->setEnabled(!connected);
    baudSpin_->setEnabled(!connected);
    for (int index = 0; index < kServoCount; ++index) {
        refreshServoUi(index);
    }
    refreshMotionUi();
    refreshGaitBackendUi();
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
    enableButtons_[index]->setEnabled(connected && supported && !motionActive);
    neutralButtons_[index]->setEnabled(connected && supported && enabled
                                       && !pendingDisable && !motionActive);
    applyButtons_[index]->setEnabled(connected && supported && enabled
                                     && !pendingDisable && !motionActive);
    QString statusColor = QStringLiteral("#9e9e9e");
    if (!connected) {
        statusLabels_[index]->setText(QStringLiteral("Disconnected"));
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
    const bool actionable = enabled && controller_->isConnected()
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

    const bool connected = controller_->isConnected();
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

    const bool connected = controller_->isConnected();
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

void MainWindow::appendLog(const QString &message)
{
    log_->appendPlainText(message);
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
    controller_->shutdown();
    event->accept();
}

} // namespace rb