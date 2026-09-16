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

// Subtle industrial-console card style: light background, thin border, no
// gradients or glassmorphism, to keep the surface low-noise.
void applyCardStyle(QGroupBox *box)
{
    box->setStyleSheet(QStringLiteral(
        "QGroupBox {"
        "  border: 1px solid #cfd6dd;"
        "  border-radius: 6px;"
        "  margin-top: 10px;"
        "  padding: 8px;"
        "  background: #ffffff;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  left: 10px;"
        "  padding: 0 4px;"
        "  color: #37474f;"
        "  font-weight: bold;"
        "}"));
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
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(6, 6, 6, 6);
    root->setSpacing(8);

    root->addWidget(createConnectionBar());
    root->addWidget(createDashboard(), 1);
    root->addWidget(createActuatorPanel(), 1);
    root->addWidget(createLowerDashboard(), 1);

    setCentralWidget(central);

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
    auto *layout = new QHBoxLayout(bar);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(6);

    auto *title = new QLabel(QStringLiteral("RoboBeetle Console"), bar);
    title->setStyleSheet(QStringLiteral("font-weight: bold; font-size: 13px; color: #37474f;"));
    layout->addWidget(title);

    layout->addWidget(new QLabel(QStringLiteral("Serial Port"), bar));
    portCombo_ = new QComboBox(bar);
    portCombo_->setEditable(true);
    layout->addWidget(portCombo_);

    layout->addWidget(new QLabel(QStringLiteral("Baud Rate"), bar));
    baudSpin_ = new QSpinBox(bar);
    baudSpin_->setRange(1200, 3000000);
    baudSpin_->setValue(9600);
    layout->addWidget(baudSpin_);

    auto *refresh = new QPushButton(QStringLiteral("Refresh"), bar);
    connectButton_ = new QPushButton(QStringLiteral("Connect"), bar);
    layout->addWidget(refresh);
    layout->addWidget(connectButton_);

    auto *emergencyStop = new QPushButton(QStringLiteral("Emergency Stop"), bar);
    emergencyStop->setEnabled(false);
    emergencyStop->setToolTip(QStringLiteral("Protocol V2 has no Emergency Stop message in Phase 1"));
    layout->addWidget(emergencyStop);

    layout->addStretch();
    layout->addWidget(new QLabel(QStringLiteral("State"), bar));
    connectionStatus_ = new QLabel(QStringLiteral("Disconnected"), bar);
    connectionStatus_->setStyleSheet(QStringLiteral("font-weight: bold;"));
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
    applyCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    auto *title = new QLabel(QStringLiteral("Waiting for video stream"), box);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet(QStringLiteral("font-size: 15px; color: #455a64; font-weight: bold;"));
    auto *subtitle = new QLabel(QStringLiteral("Video backend not connected"), box);
    subtitle->setAlignment(Qt::AlignCenter);
    subtitle->setStyleSheet(QStringLiteral("color: #78909c;"));
    layout->addStretch();
    layout->addWidget(title);
    layout->addWidget(subtitle);
    layout->addStretch();
    return box;
}

QWidget *MainWindow::createLeakCard()
{
    auto *box = new QGroupBox(QStringLiteral("Leak Detection"), this);
    applyCardStyle(box);
    auto *layout = new QHBoxLayout(box);
    leakDot_ = new QLabel(box);
    leakDot_->setFixedSize(12, 12);
    leakDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 6px;"));
    leakStatus_ = new QLabel(QStringLiteral("Leak: Unknown"), box);
    leakStatus_->setStyleSheet(QStringLiteral("color: #666666; font-weight: bold;"));
    layout->addWidget(leakDot_);
    layout->addWidget(leakStatus_);
    layout->addStretch();
    return box;
}

QWidget *MainWindow::createImuCard()
{
    auto *box = new QGroupBox(QStringLiteral("IMU — JY901S"), this);
    applyCardStyle(box);
    auto *grid = new QGridLayout(box);
    imuDot_ = new QLabel(box);
    imuDot_->setFixedSize(10, 10);
    imuDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
    imuStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    imuStatus_->setStyleSheet(QStringLiteral("color: #666666; font-weight: bold;"));
    imuAcc_ = new QLabel(QStringLiteral("--"), box);
    imuGyro_ = new QLabel(QStringLiteral("--"), box);
    imuAngle_ = new QLabel(QStringLiteral("--"), box);
    grid->addWidget(imuDot_, 0, 0);
    grid->addWidget(imuStatus_, 0, 1, 1, 3);
    grid->addWidget(new QLabel(QStringLiteral("Acc"), box), 1, 0);
    grid->addWidget(imuAcc_, 1, 1);
    grid->addWidget(new QLabel(QStringLiteral("Gyro"), box), 1, 2);
    grid->addWidget(imuGyro_, 1, 3);
    grid->addWidget(new QLabel(QStringLiteral("Angle"), box), 2, 0);
    grid->addWidget(imuAngle_, 2, 1, 1, 3);
    grid->setColumnStretch(3, 1);
    return box;
}

QWidget *MainWindow::createDepthCard()
{
    auto *box = new QGroupBox(QStringLiteral("Depth Sensor — ROVMAKER"), this);
    applyCardStyle(box);
    auto *grid = new QGridLayout(box);
    depthDot_ = new QLabel(box);
    depthDot_->setFixedSize(10, 10);
    depthDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
    depthStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    depthStatus_->setStyleSheet(QStringLiteral("color: #666666; font-weight: bold;"));
    depthValue_ = new QLabel(QStringLiteral("--"), box);
    depthTemperature_ = new QLabel(QStringLiteral("--"), box);
    depthAge_ = new QLabel(QStringLiteral("--"), box);
    grid->addWidget(depthDot_, 0, 0);
    grid->addWidget(depthStatus_, 0, 1, 1, 3);
    grid->addWidget(new QLabel(QStringLiteral("Depth"), box), 1, 0);
    grid->addWidget(depthValue_, 1, 1);
    grid->addWidget(new QLabel(QStringLiteral("Temp"), box), 1, 2);
    grid->addWidget(depthTemperature_, 1, 3);
    grid->addWidget(new QLabel(QStringLiteral("Age"), box), 2, 0);
    grid->addWidget(depthAge_, 2, 1, 1, 3);
    grid->setColumnStretch(3, 1);
    return box;
}

QWidget *MainWindow::createProtocolSummaryCard()
{
    auto *box = new QGroupBox(QStringLiteral("Protocol / Link"), this);
    applyCardStyle(box);
    auto *layout = new QGridLayout(box);
    txCount_ = new QLabel(QStringLiteral("0"), box);
    rxCount_ = new QLabel(QStringLiteral("0"), box);
    crcCount_ = new QLabel(QStringLiteral("0"), box);
    timeoutCount_ = new QLabel(QStringLiteral("0"), box);
    ackRtt_ = new QLabel(QStringLiteral("—"), box);
    layout->addWidget(new QLabel(QStringLiteral("TX"), box), 0, 0);
    layout->addWidget(txCount_, 0, 1);
    layout->addWidget(new QLabel(QStringLiteral("RX"), box), 1, 0);
    layout->addWidget(rxCount_, 1, 1);
    layout->addWidget(new QLabel(QStringLiteral("CRC"), box), 2, 0);
    layout->addWidget(crcCount_, 2, 1);
    layout->addWidget(new QLabel(QStringLiteral("Timeout"), box), 3, 0);
    layout->addWidget(timeoutCount_, 3, 1);
    layout->addWidget(new QLabel(QStringLiteral("ACK RTT"), box), 4, 0);
    layout->addWidget(ackRtt_, 4, 1);
    layout->setColumnStretch(1, 1);
    return box;
}

QWidget *MainWindow::createStatusColumn()
{
    auto *column = new QWidget(this);
    auto *layout = new QVBoxLayout(column);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);
    layout->addWidget(createLeakCard());
    layout->addWidget(createImuCard(), 1);
    layout->addWidget(createDepthCard(), 1);
    layout->addWidget(createProtocolSummaryCard());
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
    auto *layout = new QVBoxLayout(box);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

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
    warning->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));
    layout->addWidget(warning);

    auto *statusRow = new QHBoxLayout;
    statusLabels_[index] = new QLabel(QStringLiteral("Disconnected"), box);
    statusLabels_[index]->setStyleSheet(QStringLiteral("font-weight: bold;"));
    statusRow->addWidget(statusLabels_[index]);
    statusRow->addStretch();
    layout->addLayout(statusRow);

    auto *pwmRow = new QHBoxLayout;
    pwmSpins_[index] = new QSpinBox(box);
    pwmSpins_[index]->setRange(descriptor == nullptr ? 0 : descriptor->commandMinPwmUs,
                               descriptor == nullptr ? 0 : descriptor->commandMaxPwmUs);
    pwmSpins_[index]->setValue(descriptor == nullptr ? 0 : descriptor->neutralPwmUs);
    pwmSpins_[index]->setSuffix(QStringLiteral(" μs"));
    pwmSpins_[index]->setEnabled(supported);
    pwmRow->addWidget(new QLabel(QStringLiteral("PWM"), box));
    pwmRow->addWidget(pwmSpins_[index], 1);
    layout->addLayout(pwmRow);

    pwmSliders_[index] = new QSlider(Qt::Horizontal, box);
    pwmSliders_[index]->setRange(descriptor == nullptr ? 0 : descriptor->commandMinPwmUs,
                                 descriptor == nullptr ? 0 : descriptor->commandMaxPwmUs);
    pwmSliders_[index]->setValue(descriptor == nullptr ? 0 : descriptor->neutralPwmUs);
    pwmSliders_[index]->setEnabled(supported);
    layout->addWidget(pwmSliders_[index]);

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

    auto *actionColumn = new QVBoxLayout;
    actionColumn->setSpacing(2);
    actionColumn->addWidget(enableButtons_[index]);
    actionColumn->addWidget(neutralButtons_[index]);
    actionColumn->addWidget(applyButtons_[index]);
    layout->addLayout(actionColumn);

    auto *angleRow = new QHBoxLayout;
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
    layout->addLayout(angleRow);

    const bool angleSupported = supported && descriptor != nullptr && descriptor->angleSupported;
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
    layout->addWidget(angleButtons_[index]);

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
    applyCardStyle(box);
    auto *layout = new QVBoxLayout(box);
    layout->setSpacing(4);

    auto *header = new QHBoxLayout;
    header->addStretch();
    auto *disableAll = new QPushButton(QStringLiteral("Disable All"), box);
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
    applyCardStyle(box);
    auto *layout = new QGridLayout(box);
    const MotionMode modes[] = {
        MotionMode::Forward,
        MotionMode::Backward,
        MotionMode::TurnLeft,
        MotionMode::TurnRight,
        MotionMode::Ascend,
        MotionMode::Descend,
    };
    const int modeCount = static_cast<int>(sizeof(modes) / sizeof(modes[0]));
    for (int index = 0; index < modeCount; ++index) {
        const MotionMode mode = modes[index];
        auto *button = new QPushButton(motionModeText(mode), box);
        button->setCheckable(true);
        button->setAutoExclusive(false);
        button->setStyleSheet(QStringLiteral(
            "QPushButton:checked { background-color: #1976D2; color: white; "
            "font-weight: bold; }"));
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
        layout->addWidget(
            motionButtons_[static_cast<std::size_t>(mode)],
            index / 3,
            index % 3);
    }
    motionStopButton_ = new QPushButton(QStringLiteral("Stop"), box);
    motionStatus_ = new QLabel(QStringLiteral("Stopped"), box);
    gaitBackendCombo_ = new QComboBox(box);
    gaitBackendCombo_->setObjectName(QStringLiteral("gaitBackendCombo"));
    gaitBackendCombo_->addItem(QStringLiteral("Unknown"), -1);
    gaitBackendCombo_->addItem(
        QStringLiteral("SimpleGait"), static_cast<int>(GaitBackend::SimpleGait));
    gaitBackendCombo_->addItem(
        QStringLiteral("CPG"), static_cast<int>(GaitBackend::CPG));
    gaitBackendStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    auto *provisional = new QLabel(
        QStringLiteral("Bench Provisional / Pending Water Verification"),
        box);
    provisional->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));

    layout->addWidget(new QLabel(QStringLiteral("Gait Backend:"), box), 2, 0);
    layout->addWidget(gaitBackendCombo_, 2, 1);
    layout->addWidget(gaitBackendStatus_, 2, 2);
    layout->addWidget(motionStopButton_, 3, 0);
    layout->addWidget(new QLabel(QStringLiteral("Status"), box), 3, 1);
    layout->addWidget(motionStatus_, 3, 2);
    layout->addWidget(provisional, 4, 0, 1, 3);

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

    connect(motionStopButton_, &QPushButton::clicked, this, [this] {
        controller_->stopMotion();
        refreshMotionUi();
    });
    refreshMotionUi();
    refreshGaitBackendUi();
    return box;
}

QWidget *MainWindow::createDataPlotsTab()
{
    // Independent central Data Plots region. Internally uses placeholder
    // sub-tabs (IMU | Depth | Actuator). No plotting backend or buffer.
    auto *tab = new QTabWidget(this);
    const QString titles[] = {
        QStringLiteral("IMU"),
        QStringLiteral("Depth"),
        QStringLiteral("Actuator"),
    };
    for (const QString &title : titles) {
        auto *placeholder = new QGroupBox(
            QStringLiteral("%1 Plots").arg(title), this);
        applyCardStyle(placeholder);
        auto *inner = new QVBoxLayout(placeholder);
        auto *label = new QLabel(QStringLiteral("Plot placeholder — no data buffer"), placeholder);
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet(QStringLiteral("color: #90a4ae;"));
        inner->addWidget(label);
        tab->addTab(placeholder, title);
    }
    return tab;
}

QWidget *MainWindow::createLogTab()
{
    auto *tab = new QWidget(this);
    auto *layout = new QVBoxLayout(tab);
    layout->setContentsMargins(4, 4, 4, 4);
    log_ = new QPlainTextEdit(tab);
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(1000);
    layout->addWidget(log_);
    return tab;
}

QWidget *MainWindow::createTelemetryDetailsTab()
{
    auto *tab = new QWidget(this);
    auto *layout = new QVBoxLayout(tab);
    layout->setContentsMargins(4, 4, 4, 4);
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
    layout->setContentsMargins(4, 4, 4, 4);
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
    tabs->addTab(createLogTab(), QStringLiteral("Log"));
    tabs->addTab(createTelemetryDetailsTab(), QStringLiteral("Telemetry Details"));
    tabs->addTab(createProtocolDetailsTab(), QStringLiteral("Protocol Details"));
    return tabs;
}

QWidget *MainWindow::createLowerDashboard()
{
    // Motion + independent Data Plots + Log/Protocol Details side by side.
    auto *lower = new QWidget(this);
    auto *layout = new QHBoxLayout(lower);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    layout->addWidget(createMotionPanel());
    layout->addWidget(createDataPlotsTab(), 1);
    layout->addWidget(createLogDetailsTabs(), 1);
    return lower;
}

void MainWindow::setLeakUiState(LeakState state)
{
    if (leakStatus_ == nullptr || leakDot_ == nullptr) {
        return;
    }
    leakStatus_->setText(leakStateDisplayText(state));
    switch (state) {
    case LeakState::Unknown:
        leakStatus_->setStyleSheet(QStringLiteral("color: #666666; font-weight: bold;"));
        leakDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 6px;"));
        break;
    case LeakState::Dry:
        leakStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-weight: bold;"));
        leakDot_->setStyleSheet(QStringLiteral("background: #43A047; border-radius: 6px;"));
        break;
    case LeakState::Wet:
        leakStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-weight: bold;"));
        leakDot_->setStyleSheet(QStringLiteral("background: #E53935; border-radius: 6px;"));
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
        imuStatus_->setStyleSheet(QStringLiteral("color: #666666; font-weight: bold;"));
        imuDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
        break;
    case ImuStatus::Receiving:
        imuStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-weight: bold;"));
        imuDot_->setStyleSheet(QStringLiteral("background: #43A047; border-radius: 5px;"));
        break;
    case ImuStatus::Stale:
        imuStatus_->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));
        imuDot_->setStyleSheet(QStringLiteral("background: #FB8C00; border-radius: 5px;"));
        break;
    case ImuStatus::Error:
        imuStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-weight: bold;"));
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
        depthStatus_->setStyleSheet(QStringLiteral("color: #666666; font-weight: bold;"));
        depthDot_->setStyleSheet(QStringLiteral("background: #9e9e9e; border-radius: 5px;"));
        break;
    case DepthStatus::Receiving:
        depthStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-weight: bold;"));
        depthDot_->setStyleSheet(QStringLiteral("background: #43A047; border-radius: 5px;"));
        break;
    case DepthStatus::Stale:
        depthStatus_->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));
        depthDot_->setStyleSheet(QStringLiteral("background: #FB8C00; border-radius: 5px;"));
        break;
    case DepthStatus::Error:
        depthStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-weight: bold;"));
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
    if (!connected) {
        statusLabels_[index]->setText(QStringLiteral("Disconnected"));
    } else if (!supported) {
        statusLabels_[index]->setText(QStringLiteral("Unsupported"));
    } else if (pendingDisable) {
        statusLabels_[index]->setText(QStringLiteral("Disable pending ACK"));
    } else if (enabled) {
        statusLabels_[index]->setText(QStringLiteral("Enabled / ACKed"));
    } else if (descriptor->calibrationPending) {
        statusLabels_[index]->setText(QStringLiteral("Disabled — Calibration Pending"));
    } else {
        statusLabels_[index]->setText(QStringLiteral("Disabled"));
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