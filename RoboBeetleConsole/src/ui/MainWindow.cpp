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
#include <QSlider>
#include <QSpinBox>
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
    case MotionMode::Stop: return QStringLiteral("STOP");
    case MotionMode::Forward: return QStringLiteral("FORWARD");
    case MotionMode::Backward: return QStringLiteral("BACKWARD");
    case MotionMode::TurnLeft: return QStringLiteral("TURN_LEFT");
    case MotionMode::TurnRight: return QStringLiteral("TURN_RIGHT");
    case MotionMode::Ascend: return QStringLiteral("ASCEND");
    case MotionMode::Descend: return QStringLiteral("DESCEND");
    case MotionMode::Count: break;
    }
    return QStringLiteral("UNKNOWN");
}

} // namespace

MainWindow::MainWindow(RobotController *controller, QWidget *parent)
    : QMainWindow(parent), controller_(controller)
{
    Q_ASSERT(controller_ != nullptr);
    setWindowTitle(QStringLiteral("RoboBeetle Console — My first Qt"));
    resize(1000, 760);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    root->addWidget(createConnectionPanel());

    auto *servos = new QGridLayout;
    const auto &descriptors = servoDescriptorTable();
    for (int index = 0; index < kServoCount; ++index) {
        servos->addWidget(createServoPanel(index, descriptors.at(index).id),
                          index / 2, index % 2);
    }
    root->addLayout(servos);
    root->addWidget(createGlobalPanel());
    root->addWidget(createMotionPanel());
    root->addWidget(createImuPanel());
    root->addWidget(createDepthPanel());
    root->addWidget(createMonitorPanel(), 1);
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
    });
    connect(controller_, &RobotController::servoDisablePendingChanged,
            this, [this](int index, bool) {
        if (index < 0 || index >= kServoCount) {
            return;
        }
        refreshServoUi(index);
    });
    connect(controller_, &RobotController::motionStateChanged,
            this, [this](MotionState, MotionMode) {
                refreshMotionUi();
                for (int index = 0; index < kServoCount; ++index) {
                    refreshServoUi(index);
                }
            });
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
    controller_->refreshSerialPorts();
}

QWidget *MainWindow::createConnectionPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Connection"), this);
    auto *layout = new QGridLayout(box);
    portCombo_ = new QComboBox(box);
    portCombo_->setEditable(true);
    baudSpin_ = new QSpinBox(box);
    baudSpin_->setRange(1200, 3000000);
    baudSpin_->setValue(9600);
    auto *refresh = new QPushButton(QStringLiteral("Refresh"), box);
    connectButton_ = new QPushButton(QStringLiteral("Connect"), box);
    connectionStatus_ = new QLabel(QStringLiteral("Disconnected"), box);

    layout->addWidget(new QLabel(QStringLiteral("Serial port"), box), 0, 0);
    layout->addWidget(portCombo_, 0, 1);
    layout->addWidget(new QLabel(QStringLiteral("Baud rate"), box), 0, 2);
    layout->addWidget(baudSpin_, 0, 3);
    layout->addWidget(refresh, 0, 4);
    layout->addWidget(connectButton_, 0, 5);
    layout->addWidget(new QLabel(QStringLiteral("State"), box), 1, 0);
    layout->addWidget(connectionStatus_, 1, 1, 1, 5);

    connect(refresh, &QPushButton::clicked, controller_, &RobotController::refreshSerialPorts);
    connect(connectButton_, &QPushButton::clicked, this, [this] {
        if (controller_->isConnected()) {
            controller_->disconnectTransport();
            return;
        }
        controller_->connectTransport({portCombo_->currentText().trimmed(), baudSpin_->value()});
    });
    return box;
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
    auto *layout = new QGridLayout(box);
    const QString warningText = descriptor == nullptr || !supported
        ? QStringLiteral("UNSUPPORTED — NO HARDWARE CHANNEL")
        : descriptor->calibrationPending
            ? QStringLiteral("CALIBRATION PENDING — PWM bring-up only: %1–%2 μs")
                  .arg(descriptor->commandMinPwmUs)
                  .arg(descriptor->commandMaxPwmUs)
            : QStringLiteral("PWM command range: %1–%2 μs; angle: %3–%4°")
                  .arg(descriptor->commandMinPwmUs)
                  .arg(descriptor->commandMaxPwmUs)
                  .arg(static_cast<double>(descriptor->commandMinAngleCdeg) / 100.0, 0, 'f', 1)
                  .arg(static_cast<double>(descriptor->commandMaxAngleCdeg) / 100.0, 0, 'f', 1);
    auto *warning = new QLabel(warningText, box);
    warning->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));

    pwmSpins_[index] = new QSpinBox(box);
    pwmSpins_[index]->setRange(descriptor == nullptr ? 0 : descriptor->commandMinPwmUs,
                               descriptor == nullptr ? 0 : descriptor->commandMaxPwmUs);
    pwmSpins_[index]->setValue(descriptor == nullptr ? 0 : descriptor->neutralPwmUs);
    pwmSpins_[index]->setSuffix(QStringLiteral(" μs"));
    pwmSliders_[index] = new QSlider(Qt::Horizontal, box);
    pwmSliders_[index]->setRange(descriptor == nullptr ? 0 : descriptor->commandMinPwmUs,
                                 descriptor == nullptr ? 0 : descriptor->commandMaxPwmUs);
    pwmSliders_[index]->setValue(descriptor == nullptr ? 0 : descriptor->neutralPwmUs);
    pwmSpins_[index]->setEnabled(supported);
    pwmSliders_[index]->setEnabled(supported);
    enableButtons_[index] = new QPushButton(QStringLiteral("Enable"), box);
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
    statusLabels_[index] = new QLabel(QStringLiteral("Disconnected"), box);

    layout->addWidget(warning, 0, 0, 1, 3);
    layout->addWidget(new QLabel(QStringLiteral("Status"), box), 1, 0);
    layout->addWidget(statusLabels_[index], 1, 1, 1, 2);
    layout->addWidget(new QLabel(QStringLiteral("PWM"), box), 2, 0);
    layout->addWidget(pwmSpins_[index], 2, 1);
    layout->addWidget(pwmSliders_[index], 3, 0, 1, 3);
    layout->addWidget(enableButtons_[index], 4, 0);
    layout->addWidget(neutralButtons_[index], 4, 1);
    layout->addWidget(applyButtons_[index], 4, 2);
    layout->addWidget(new QLabel(QStringLiteral("Angle"), box), 5, 0);
    layout->addWidget(angleSpins_[index], 5, 1);
    layout->addWidget(angleButtons_[index], 5, 2);

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

QWidget *MainWindow::createGlobalPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Global"), this);
    auto *layout = new QHBoxLayout(box);
    leakStatus_ = new QLabel(QStringLiteral("Leak: Unknown"), box);
    auto *disableAll = new QPushButton(QStringLiteral("Disable All"), box);
    auto *emergencyStop = new QPushButton(QStringLiteral("Emergency Stop — Not Implemented"), box);
    emergencyStop->setEnabled(false);
    emergencyStop->setToolTip(QStringLiteral("Protocol V2 has no Emergency Stop message in Phase 1"));
    layout->addWidget(leakStatus_);
    layout->addWidget(disableAll);
    layout->addWidget(emergencyStop);
    layout->addStretch();
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
    auto *layout = new QGridLayout(box);
    motionModeCombo_ = new QComboBox(box);
    const MotionMode modes[] = {
        MotionMode::Stop,
        MotionMode::Forward,
        MotionMode::Backward,
        MotionMode::TurnLeft,
        MotionMode::TurnRight,
        MotionMode::Ascend,
        MotionMode::Descend,
    };
    for (const MotionMode mode : modes) {
        motionModeCombo_->addItem(
            motionModeText(mode),
            static_cast<int>(mode));
    }
    motionStartButton_ = new QPushButton(QStringLiteral("Start"), box);
    motionStopButton_ = new QPushButton(QStringLiteral("Stop"), box);
    motionStatus_ = new QLabel(QStringLiteral("Stopped"), box);
    auto *provisional = new QLabel(
        QStringLiteral("Bench Provisional / Pending Water Verification"),
        box);
    provisional->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));

    layout->addWidget(new QLabel(QStringLiteral("Mode"), box), 0, 0);
    layout->addWidget(motionModeCombo_, 0, 1);
    layout->addWidget(motionStartButton_, 0, 2);
    layout->addWidget(motionStopButton_, 0, 3);
    layout->addWidget(new QLabel(QStringLiteral("Status"), box), 1, 0);
    layout->addWidget(motionStatus_, 1, 1, 1, 3);
    layout->addWidget(provisional, 2, 0, 1, 4);

    connect(motionModeCombo_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int) { refreshMotionUi(); });
    connect(motionStartButton_, &QPushButton::clicked, this, [this] {
        const MotionMode mode = static_cast<MotionMode>(
            motionModeCombo_->currentData().toInt());
        controller_->startMotion(mode);
        refreshMotionUi();
    });
    connect(motionStopButton_, &QPushButton::clicked, this, [this] {
        controller_->stopMotion();
        refreshMotionUi();
    });
    refreshMotionUi();
    return box;
}

void MainWindow::setLeakUiState(LeakState state)
{
    if (leakStatus_ == nullptr) {
        return;
    }
    leakStatus_->setText(leakStateDisplayText(state));
    switch (state) {
    case LeakState::Unknown:
        leakStatus_->setStyleSheet(QStringLiteral("color: #666666; font-weight: bold;"));
        break;
    case LeakState::Dry:
        leakStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-weight: bold;"));
        break;
    case LeakState::Wet:
        leakStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-weight: bold;"));
        break;
    }
}

QWidget *MainWindow::createImuPanel()
{
    auto *box = new QGroupBox(QStringLiteral("IMU — JY901S"), this);
    auto *form = new QFormLayout(box);
    imuStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    imuAcc_ = new QLabel(QStringLiteral("--"), box);
    imuGyro_ = new QLabel(QStringLiteral("--"), box);
    imuAngle_ = new QLabel(QStringLiteral("--"), box);
    imuDiagnostics_ = new QLabel(QStringLiteral("--"), box);
    imuDiagnostics_->setWordWrap(true);
    form->addRow(QStringLiteral("Status"), imuStatus_);
    form->addRow(QStringLiteral("Acc"), imuAcc_);
    form->addRow(QStringLiteral("Gyro"), imuGyro_);
    form->addRow(QStringLiteral("Angle"), imuAngle_);
    form->addRow(QStringLiteral("Diagnostics"), imuDiagnostics_);
    return box;
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
        break;
    case ImuStatus::Receiving:
        imuStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-weight: bold;"));
        break;
    case ImuStatus::Stale:
        imuStatus_->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));
        break;
    case ImuStatus::Error:
        imuStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-weight: bold;"));
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

QWidget *MainWindow::createDepthPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Depth Sensor — ROVMAKER"), this);
    auto *form = new QFormLayout(box);
    depthStatus_ = new QLabel(QStringLiteral("Unknown"), box);
    depthValue_ = new QLabel(QStringLiteral("--"), box);
    depthTemperature_ = new QLabel(QStringLiteral("--"), box);
    depthAge_ = new QLabel(QStringLiteral("--"), box);
    depthDiagnostics_ = new QLabel(QStringLiteral("--"), box);
    depthDiagnostics_->setWordWrap(true);
    form->addRow(QStringLiteral("Status"), depthStatus_);
    form->addRow(QStringLiteral("Depth"), depthValue_);
    form->addRow(QStringLiteral("Temperature"), depthTemperature_);
    form->addRow(QStringLiteral("Sample age"), depthAge_);
    form->addRow(QStringLiteral("Diagnostics"), depthDiagnostics_);
    return box;
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
        break;
    case DepthStatus::Receiving:
        depthStatus_->setStyleSheet(QStringLiteral("color: #228B22; font-weight: bold;"));
        break;
    case DepthStatus::Stale:
        depthStatus_->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));
        break;
    case DepthStatus::Error:
        depthStatus_->setStyleSheet(QStringLiteral("color: #B00020; font-weight: bold;"));
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

QWidget *MainWindow::createMonitorPanel()
{
    auto *box = new QGroupBox(QStringLiteral("Protocol Monitor"), this);
    auto *layout = new QVBoxLayout(box);
    auto *form = new QFormLayout;
    txHex_ = new QLineEdit(box);
    rxHex_ = new QLineEdit(box);
    txHex_->setReadOnly(true);
    rxHex_->setReadOnly(true);
    txCount_ = new QLabel(QStringLiteral("0"), box);
    rxCount_ = new QLabel(QStringLiteral("0"), box);
    crcCount_ = new QLabel(QStringLiteral("0"), box);
    timeoutCount_ = new QLabel(QStringLiteral("0"), box);
    ackRtt_ = new QLabel(QStringLiteral("—"), box);
    ackStatus_ = new QLabel(QStringLiteral("Idle"), box);
    form->addRow(QStringLiteral("TX Hex"), txHex_);
    form->addRow(QStringLiteral("RX Hex"), rxHex_);

    auto *counts = new QHBoxLayout;
    counts->addWidget(new QLabel(QStringLiteral("TX packets:"), box));
    counts->addWidget(txCount_);
    counts->addWidget(new QLabel(QStringLiteral("RX packets:"), box));
    counts->addWidget(rxCount_);
    counts->addWidget(new QLabel(QStringLiteral("CRC errors:"), box));
    counts->addWidget(crcCount_);
    counts->addWidget(new QLabel(QStringLiteral("Timeouts:"), box));
    counts->addWidget(timeoutCount_);
    counts->addWidget(new QLabel(QStringLiteral("Last ACK RTT:"), box));
    counts->addWidget(ackRtt_);
    counts->addStretch();

    log_ = new QPlainTextEdit(box);
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(1000);
    layout->addLayout(form);
    layout->addLayout(counts);
    layout->addWidget(new QLabel(QStringLiteral("ACK status"), box));
    layout->addWidget(ackStatus_);
    layout->addWidget(new QLabel(QStringLiteral("Event log"), box));
    layout->addWidget(log_, 1);
    return box;
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
    enableButtons_[index]->setText(enabled ? QStringLiteral("Disable") : QStringLiteral("Enable"));
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
    if (motionModeCombo_ == nullptr || motionStartButton_ == nullptr
        || motionStopButton_ == nullptr || motionStatus_ == nullptr) {
        return;
    }

    const MotionState state = controller_->motionState();
    const MotionMode selectedMode = static_cast<MotionMode>(
        motionModeCombo_->currentData().toInt());
    switch (state) {
    case MotionState::Stopped:
        motionStatus_->setText(QStringLiteral("Stopped"));
        break;
    case MotionState::Running:
        motionStatus_->setText(QStringLiteral("Running %1")
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
    motionModeCombo_->setEnabled(connected && state != MotionState::Stopping);
    motionStartButton_->setEnabled(
        connected && selectedMode != MotionMode::Stop
        && state != MotionState::Stopping
        && controller_->isMotionReady(selectedMode));
    motionStopButton_->setEnabled(connected && controller_->isMotionActive());
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
