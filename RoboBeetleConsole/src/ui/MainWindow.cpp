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

namespace rb {

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
    neutralButtons_[index] = new QPushButton(QStringLiteral("Neutral"), box);
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
    auto *disableAll = new QPushButton(QStringLiteral("Disable All"), box);
    auto *emergencyStop = new QPushButton(QStringLiteral("Emergency Stop — Not Implemented"), box);
    emergencyStop->setEnabled(false);
    emergencyStop->setToolTip(QStringLiteral("Protocol V2 has no Emergency Stop message in Phase 1"));
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
    enableButtons_[index]->setText(enabled ? QStringLiteral("Disable") : QStringLiteral("Enable"));
    enableButtons_[index]->setEnabled(connected && supported);
    neutralButtons_[index]->setEnabled(connected && supported && enabled);
    applyButtons_[index]->setEnabled(connected && supported && enabled);
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
        && !controller_->isServoDisablePending(id);
    angleSpins_[index]->setEnabled(actionable);
    angleButtons_[index]->setEnabled(actionable);
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
