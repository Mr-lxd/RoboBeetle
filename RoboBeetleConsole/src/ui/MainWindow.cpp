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

    auto *servos = new QHBoxLayout;
    servos->addWidget(createServoPanel(0, ServoId::Servo1));
    servos->addWidget(createServoPanel(1, ServoId::Servo2));
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
        if (index < 0 || index >= 2) {
            return;
        }
        enableButtons_[index]->setText(enabled ? QStringLiteral("Disable") : QStringLiteral("Enable"));
        neutralButtons_[index]->setEnabled(enabled);
        applyButtons_[index]->setEnabled(enabled);
        setAngleUiEnabled(index, enabled);
    });
    connect(controller_, &RobotController::servoDisablePendingChanged,
            this, [this](int index, bool) {
        if (index < 0 || index >= 2) {
            return;
        }
        setAngleUiEnabled(index, controller_->isServoEnabled(static_cast<ServoId>(index)));
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
    const bool supported = controller_->isServoSupported(id);
    auto *box = new QGroupBox(
        supported
            ? QStringLiteral("Servo %1").arg(index + 1)
            : QStringLiteral("Servo %1 — Unsupported / Planned").arg(index + 1),
        this);
    auto *layout = new QGridLayout(box);
    const RobotControllerConfig config = controller_->config();
    auto *warning = new QLabel(
        supported
            ? QStringLiteral("BRING-UP PROVISIONAL: %1–%2 μs")
                  .arg(config.provisionalPwmMinUs)
                  .arg(config.provisionalPwmMaxUs)
            : QStringLiteral("UNSUPPORTED IN PHASE 1 — NO HARDWARE CHANNEL"),
        box);
    warning->setStyleSheet(QStringLiteral("color: #b35c00; font-weight: bold;"));

    pwmSpins_[index] = new QSpinBox(box);
    pwmSpins_[index]->setRange(config.provisionalPwmMinUs, config.provisionalPwmMaxUs);
    pwmSpins_[index]->setValue(config.provisionalNeutralUs);
    pwmSpins_[index]->setSuffix(QStringLiteral(" μs"));
    pwmSliders_[index] = new QSlider(Qt::Horizontal, box);
    pwmSliders_[index]->setRange(config.provisionalPwmMinUs, config.provisionalPwmMaxUs);
    pwmSliders_[index]->setValue(config.provisionalNeutralUs);
    pwmSpins_[index]->setEnabled(supported);
    pwmSliders_[index]->setEnabled(supported);
    enableButtons_[index] = new QPushButton(QStringLiteral("Enable"), box);
    neutralButtons_[index] = new QPushButton(QStringLiteral("Neutral"), box);
    applyButtons_[index] = new QPushButton(QStringLiteral("Apply PWM"), box);
    angleSpins_[index] = new QDoubleSpinBox(box);
    angleSpins_[index]->setRange(static_cast<double>(config.provisionalAngleMinCdeg) / 100.0,
                                 static_cast<double>(config.provisionalAngleMaxCdeg) / 100.0);
    angleSpins_[index]->setDecimals(1);
    angleSpins_[index]->setSingleStep(0.1);
    angleSpins_[index]->setValue(0.0);
    angleSpins_[index]->setSuffix(QStringLiteral(" deg"));
    angleSpins_[index]->setEnabled(false);
    angleButtons_[index] = new QPushButton(
        supported ? QStringLiteral("Set Angle") : QStringLiteral("Set Angle — Unsupported / Planned"), box);
    angleButtons_[index]->setEnabled(false);
    angleButtons_[index]->setToolTip(
        supported
            ? QStringLiteral("Send Servo1 angle as Protocol V2 centidegrees after Enable ACK")
            : QStringLiteral("Servo2 is unsupported in Phase 1 and cannot receive angle commands"));

    layout->addWidget(warning, 0, 0, 1, 3);
    layout->addWidget(new QLabel(QStringLiteral("PWM"), box), 1, 0);
    layout->addWidget(pwmSpins_[index], 1, 1);
    layout->addWidget(pwmSliders_[index], 2, 0, 1, 3);
    layout->addWidget(enableButtons_[index], 3, 0);
    layout->addWidget(neutralButtons_[index], 3, 1);
    layout->addWidget(applyButtons_[index], 3, 2);
    layout->addWidget(new QLabel(QStringLiteral("Angle"), box), 4, 0);
    layout->addWidget(angleSpins_[index], 4, 1);
    layout->addWidget(angleButtons_[index], 4, 2);

    connect(pwmSpins_[index], qOverload<int>(&QSpinBox::valueChanged),
            pwmSliders_[index], &QSlider::setValue);
    connect(pwmSliders_[index], &QSlider::valueChanged,
            pwmSpins_[index], &QSpinBox::setValue);
    connect(enableButtons_[index], &QPushButton::clicked, this, [this, id, index] {
        if (controller_->isServoEnabled(id)) {
            controller_->disableServo(id);
            setAngleUiEnabled(index, controller_->isServoEnabled(id));
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
            || controller_->isServoDisablePending(id)) {
            setAngleUiEnabled(index, controller_->isServoEnabled(id));
            return;
        }
        controller_->setServoAngle(id, angleDegreesToCentidegrees(angleSpins_[index]->value()));
    });
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
        for (int index = 0; index < 2; ++index) {
            setAngleUiEnabled(index, controller_->isServoEnabled(static_cast<ServoId>(index)));
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
    for (int index = 0; index < 2; ++index) {
        const ServoId id = static_cast<ServoId>(index);
        enableButtons_[index]->setEnabled(connected && controller_->isServoSupported(id));
        applyButtons_[index]->setEnabled(connected && controller_->isServoEnabled(static_cast<ServoId>(index)));
        neutralButtons_[index]->setEnabled(connected && controller_->isServoEnabled(static_cast<ServoId>(index)));
        setAngleUiEnabled(index, connected);
    }
}

void MainWindow::setAngleUiEnabled(int index, bool enabled)
{
    if (index < 0 || index >= 2) {
        return;
    }
    const ServoId id = static_cast<ServoId>(index);
    const bool actionable = enabled && controller_->isConnected()
        && controller_->isServoSupported(id) && controller_->isServoEnabled(id)
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
