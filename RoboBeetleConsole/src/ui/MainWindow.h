#pragma once

#include "robot/RobotController.h"

#include <QMainWindow>

#include <array>

class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QSpinBox;

namespace rb {

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(RobotController *controller, QWidget *parent = nullptr);

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    QWidget *createServoPanel(int index, ServoId id);
    QWidget *createConnectionPanel();
    QWidget *createGlobalPanel();
    QWidget *createMotionPanel();
    QWidget *createImuPanel();
    QWidget *createDepthPanel();
    QWidget *createMonitorPanel();
    void setConnectedUi(bool connected);
    void refreshServoUi(int index);
    void setAngleUiEnabled(int index, bool enabled);
    void setLeakUiState(LeakState state);
    void setImuUiState(const ImuMonitorState &state);
    void setDepthUiState(const DepthMonitorState &state);
    void refreshMotionUi();
    void appendLog(const QString &message);
    static QString stateText(TransportState state);

    RobotController *controller_;
    QComboBox *portCombo_{nullptr};
    QSpinBox *baudSpin_{nullptr};
    QPushButton *connectButton_{nullptr};
    QLabel *connectionStatus_{nullptr};
    QLabel *leakStatus_{nullptr};
    QComboBox *motionModeCombo_{nullptr};
    QPushButton *motionStartButton_{nullptr};
    QPushButton *motionStopButton_{nullptr};
    QLabel *motionStatus_{nullptr};
    QLabel *imuStatus_{nullptr};
    QLabel *imuAcc_{nullptr};
    QLabel *imuGyro_{nullptr};
    QLabel *imuAngle_{nullptr};
    QLabel *imuDiagnostics_{nullptr};
    QLabel *depthStatus_{nullptr};
    QLabel *depthValue_{nullptr};
    QLabel *depthTemperature_{nullptr};
    QLabel *depthAge_{nullptr};
    QLabel *depthDiagnostics_{nullptr};
    std::array<QSpinBox *, kServoCount> pwmSpins_{};
    std::array<QSlider *, kServoCount> pwmSliders_{};
    std::array<QDoubleSpinBox *, kServoCount> angleSpins_{};
    std::array<QPushButton *, kServoCount> angleButtons_{};
    std::array<QPushButton *, kServoCount> enableButtons_{};
    std::array<QPushButton *, kServoCount> neutralButtons_{};
    std::array<QPushButton *, kServoCount> applyButtons_{};
    std::array<QLabel *, kServoCount> statusLabels_{};
    QLineEdit *txHex_{nullptr};
    QLineEdit *rxHex_{nullptr};
    QLabel *txCount_{nullptr};
    QLabel *rxCount_{nullptr};
    QLabel *crcCount_{nullptr};
    QLabel *timeoutCount_{nullptr};
    QLabel *ackRtt_{nullptr};
    QLabel *ackStatus_{nullptr};
    QPlainTextEdit *log_{nullptr};
};

} // namespace rb

