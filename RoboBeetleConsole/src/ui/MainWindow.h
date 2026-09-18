#pragma once

#include "controller/IConsoleController.h"

#include <QMainWindow>

#include <array>
#include <cstddef>

class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTabWidget;
class QWidget;

namespace rb {

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(IConsoleController *controller, QWidget *parent = nullptr);

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    // Top connection bar.
    QWidget *createConnectionBar();
    // Dashboard (top): video placeholder + status cards.
    QWidget *createVideoPlaceholder();
    QWidget *createLeakCard();
    QWidget *createImuCard();
    QWidget *createDepthCard();
    QWidget *createProtocolSummaryCard();
    QWidget *createStatusColumn();
    QWidget *createDashboard();
    // Actuator (full width) + lower dashboard.
    QWidget *createServoPanel(int index, ServoId id);
    QWidget *createActuatorPanel();
    QWidget *createMotionPanel();
    // Lower dashboard: Data Plots (independent) + Log | Telemetry Details |
// Protocol Details tabs.
    QWidget *createDataPlotsTab();
    QWidget *createLogTab();
    QWidget *createTelemetryDetailsTab();
    QWidget *createProtocolDetailsTab();
    QTabWidget *createLogDetailsTabs();
    QWidget *createLowerDashboard();
    // Refresh helpers.
    void setConnectedUi(bool connected);
    void refreshServoUi(int index);
    void setAngleUiEnabled(int index, bool enabled);
    void setLeakUiState(LeakState state);
    void setImuUiState(const ImuMonitorState &state);
    void setDepthUiState(const DepthMonitorState &state);
    void refreshMotionUi();
    void refreshGaitBackendUi();
    void refreshAuthorityUi();
    void appendLog(const QString &message);
    static QString stateText(TransportState state);

    IConsoleController *controller_;

    // Connection bar.
    QComboBox *portCombo_{nullptr};
    QSpinBox *baudSpin_{nullptr};
    QPushButton *connectButton_{nullptr};
    QPushButton *acquireButton_{nullptr};
    QPushButton *releaseButton_{nullptr};
    QLabel *connectionStatus_{nullptr};
    QLabel *authorityStatus_{nullptr};

    // Leak card.
    QLabel *leakStatus_{nullptr};
    QLabel *leakDot_{nullptr};

    // Motion card.
    std::array<QPushButton *, static_cast<std::size_t>(MotionMode::Count)> motionButtons_{};
    QPushButton *motionStopButton_{nullptr};
    QLabel *motionStatus_{nullptr};
    QComboBox *gaitBackendCombo_{nullptr};
    QLabel *gaitBackendStatus_{nullptr};

    // IMU card.
    QLabel *imuDot_{nullptr};
    QLabel *imuStatus_{nullptr};
    QLabel *imuAcc_{nullptr};
    QLabel *imuGyro_{nullptr};
    QLabel *imuAngle_{nullptr};

    // Depth card.
    QLabel *depthDot_{nullptr};
    QLabel *depthStatus_{nullptr};
    QLabel *depthValue_{nullptr};
    QLabel *depthTemperature_{nullptr};
    QLabel *depthAge_{nullptr};

    // Telemetry details tab (holds the long IMU/Depth diagnostics text).
    QLabel *imuDiagnostics_{nullptr};
    QLabel *depthDiagnostics_{nullptr};

    // Servo cards (one per semantic actuator).
    std::array<QSpinBox *, kServoCount> pwmSpins_{};
    std::array<QSlider *, kServoCount> pwmSliders_{};
    std::array<QDoubleSpinBox *, kServoCount> angleSpins_{};
    std::array<QPushButton *, kServoCount> angleButtons_{};
    std::array<QPushButton *, kServoCount> enableButtons_{};
    std::array<QPushButton *, kServoCount> neutralButtons_{};
    std::array<QPushButton *, kServoCount> applyButtons_{};
    std::array<QLabel *, kServoCount> statusLabels_{};

    // Protocol summary card (dashboard) + protocol details tab.
    QLineEdit *txHex_{nullptr};
    QLineEdit *rxHex_{nullptr};
    QLabel *txCount_{nullptr};
    QLabel *rxCount_{nullptr};
    QLabel *crcCount_{nullptr};
    QLabel *timeoutCount_{nullptr};
    QLabel *ackRtt_{nullptr};
    QLabel *ackStatus_{nullptr};

    // Log tab.
    QPlainTextEdit *log_{nullptr};
};

} // namespace rb