#pragma once

#include "controller/IConsoleController.h"
#include "vision/DetectionMetadata.h"

#include <QMainWindow>
#include <QSize>

#include <array>
#include <cstddef>
#include <optional>

class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QSplitter;
class QTabWidget;
class QWidget;
class QEvent;
class QGridLayout;
class QScrollArea;

namespace rb {

namespace ui {
class ElidedLabel;
}

namespace vision {
class DetectionClient;
class VideoView;
class VisionClient;
class VisionControlClient;
}

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(
        IConsoleController *controller,
        vision::VisionClient *visionClient = nullptr,
        vision::VisionControlClient *visionControlClient = nullptr,
        vision::DetectionClient *detectionClient = nullptr,
        QWidget *parent = nullptr);

protected:
    void closeEvent(QCloseEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

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
    QWidget *createServoFineControlTab();
    // Lower dashboard: Data Plots (independent) + Log | Telemetry Details |
// Protocol Details tabs.
    QWidget *createDataPlotsTab();
    QWidget *createLogTab();
    QWidget *createTelemetryDetailsTab();
    QWidget *createProtocolDetailsTab();
    QTabWidget *createLogDetailsTabs();
    QWidget *createLowerDashboard();
    QWidget *createOperatorTools();
    QWidget *createOperatorActionBar();
    QTabWidget *createOperatorToolsTabs();
    QWidget *createVisionDetailsTab();
    void bindVisionUi();
    void bindControllerUi();
    void refreshVideoDiagnosticsUi();
    void refreshProtocolUi(const ProtocolMonitor &monitor);
    void refreshVisionNoticeUi();
    void refreshVisionEndpointUi();
    void syncDetectionStream();
    void refreshDetectionOverlay();
    void resetDetectionSession();
    void reflowActuatorCards();
    void updateVideoSurfaceGeometry();
    void initializeWorkspaceSizes();
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
    void refreshVisionUi();
    void refreshCaptureUi();
    void refreshInferenceUi();
    void refreshPiHostUi();
    void applyPiHost();
    [[nodiscard]] bool hasRemoteVisionWorkThatMayContinue() const;
    [[nodiscard]] bool hostCandidateValid(const QString &candidate) const;
    [[nodiscard]] bool hostDirty() const;
    void appendLog(const QString &message);
    static QString stateText(TransportState state);

    IConsoleController *controller_;
    vision::VisionClient *visionClient_{nullptr};
    vision::VisionControlClient *visionControlClient_{nullptr};
    vision::DetectionClient *detectionClient_{nullptr};

    // Vision card.
    QLineEdit *piHost_{nullptr};
    QPushButton *applyPiHostButton_{nullptr};
    QLabel *piHostHint_{nullptr};
    QString committedPiHost_;
    bool updatingEndpoints_{false};
    bool visionOutcomeUncertain_{false};
    QString visionNoticeText_;
    QString visionNoticeTooltip_;
    TransportState transportState_{TransportState::Disconnected};
    QSpinBox *visionPort_{nullptr};
    QPushButton *visionConnectButton_{nullptr};
    QLabel *visionState_{nullptr};
    QLabel *visionDot_{nullptr};
    QLabel *visionDiagnostics_{nullptr};
    QLabel *videoFpsSummary_{nullptr};
    QLabel *videoResolutionSummary_{nullptr};
    QLabel *storageFreeSummary_{nullptr};
    QLabel *inferenceState_{nullptr};
    QLabel *inferenceDot_{nullptr};
    QLabel *inferencePerformanceSummary_{nullptr};
    QLabel *inferenceDetectionSummary_{nullptr};
    QLabel *inferenceDiagnostics_{nullptr};
    QPushButton *startInferenceButton_{nullptr};
    QPushButton *stopInferenceButton_{nullptr};
    QPushButton *visionRefreshStatusButton_{nullptr};
    QLabel *visionControlState_{nullptr};
    ui::ElidedLabel *visionControlMessage_{nullptr};
    vision::VideoView *videoView_{nullptr};
    QPushButton *snapshotButton_{nullptr};
    QPushButton *startRecordingButton_{nullptr};
    QPushButton *stopRecordingButton_{nullptr};
    QLabel *captureState_{nullptr};
    QLabel *captureCountSummary_{nullptr};
    QLabel *captureDiagnostics_{nullptr};
    QLineEdit *inferenceArtifactValue_{nullptr};
    QLineEdit *inferenceShaValue_{nullptr};
    QLabel *inferenceOperationValue_{nullptr};
    QLabel *inferenceThresholdValue_{nullptr};
    QLabel *inferenceLastErrorValue_{nullptr};
    QLabel *visionEndpointDetails_{nullptr};
    QLabel *controlResponseDetails_{nullptr};
    QLabel *protocolCountersDetails_{nullptr};
    QString lastControlResponseText_;
    QString lastControlResponseDetail_;
    bool haveVisionReceiveSample_{false};
    quint64 visionJpegDecodeErrors_{0};
    std::optional<vision::DetectionFrame> latestDetectionFrame_;
    bool detectionConnectAttempted_{false};
    QString detectionAttemptHost_;
    quint16 detectionAttemptPort_{0};

    // Connection bar.
    QComboBox *serialPortCombo_{nullptr};
    QSpinBox *serialBaud_{nullptr};
    QSpinBox *robotTcpPort_{nullptr};
    QPushButton *connectButton_{nullptr};
    QPushButton *acquireButton_{nullptr};
    QPushButton *releaseButton_{nullptr};
    QLabel *connectionStatus_{nullptr};
    QLabel *authorityStatus_{nullptr};
    QPushButton *enableAllButton_{nullptr};
    QPushButton *emergencyStopButton_{nullptr};

    // Leak card.
    QLabel *leakStatus_{nullptr};
    QLabel *leakDot_{nullptr};

    // Motion card.
    std::array<QPushButton *, static_cast<std::size_t>(MotionMode::Count)> motionButtons_{};
    QPushButton *motionStopButton_{nullptr};
    QLabel *motionStatus_{nullptr};
    QComboBox *gaitBackendCombo_{nullptr};
    QLabel *gaitBackendStatus_{nullptr};
    QPushButton *disableAllButton_{nullptr};

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
    std::array<QPushButton *, kServoCount> enableButtons_{};
    std::array<QPushButton *, kServoCount> neutralButtons_{};
    std::array<QPushButton *, kServoCount> applyButtons_{};
    std::array<QPushButton *, kServoCount> angleApplyButtons_{};
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

    // Task 03 presentation tree. These own no new runtime behavior.
    QSplitter *workspaceSplitter_{nullptr};
    QTabWidget *operatorToolsTabs_{nullptr};
    QWidget *telemetrySidebar_{nullptr};
    QWidget *operatorToolsPane_{nullptr};
    QWidget *videoCard_{nullptr};
    QWidget *videoContentHost_{nullptr};
    QWidget *visionSummaryPanel_{nullptr};
    QSize visionFrameSize_{};
    bool videoGeometryUpdatePending_{false};
    QScrollArea *motionScrollArea_{nullptr};
    QScrollArea *actuatorScroll_{nullptr};
    QWidget *actuatorCardsHost_{nullptr};
    QGridLayout *actuatorGrid_{nullptr};
    QTabWidget *dataPlotTabs_{nullptr};
    std::array<QWidget *, kServoCount> servoPanels_{};
    int actuatorColumnCount_{0};
    bool actuatorReflowPending_{false};
    bool workspaceSizeInitPending_{false};
};

} // namespace rb
