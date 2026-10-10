#pragma once

#include "controller/IConsoleController.h"
#include "input/GamepadMapper.h"
#include "input/XInputGamepad.h"
#include <QElapsedTimer>
#include "vision/DetectionMetadata.h"

#include <QMainWindow>
#include <QSize>

#include <array>
#include <cstddef>
#include <optional>

class QCloseEvent;
class QShowEvent;
class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QButtonGroup;
class QProgressBar;
class QPushButton;
class QSlider;
class QSpinBox;
class QSplitter;
class QTabWidget;
class QVBoxLayout;
class QWidget;
class QEvent;
class QGridLayout;
class QScrollArea;

#include <QList>

namespace rb {

namespace ui {
class ElidedLabel;
}

namespace vision {
class DetectionClient;
class VideoView;
class VisionClient;
class VisionControlClient;
class VisualDiagnosticSession;
class VisualCsvLogger;
class VisualDispatchSession;
struct VisualViewContext;
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
    ~MainWindow() override;
    // Height the Operator tools pane needs so that no tab has to scroll for its
    // preferred content (action bar + tab bar + tallest page). The pane never
    // gets less than this; the video/dashboard area gives way first.
    [[nodiscard]] int operatorToolsRequiredHeight() const;
    // Smallest window height that honours operatorToolsRequiredHeight() while
    // the dashboard keeps its own minimum. This is also the default height.
    [[nodiscard]] int fullyExpandedWindowHeight() const;
    // True when the screen cannot hold fullyExpandedWindowHeight(); main() then
    // shows the window maximized. The window itself keeps a plain resize so
    // callers (tests, previews) stay in control of the geometry.
    [[nodiscard]] bool startupWantsMaximized() const noexcept { return startupMaximize_; }
    // Re-evaluates minimum/default geometry with real (shown) size hints. main()
    // calls it after one invisible show so the first visible frame is final.
    void settleStartupGeometry();

protected:
    void closeEvent(QCloseEvent *event) override;
    void showEvent(QShowEvent *event) override;
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
    // Task 06 green region (everything after the status text + its stretch).
    void createGreenRegion(QVBoxLayout *summaryLayout, QGroupBox *summary);
    void applyAutoFollowCondensation();
    void refreshAutoFollowUi();
    void refreshDepthControlUi();
    void onAxisSelected(int axisIndex);
    void bindVisionUi();
    void refreshVisualDiagnosticTexts();
    void refreshVisualCsvUi();
    void refreshVisualDispatchUi();
    void bindControllerUi();
    void refreshVideoDiagnosticsUi();
    void refreshProtocolUi(const ProtocolMonitor &monitor);
    void refreshVisionNoticeUi();
    void refreshVisionEndpointUi();
    void syncDetectionStream();
    void refreshDetectionOverlay();
    [[nodiscard]] vision::VisualViewContext visualDisplayContext() const;
    void resetDetectionSession();
    void reflowActuatorCards();
    void updateVideoSurfaceGeometry();
    void initializeWorkspaceSizes();
    void updateWorkspaceMinimums();
    void reserveVisionPanelHeight();
    void applyStartupGeometry();
    // Refresh helpers.
    void setConnectedUi(bool connected);
    void refreshServoUi(int index);
    void setAngleUiEnabled(int index, bool enabled);
    void setLeakUiState(LeakState state);
    void setImuUiState(const ImuMonitorState &state);
    void setDepthUiState(const DepthMonitorState &state);
    void refreshMotionUi();
    void refreshGaitSelectorsUi();
    void refreshAuthorityUi();
    void pollGamepad();
    void disableGamepad();
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
    vision::VisualDiagnosticSession *visualSession_{nullptr};
    quint64 visualSessionId_{0};
    vision::VisualDispatchSession *visualDispatch_{nullptr};
    QCheckBox *visualDispatchEnabled_{nullptr};
    QLabel *visualDispatchStatus_{nullptr};
    // Task 06 green region. Sub-region A keeps the frozen vision actions;
    // sub-region B is the new Auto Follow block.
    QGroupBox *visionControlsGroup_{nullptr};
    QGroupBox *autoFollowGroup_{nullptr};
    QGridLayout *visionControlsGrid_{nullptr};
    QGridLayout *autoFollowGrid_{nullptr};
    QGridLayout *greenRegionGrid_{nullptr};
    QLabel *autoFollowStatePill_{nullptr};
    QLabel *autoFollowChecklist_{nullptr};
    QLabel *autoFollowAlert_{nullptr};
    QLabel *autoFollowDetail_{nullptr};
    QLabel *autoFollowFooter_{nullptr};
    QWidget *autoFollowAxisSelector_{nullptr};
    // Depth Sensor card: zeroed-depth bar and Zero action (wired by the follow-up PR).
    QProgressBar *depthBar_{nullptr};
    QPushButton *depthZeroButton_{nullptr};
    QWidget *connectionBar_{nullptr};
    bool startupMaximize_{false};
    QPushButton *visualArmButton_{nullptr};
    QPushButton *visualDisarmButton_{nullptr};
    int autoFollowMissing_{3};
    int autoFollowTotal_{3};
    int autoFollowChecklistLines_{3};
    QString autoFollowDepthNote_;
    QButtonGroup *axisGroup_{nullptr};
    rb::DepthEnvelopeMemory depthUiMemory_;
    QString autoFollowStateText_;
    QString autoFollowChecklistFull_;
    QString autoFollowChecklistTip_;
    QString autoFollowDetailText_;
    QString autoFollowBlockedText_; // all items ready but arm() would still refuse
    bool autoFollowCondensed_{false};
    bool autoFollowChecklistWanted_{false};
    QList<QWidget *> autoFollowPinned_;
    QString visualDispatchRejection_;
    QString visualOperatorStopStatus_;
    QLabel *visualProposalDetails_{nullptr};
    vision::VisualCsvLogger *visualCsvLogger_{nullptr};
    QCheckBox *visualCsvEnabled_{nullptr};
    QLineEdit *visualCsvDirectory_{nullptr};
    QPushButton *visualCsvBrowse_{nullptr};
    QLabel *visualCsvStatus_{nullptr};

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
    QLabel *visionDiagnosticScreen_{nullptr};
    QLabel *visionDot_{nullptr};
    QLabel *visionDiagnostics_{nullptr};
    QLabel *videoFpsSummary_{nullptr};
    QLabel *videoResolutionSummary_{nullptr};
    QLabel *storageFreeSummary_{nullptr};
    QLabel *inferenceState_{nullptr};
    QLabel *inferenceDot_{nullptr};
    QLabel *inferencePerformanceSummary_{nullptr};
    QLabel *inferenceDetectionSummary_{nullptr};
    QLabel *inferenceMemorySummary_{nullptr};
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
    QPushButton *gamepadToggleButton_{nullptr};
    QLabel *gamepadStatusLabel_{nullptr};
    GamepadMapper gamepadMapper_;
    XInputGamepad gamepadReader_;
    DepthEnvelopeMemory gamepadDepthMemory_;
    QElapsedTimer gamepadClock_;
    std::optional<bool> gamepadPreviousAccepted_;
    bool pollingGamepad_{false};
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
    QComboBox *frontRearCoordinationCombo_{nullptr};
    QLabel *frontRearCoordinationStatus_{nullptr};
    QPushButton *disableAllButton_{nullptr};

    // IMU card.
    QLabel *imuDot_{nullptr};
    QLabel *imuAcc_{nullptr};
    QLabel *imuGyro_{nullptr};
    QLabel *imuAngle_{nullptr};

    // Depth card.
    QLabel *depthDot_{nullptr};
    QLabel *depthValue_{nullptr};
    QLabel *depthTemperature_{nullptr};
    QLabel *depthAge_{nullptr};

    // Telemetry details tab (holds the long IMU/Depth diagnostics text).
    QLabel *imuDiagnostics_{nullptr};
    QLabel *motionTelemetryDiagnostics_{nullptr};
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
