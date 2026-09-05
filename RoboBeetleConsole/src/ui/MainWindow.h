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
    QWidget *createMonitorPanel();
    void setConnectedUi(bool connected);
    void setAngleUiEnabled(int index, bool enabled);
    void appendLog(const QString &message);
    static QString stateText(TransportState state);

    RobotController *controller_;
    QComboBox *portCombo_{nullptr};
    QSpinBox *baudSpin_{nullptr};
    QPushButton *connectButton_{nullptr};
    QLabel *connectionStatus_{nullptr};
    std::array<QSpinBox *, 2> pwmSpins_{};
    std::array<QSlider *, 2> pwmSliders_{};
    std::array<QDoubleSpinBox *, 2> angleSpins_{};
    std::array<QPushButton *, 2> angleButtons_{};
    std::array<QPushButton *, 2> enableButtons_{};
    std::array<QPushButton *, 2> neutralButtons_{};
    std::array<QPushButton *, 2> applyButtons_{};
    QLineEdit *txHex_{nullptr};
    QLineEdit *rxHex_{nullptr};
    QLabel *txCount_{nullptr};
    QLabel *rxCount_{nullptr};
    QLabel *crcCount_{nullptr};
    QLabel *timeoutCount_{nullptr};
    QLabel *ackStatus_{nullptr};
    QPlainTextEdit *log_{nullptr};
};

} // namespace rb

