#include "ui/MainWindow.h"
#include "vision/VisualDispatchSession.h"
#include "vision/VisualDiagnosticSession.h"
#include "helpers/VisualControllerFixture.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QSettings>
#include <QSpinBox>
#include <QKeyEvent>
#include <QTemporaryDir>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QElapsedTimer>
#include <cstdio>
namespace {
int failures = 0;
void check(bool condition, const char *why) { if (!condition) { ++failures; std::fprintf(stderr, "FAIL: %s\n", why); } }
void frame(rb::MainWindow &window, quint64 id, int area) {
    rb::vision::DetectionFrame f{id,1000+id,{640,480},{{1,"fish",.95,{320,240}}}};
    f.detections.front().areaCells = area;
    window.findChild<rb::vision::VisualDiagnosticSession *>()->onDetectionArrival(f,{rb::vision::DetectionDisplayState::Target,f});
}
void testDryRunAndSelectors() {
    rb::test::VisualControllerFixture controller;
    controller.connected = controller.active = false;
    rb::MainWindow window(&controller);
    auto *exec = window.findChild<QComboBox *>("visualExecutionMode");
    auto *dry = window.findChild<QCheckBox *>("visualDryRun");
    auto *gate = window.findChild<QCheckBox *>("visualDispatchEnabled");
    auto *arm = window.findChild<QPushButton *>("visualArmButton");
    auto *session = window.findChild<rb::vision::VisualDispatchSession *>();
    check(exec && dry && session, "compact execution controls exist");
    if (!exec || !dry || !session) return;
    check(exec->currentIndex()==0 && !dry->isChecked() && !dry->isEnabled(), "execution defaults discrete with Dry run off");
    exec->setCurrentIndex(1); dry->setChecked(true); gate->setChecked(true);
    frame(window,1,24); session->timerTick(); arm->click();
    check(session->armed(), "desktop Dry run arms without hardware");
    check(!exec->isEnabled() && !dry->isEnabled(), "armed selectors lock");
    check(window.findChild<QLabel *>("autoFollowStatePill")->text().contains("Arrived"), "large target enters Arrived without starting");
    check(controller.sends.empty() && controller.manualRecords.empty(), "Dry run UI submits no robot control writes");
    check(window.findChild<QPushButton *>("gamepadConfigButton")->isEnabled()==false, "visual tuning locks while armed");
    arm->click();
    check(exec->isEnabled() && dry->isEnabled(), "disarm unlocks selectors");
    check(window.findChild<QPushButton *>("gamepadConfigButton")->isEnabled(), "visual tuning available without hardware while disarmed");
    window.resize(1280,window.fullyExpandedWindowHeight());
    check(window.fullyExpandedWindowHeight()<=992,"1280-wide expanded height fits 992 pixels");
}
class ProportionalFixture : public rb::test::VisualControllerFixture {
public:
    int starts{0},stops{0},streams{0}; bool prop{false};
    ProportionalFixture() { enabled=known=0x1f; }
    bool hasCpgSchema2() const override { return true; }
    quint8 cpgFeatureLevel() const override { return 2; }
    bool proportionalActive() const override { return prop; }
    std::optional<quint32> submitVisualProportionalStart(const ProportionalConfig &) override {
        ++starts; auto id=next++;
        QTimer::singleShot(0,this,[this,id] { prop=true; motionActive=true; emit motionStateChanged(rb::MotionState::Running,rb::MotionMode::Forward); ack(id,rb::CommandTerminalResult::Ok); });
        return id;
    }
    std::optional<quint32> submitVisualProportionalStop() override { ++stops; prop=false; motionActive=false; return next++; }
    bool setProportionalInput(quint16,qint16,qint16) override { ++streams; return prop; }
};
void testVisualOwnershipAndSingleManualStop() {
    ProportionalFixture controller;
    rb::MainWindow window(&controller);
    auto *session=window.findChild<rb::vision::VisualDispatchSession *>();
    session->setTimerEnabled(false);
    window.findChild<QComboBox *>("gamepadModeCombo")->setCurrentIndex(1);
    window.findChild<QComboBox *>("visualExecutionMode")->setCurrentIndex(1);
    window.findChild<QCheckBox *>("visualDispatchEnabled")->setChecked(true);
    frame(window,1,6); session->timerTick();
    window.findChild<QPushButton *>("visualArmButton")->click();
    QApplication::processEvents(); session->timerTick();
    check(session->armed() && controller.starts==1 && controller.prop,"real visual session owns its proportional start");
    QObject::disconnect(window.findChild<rb::vision::VisualDiagnosticSession *>(),nullptr,session,nullptr);
    const int before=controller.streams;
    QElapsedTimer wait; wait.start();
    while(wait.elapsed()<60) QApplication::processEvents();
    check(session->armed() && controller.stops==0 && controller.streams==before,"disabled gamepad neither stops nor streams visual proportional session");
    window.findChild<QPushButton *>("motionStopButton")->click();
    check(!session->armed() && controller.stops==1 && controller.manualRecords.empty(),"manual STOP submits exactly one visual proportional STOP");
    check(session->stopAwaiting() && !window.findChild<QComboBox *>("visualExecutionMode")->isEnabled(),"pending STOP keeps execution selector locked");
}
void testIndependentTuningAndReload() {
    QSettings settings;
    settings.setValue("Gamepad/Proportional/v1/maxScale",700);
    rb::test::VisualControllerFixture controller;
    controller.active=controller.connected=false;
    {
        rb::MainWindow window(&controller);
        window.findChild<QComboBox *>("visualExecutionMode")->setCurrentIndex(1);
        window.findChild<QCheckBox *>("visualDryRun")->setChecked(true);
        QTimer::singleShot(0,&window,[&] {
            auto *dialog=window.findChild<QDialog *>();
            check(dialog!=nullptr,"visual tuning opens dialog");
            if (!dialog) return;
            dialog->findChild<QDoubleSpinBox *>("visualApproachParameter3")->setValue(25);
            dialog->findChild<QDoubleSpinBox *>("visualApproachParameter11")->setValue(.8);
            dialog->findChild<QDialogButtonBox *>()->button(QDialogButtonBox::Ok)->click();
        });
        window.findChild<QPushButton *>("gamepadConfigButton")->click();
        auto *session=window.findChild<rb::vision::VisualDispatchSession *>();
        check(session->approachConfig().stopArea==25 && session->proportionalConfig().max_scale==800,"visual tuning applies independent area and Q");
        check(settings.value("Gamepad/Proportional/v1/maxScale").toInt()==700,"visual tuning preserves gamepad settings");
        check(controller.sends.empty() && controller.manualRecords.empty(),"tuning submits no control writes");
    }
    rb::MainWindow reopened(&controller);
    auto *session=reopened.findChild<rb::vision::VisualDispatchSession *>();
    check(session->approachConfig().stopArea==25 && session->proportionalConfig().max_scale==800,"visual approach settings reload independently");
    check(session->executionMode()==rb::vision::VisualExecutionMode::Discrete && !session->dryRun(),"Exec and Dry run never persist");
}

void testFollowThresholdsAndPersistence() {
    rb::test::VisualControllerFixture controller;
    rb::MainWindow window(&controller);
    window.findChild<QComboBox *>("visualExecutionMode")->setCurrentIndex(1);
    auto *follow = window.findChild<QSpinBox *>("visualFollowArea");
    check(follow != nullptr, "Follow area shortcut exists");
    if (!follow) return;
    follow->setValue(21);
    auto *session = window.findChild<rb::vision::VisualDispatchSession *>();
    check(session->approachConfig().stopArea != 21, "typing Follow does not apply thresholds");
    auto *set = window.findChild<QPushButton *>("visualFollowSet");
    check(set != nullptr, "Follow has explicit Set button");
    if (!set) return;
    set->click();
    const auto &config = session->approachConfig();
    QSettings settings;
    check(config.stopArea == 21 && qAbs(config.slowArea - 8.4) < 1e-9
              && qAbs(config.resumeArea - 14.7) < 1e-9,
          "Follow changes all three shared thresholds proportionally");
    check(settings.value("Visual/Approach/v1/stopArea").toDouble() == 21
              && qAbs(settings.value("Visual/Approach/v1/slowArea").toDouble() - 8.4) < 1e-9
              && qAbs(settings.value("Visual/Approach/v1/resumeArea").toDouble() - 14.7) < 1e-9,
          "Follow persists thresholds using existing visual configuration keys");
    window.show();
    follow->setFocus();
    QApplication::processEvents();
    follow->setValue(23);
    session->setApproachConfig(session->approachConfig());
    check(follow->value() == 23 && session->approachConfig().stopArea == 21,
          "refresh preserves focused draft without applying it");
    QKeyEvent enter(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
    QApplication::sendEvent(follow, &enter);
    check(session->approachConfig().stopArea == 23, "Enter commits focused Follow input");
}
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    QTemporaryDir settings;
    QCoreApplication::setOrganizationName("RoboBeetleVisualApproachTests");
    QCoreApplication::setApplicationName("Isolated");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat,QSettings::UserScope,settings.path());
    testDryRunAndSelectors();
    testIndependentTuningAndReload();
    testVisualOwnershipAndSingleManualStop();
    testFollowThresholdsAndPersistence();
    return failures ? 1 : 0;
}
