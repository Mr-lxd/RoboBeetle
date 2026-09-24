#include "remote/RemoteRobotController.h"
#include "ui/MainWindow.h"

#include "robobeetle/gateway/gateway_types.hpp"
#include "robobeetle/gateway/rbrp_codec.hpp"

#include <QApplication>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QHostAddress>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>

#include <cstdio>
#include <functional>
#include <optional>
#include <vector>

namespace {

using namespace robobeetle::gateway;

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool pumpUntil(const std::function<bool()> &predicate, int timeoutMs = 1500)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QApplication::processEvents(QEventLoop::AllEvents, 10);
        if (predicate()) {
            return true;
        }
        QThread::msleep(1);
    }
    QApplication::processEvents(QEventLoop::AllEvents, 10);
    return predicate();
}

QByteArray wireBytes(const Bytes &bytes)
{
    return QByteArray(reinterpret_cast<const char *>(bytes.data()),
                      static_cast<qsizetype>(bytes.size()));
}

class FakeGatewayPeer {
public:
    FakeGatewayPeer()
    {
        expect(server_.listen(QHostAddress::LocalHost, 0),
               "fake gateway must listen");
    }

    quint16 port() const { return server_.serverPort(); }

    bool accept()
    {
        if (!pumpUntil([this] { return server_.hasPendingConnections(); })) {
            return false;
        }
        peer_ = server_.nextPendingConnection();
        decoder_.reset();
        queued_.clear();
        return peer_ != nullptr;
    }

    std::optional<RbrpFrame> nextFrame(RbrpMessageKind kind,
                                       int timeoutMs = 1500)
    {
        std::optional<RbrpFrame> result;
        const bool found = pumpUntil([&] {
            for (auto it = queued_.begin(); it != queued_.end(); ++it) {
                if (it->kind == kind) {
                    result = *it;
                    queued_.erase(it);
                    return true;
                }
            }
            if (peer_ == nullptr || peer_->bytesAvailable() == 0) {
                return false;
            }
            const QByteArray bytes = peer_->readAll();
            std::vector<RbrpFrame> decoded;
            const auto status = decoder_.feed(
                reinterpret_cast<const Byte *>(bytes.constData()),
                static_cast<std::size_t>(bytes.size()), decoded);
            expect(status == RbrpFeedStatus::Ok,
                   "fake gateway must decode controller traffic");
            queued_.insert(queued_.end(), decoded.begin(), decoded.end());
            return false;
        }, timeoutMs);
        return found ? result : std::nullopt;
    }

    void send(const GatewayMessage &message)
    {
        const auto encoded = encode_gateway_message(message);
        expect(encoded.status == RbrpEncodeStatus::Ok,
               "fake gateway response must encode");
        if (peer_ == nullptr || encoded.status != RbrpEncodeStatus::Ok) {
            return;
        }
        peer_->write(wireBytes(encoded.wire));
        peer_->flush();
        QApplication::processEvents(QEventLoop::AllEvents, 10);
    }

    void disconnectPeer()
    {
        if (peer_ == nullptr) {
            return;
        }
        peer_->disconnectFromHost();
        (void)pumpUntil(
            [this] { return peer_->state() == QAbstractSocket::UnconnectedState; });
        peer_->deleteLater();
        peer_ = nullptr;
        decoder_.reset();
        queued_.clear();
    }

private:
    QTcpServer server_;
    QTcpSocket *peer_{nullptr};
    RbrpDecoder decoder_;
    std::vector<RbrpFrame> queued_;
};

GatewayMessage helloReply(RequestId requestId)
{
    HelloReply reply;
    reply.server_capabilities = 0;
    reply.max_payload = 512;
    reply.control_heartbeat_interval_ms = 250;
    reply.authority_lease_timeout_ms = 1000;
    return {requestId, reply};
}

GatewayMessage acquireReply(RequestId requestId)
{
    AcquireReply reply;
    reply.result = AcquireResult::Granted;
    reply.authority_state = AuthorityState::Owned;
    reply.session_state = GatewayApplicationSessionState::SafetyQuiet;
    reply.link_state = GatewayApplicationLinkState::Unconfirmed;
    reply.lease_timeout_ms = 1000;
    return {requestId, reply};
}

GatewayMessage activeState()
{
    ControlStateMessage state;
    state.authority_state = AuthorityState::Owned;
    state.session_state = GatewayApplicationSessionState::Online;
    state.link_state = GatewayApplicationLinkState::Active;
    state.reason = GatewayStateReason::Acquired;
    state.lease_remaining_ms = 900;
    return {0, state};
}

void sendSubmitted(FakeGatewayPeer &gateway,
                   RequestId requestId, quint16 sequence)
{
    CommandSubmittedMessage submitted;
    submitted.status = CommandSubmittedStatus::Submitted;
    submitted.sequence = sequence;
    gateway.send({requestId, submitted});
}

void sendOutcome(FakeGatewayPeer &gateway, RequestId requestId,
                 RobotCommandKind kind, quint16 sequence,
                 GatewayCommandOutcome outcome)
{
    GatewayCommandOutcomeMessage message;
    message.command_kind = kind;
    message.event.outcome = outcome;
    message.event.sequence = sequence;
    message.event.result = 0;
    gateway.send({requestId, message});
}

QPushButton *buttonWithText(const QWidget *root, const QString &text)
{
    for (QPushButton *button : root->findChildren<QPushButton *>()) {
        if (button->text() == text) {
            return button;
        }
    }
    return nullptr;
}

QGroupBox *groupWithTitle(const QWidget *root, const QString &title)
{
    for (QGroupBox *box : root->findChildren<QGroupBox *>()) {
        if (box->title() == title) {
            return box;
        }
    }
    return nullptr;
}

void completeHello(FakeGatewayPeer &gateway)
{
    const auto hello = gateway.nextFrame(RbrpMessageKind::Hello);
    expect(hello.has_value(), "remote controller must send Hello");
    if (hello.has_value()) {
        gateway.send(helloReply(hello->request_id));
    }
}

void completeAcquire(FakeGatewayPeer &gateway)
{
    const auto acquire = gateway.nextFrame(RbrpMessageKind::AcquireControl);
    expect(acquire.has_value(), "UI Acquire must emit AcquireControl");
    if (acquire.has_value()) {
        gateway.send(acquireReply(acquire->request_id));
        gateway.send(activeState());
    }
}

void testRemoteControllerAndUi()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::MainWindow window(&controller);

    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = gateway.port();
    controller.connectController(config);

    expect(gateway.accept(), "remote controller must connect to fake gateway");
    QPushButton *acquire = buttonWithText(&window, QStringLiteral("Acquire"));
    QPushButton *release = buttonWithText(&window, QStringLiteral("Release"));
    expect(acquire != nullptr && release != nullptr,
           "remote MainWindow must expose Acquire and Release");
    expect(acquire != nullptr && !acquire->isEnabled(),
           "Acquire must remain disabled until HelloReply completes");

    completeHello(gateway);
    expect(pumpUntil([&] {
        return controller.isConnected()
            && controller.authorityState() == rb::ControlAuthorityState::Unowned;
    }), "Hello must leave UI connected but Unowned");

    expect(acquire != nullptr && acquire->isEnabled(),
           "Acquire must enable after Hello while Unowned");

    if (acquire != nullptr) {
        acquire->click();
    }
    completeAcquire(gateway);
    expect(pumpUntil([&] { return controller.isControlActive(); }),
           "Acquire + Active state must enable remote control");
    expect(release != nullptr && release->isEnabled(),
           "Release must enable while remote authority is owned");

    QWidget *frontRight =
        window.findChild<QWidget *>(QStringLiteral("servoStatusRow0"));
    expect(frontRight != nullptr,
           "MainWindow must retain the FrontRight actuator status row");
    if (frontRight == nullptr) {
        return;
    }
    expect(frontRight->findChildren<QPushButton *>().isEmpty(),
           "Actuator status rows must not contain control buttons");

    QPushButton *enable =
        window.findChild<QPushButton *>(QStringLiteral("servoEnableButton0"));
    QPushButton *applyPwm =
        window.findChild<QPushButton *>(QStringLiteral("servoApplyButton0"));
    expect(enable != nullptr && enable->isEnabled(),
           "FrontRight Enable must be actionable when remote control is Active");
    expect(applyPwm != nullptr && !applyPwm->isEnabled(),
           "raw PWM remains gated until the servo is enabled");

    QPushButton *backward =
        buttonWithText(&window, QStringLiteral("Brake"));
    expect(backward != nullptr && !backward->isEnabled(),
           "Brake remains disabled because the underlying backward command is not bench-startable");

    if (enable != nullptr) {
        enable->click();
    }
    const auto enableRequest =
        gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(enableRequest.has_value(),
           "Enable button must emit typed RBRP CommandRequest");
    if (!enableRequest.has_value()) {
        return;
    }
    expect(enableRequest->payload == Bytes({0x01U, 0x01U, 0x00U}),
           "FrontRight Enable must map to EnableServos mask 0x0001");

    sendSubmitted(gateway, enableRequest->request_id, 42U);
    QApplication::processEvents();
    expect(!controller.isServoEnabled(rb::ServoId::FrontRight),
           "CommandSubmitted must not prematurely confirm Servo Enable");

    sendOutcome(gateway, enableRequest->request_id,
                RobotCommandKind::EnableServos, 42U,
                GatewayCommandOutcome::Accepted);
    expect(pumpUntil([&] {
        return controller.isServoEnabled(rb::ServoId::FrontRight);
    }), "Accepted CommandOutcome must confirm Servo Enable");

    expect(enable != nullptr && enable->text() == QStringLiteral("Release"),
           "Servo Fine Control must reflect the accepted remote Enable outcome");
    auto *servoStatusDot =
        frontRight->findChild<QLabel *>(QStringLiteral("servoStatusDot0"));
    auto *servoStatusLabel =
        frontRight->findChild<QLabel *>(QStringLiteral("servoStatusLabel0"));
    expect(servoStatusDot != nullptr
               && servoStatusDot->styleSheet().contains(QStringLiteral("#2F80ED"))
               && servoStatusLabel != nullptr
               && servoStatusLabel->styleSheet().contains(QStringLiteral("#2F80ED")),
           "accepted Servo Enable is highlighted in blue in Actuator Control");

    expect(applyPwm != nullptr && applyPwm->isEnabled(),
           "raw PWM becomes actionable after accepted remote Enable");
    expect(controller.setServoPwm(rb::ServoId::FrontRight, 1500),
           "Remote controller must submit raw PWM through RBRP");
    const auto pwmRequest =
        gateway.nextFrame(RbrpMessageKind::CommandRequest, 120);
    expect(pwmRequest.has_value(),
           "raw PWM must emit one remote CommandRequest");
    if (pwmRequest.has_value()) {
        expect(pwmRequest->payload == Bytes({0x08U, 0x00U, 0xdcU, 0x05U}),
               "FrontRight 1500 us must encode SetServoPwm with servo ID and uint16 LE pulse");
        sendSubmitted(gateway, pwmRequest->request_id, 43U);
        sendOutcome(gateway, pwmRequest->request_id,
                    RobotCommandKind::SetServoPwm, 43U,
                    GatewayCommandOutcome::Accepted);
        QApplication::processEvents();
    }

    GatewayLeakTelemetry leak;
    leak.sequence = 100;
    leak.state = robobeetle::gateway::LeakState::Dry;
    gateway.send({0, leak});

    GatewayImuTelemetry imu;
    imu.sequence = 101;
    imu.schema_version = 1;
    imu.validity_flags = 0x07;
    imu.acc_mg = {1000, -2000, 0};
    imu.gyro_tenth_dps = {10, -20, 0};
    imu.angle_centidegrees = {300, -400, 500};
    gateway.send({0, imu});

    GatewayDepthTelemetry depth;
    depth.sequence = 102;
    depth.schema_version = 1;
    depth.validity_flags = 0x03;
    depth.depth_mm = 1234;
    depth.temperature_centi_c = 2534;
    depth.sample_age_ms = 25;
    gateway.send({0, depth});

    expect(pumpUntil([&] {
        return controller.leakState() == rb::LeakState::Dry
            && controller.imuState().status == rb::ImuStatus::Receiving
            && controller.depthState().status == rb::DepthStatus::Receiving;
    }), "typed RBRP Leak/IMU/Depth telemetry must map to Console state");
    expect(controller.imuState().snapshot.has_value()
               && controller.imuState().snapshot->accMg[0] == 1000,
           "IMU telemetry values must survive remote mapping");
    expect(controller.depthState().snapshot.has_value()
               && controller.depthState().snapshot->depthMm == 1234,
           "Depth telemetry values must survive remote mapping");

    gateway.disconnectPeer();
    expect(pumpUntil([&] { return !controller.isConnected(); }),
           "network loss must be detected by remote controller");
    expect(controller.authorityState() == rb::ControlAuthorityState::Unowned,
           "network loss must clear local authority");
    expect(!controller.isServoEnabled(rb::ServoId::FrontRight),
           "network loss must clear stale Servo enabled knowledge");

    controller.connectController(config);
    expect(gateway.accept(), "reconnect must establish a new TCP generation");
    const auto replacementHello =
        gateway.nextFrame(RbrpMessageKind::Hello);
    expect(replacementHello.has_value(),
           "reconnect must start with a new Hello");
    expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest, 120).has_value(),
           "reconnect must not replay old actuator commands");

    if (replacementHello.has_value()) {
        gateway.send(helloReply(replacementHello->request_id));
    }
    expect(pumpUntil([&] {
        return controller.authorityState() == rb::ControlAuthorityState::Unowned;
    }), "reconnected generation must require explicit Acquire again");
    expect(!controller.isControlActive(),
           "reconnect must not automatically restore Active control");
}

void testUserReleaseDoesNotReportAuthorityLoss()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    QStringList logs;
    QObject::connect(&controller, &rb::IConsoleController::logMessage,
                     &controller, [&logs](const QString &message) {
        logs.push_back(message);
    });

    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = gateway.port();
    controller.connectController(config);

    expect(gateway.accept(), "release-log test must connect to fake gateway");
    completeHello(gateway);
    expect(pumpUntil([&] { return controller.canAcquireControl(); }),
           "release-log test must complete Hello");
    expect(controller.acquireControl(),
           "release-log test must submit AcquireControl");
    completeAcquire(gateway);
    expect(pumpUntil([&] { return controller.isControlActive(); }),
           "release-log test must reach Active control");

    expect(controller.releaseControl(),
           "explicit user Release must submit ReleaseControl");
    expect(gateway.nextFrame(RbrpMessageKind::ReleaseControl).has_value(),
           "gateway must receive explicit ReleaseControl");
    expect(controller.authorityState() == rb::ControlAuthorityState::Unowned
               && !controller.isControlActive(),
           "explicit Release must clear local authority immediately");

    bool sawRelease = false;
    bool sawLoss = false;
    for (const QString &message : logs) {
        sawRelease = sawRelease
            || message.contains(QStringLiteral("Remote control released"));
        sawLoss = sawLoss
            || message.contains(QStringLiteral("remote authority/link lost"));
    }
    expect(sawRelease,
           "explicit Release must emit a normal release log entry");
    expect(!sawLoss,
           "explicit Release must not be reported as authority/link loss");
}

void testSafetySupersessionIgnoresLateOutcomes()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = gateway.port();
    controller.connectController(config);

    expect(gateway.accept(), "supersession test must connect to fake gateway");
    completeHello(gateway);
    expect(pumpUntil([&] { return controller.canAcquireControl(); }),
           "supersession test must complete Hello");
    expect(controller.acquireControl(),
           "supersession test must submit AcquireControl");
    completeAcquire(gateway);
    expect(pumpUntil([&] { return controller.isControlActive(); }),
           "supersession test must reach Active control");

    expect(controller.enableServo(rb::ServoId::FrontRight),
           "pending Enable must submit before safety Disable");
    const auto pendingEnable =
        gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(pendingEnable.has_value(),
           "gateway must receive pending Enable request");
    if (!pendingEnable.has_value()) {
        return;
    }
    sendSubmitted(gateway, pendingEnable->request_id, 60U);

    expect(controller.disableServo(rb::ServoId::FrontRight),
           "Disable must supersede unresolved Enable for the same servo");
    const auto safetyDisable =
        gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(safetyDisable.has_value(),
           "gateway must receive safety Disable request");
    if (!safetyDisable.has_value()) {
        return;
    }
    sendSubmitted(gateway, safetyDisable->request_id, 61U);
    sendOutcome(gateway, safetyDisable->request_id,
                RobotCommandKind::DisableServos, 61U,
                GatewayCommandOutcome::Accepted);
    expect(pumpUntil([&] {
        return !controller.isServoDisablePending(rb::ServoId::FrontRight);
    }), "accepted Disable must clear its pending state");

    sendOutcome(gateway, pendingEnable->request_id,
                RobotCommandKind::EnableServos, 60U,
                GatewayCommandOutcome::Accepted);
    QApplication::processEvents();
    expect(!controller.isServoEnabled(rb::ServoId::FrontRight),
           "late superseded Enable outcome must never resurrect enabled state");

    const rb::ServoId paddles[] = {
        rb::ServoId::FrontRight,
        rb::ServoId::FrontLeft,
        rb::ServoId::RearRight,
        rb::ServoId::RearLeft,
    };
    quint16 sequence = 70U;
    for (const rb::ServoId id : paddles) {
        expect(controller.enableServo(id),
               "motion setup Enable must submit");
        const auto request =
            gateway.nextFrame(RbrpMessageKind::CommandRequest);
        expect(request.has_value(),
               "gateway must receive motion setup Enable");
        if (!request.has_value()) {
            return;
        }
        sendSubmitted(gateway, request->request_id, sequence);
        sendOutcome(gateway, request->request_id,
                    RobotCommandKind::EnableServos, sequence,
                    GatewayCommandOutcome::Accepted);
        ++sequence;
        expect(pumpUntil([&] { return controller.isServoEnabled(id); }),
               "motion setup Enable must be confirmed");
    }

    expect(controller.startMotion(rb::MotionMode::Forward),
           "Forward must submit for supersession test");
    const auto pendingStart =
        gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(pendingStart.has_value(),
           "gateway must receive pending StartMotion");
    if (!pendingStart.has_value()) {
        return;
    }
    sendSubmitted(gateway, pendingStart->request_id, 80U);
    QApplication::processEvents();
    expect(controller.isMotionActive(),
           "unresolved StartMotion must reserve the motion lifecycle");

    expect(controller.stopMotion(),
           "STOP must supersede unresolved StartMotion");
    const auto stop =
        gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(stop.has_value(),
           "gateway must receive superseding StopMotion");
    if (!stop.has_value()) {
        return;
    }
    sendSubmitted(gateway, stop->request_id, 81U);
    sendOutcome(gateway, stop->request_id,
                RobotCommandKind::StopMotion, 81U,
                GatewayCommandOutcome::Accepted);
    expect(pumpUntil([&] {
        return controller.motionState() == rb::MotionState::Stopping;
    }), "accepted superseding STOP must enter Stopping");

    sendOutcome(gateway, pendingStart->request_id,
                RobotCommandKind::StartMotion, 80U,
                GatewayCommandOutcome::Accepted);
    QApplication::processEvents();
    expect(controller.motionState() != rb::MotionState::Running,
           "late superseded Start outcome must never resurrect Running");
    expect(pumpUntil([&] {
        return controller.motionState() == rb::MotionState::Stopped;
    }, rb::kMotionTransitionDurationMs + 400),
           "superseding STOP must settle at Stopped");
}

void testRemoteGaitSelectorsAndMotionGate()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = gateway.port();
    controller.connectController(config);
    expect(gateway.accept(), "selector test must connect to fake gateway");
    completeHello(gateway);
    expect(pumpUntil([&] { return controller.canAcquireControl(); }),
           "selector test must complete Hello");
    expect(controller.acquireControl(),
           "selector test must acquire remote control");
    completeAcquire(gateway);
    expect(pumpUntil([&] { return controller.isControlActive(); }),
           "selector test must reach active control");

    expect(controller.setFrontRearCoordination(
               rb::FrontRearCoordination::OppositeDirection),
           "remote coordination selector should submit");
    const auto coordination = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(coordination.has_value()
               && coordination->payload
                   == Bytes({0x09, 0x01}),
           "RBRP coordination request must contain kind 0x09 and one value byte");
    expect(controller.isFrontRearCoordinationChangePending()
               && !controller.confirmedFrontRearCoordination().has_value(),
           "remote coordination remains pending until accepted outcome");
    expect(!controller.setGaitBackend(rb::GaitBackend::ExperimentalFlex),
           "remote gait selector must be blocked while coordination is pending");
    if (coordination.has_value()) {
        sendSubmitted(gateway, coordination->request_id, 10U);
        sendOutcome(gateway, coordination->request_id,
                    RobotCommandKind::SetFrontRearCoordination, 10U,
                    GatewayCommandOutcome::Accepted);
    }
    expect(pumpUntil([&] {
        return !controller.isFrontRearCoordinationChangePending()
            && controller.confirmedFrontRearCoordination().has_value();
    }), "accepted remote coordination outcome must confirm its value");
    expect(*controller.confirmedFrontRearCoordination()
               == rb::FrontRearCoordination::OppositeDirection,
           "remote coordination outcome confirms Opposite Direction");

    expect(controller.setGaitBackend(rb::GaitBackend::ExperimentalFlex),
           "remote ExperimentalFlex selector should submit");
    const auto backend = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(backend.has_value() && backend->payload == Bytes({0x07, 0x02}),
           "remote ExperimentalFlex request must preserve kind 0x07 and value two");
    expect(!controller.setFrontRearCoordination(
               rb::FrontRearCoordination::SameDirection),
           "remote coordination selector must be blocked while gait is pending");
    if (backend.has_value()) {
        sendSubmitted(gateway, backend->request_id, 11U);
        sendOutcome(gateway, backend->request_id,
                    RobotCommandKind::SetGaitBackend, 11U,
                    GatewayCommandOutcome::Accepted);
    }
    expect(pumpUntil([&] {
        return !controller.isGaitBackendChangePending()
            && controller.confirmedGaitBackend().has_value();
    }), "accepted remote gait outcome must confirm its value");
    expect(*controller.confirmedGaitBackend() == rb::GaitBackend::ExperimentalFlex,
           "remote outcome confirms ExperimentalFlex");

    const std::pair<rb::ServoId, quint16> motionServos[] = {
        {rb::ServoId::FrontRight, 12U}, {rb::ServoId::FrontLeft, 13U},
        {rb::ServoId::RearRight, 14U}, {rb::ServoId::RearLeft, 15U},
    };
    for (const auto &[servo, sequence] : motionServos) {
        expect(controller.enableServo(servo),
               "remote motion gate fixture must enable each paddle");
        const auto enable = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        expect(enable.has_value(), "remote gateway must receive paddle enable");
        if (enable.has_value()) {
            sendSubmitted(gateway, enable->request_id, sequence);
            sendOutcome(gateway, enable->request_id,
                        RobotCommandKind::EnableServos, sequence,
                        GatewayCommandOutcome::Accepted);
            expect(pumpUntil([&] { return controller.isServoEnabled(servo); }),
                   "remote paddle enable must be accepted before Motion");
        }
    }

    expect(controller.startMotion(rb::MotionMode::Forward),
           "remote gate fixture must submit Forward");
    const auto start = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(start.has_value(), "remote gateway must receive Forward request");
    expect(controller.isMotionActive(),
           "pending remote Motion is active for selector gating");
    expect(!controller.setGaitBackend(rb::GaitBackend::CPG)
               && !controller.setFrontRearCoordination(
                   rb::FrontRearCoordination::SameDirection),
           "both remote selectors must reject while Motion start is pending");
    expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest, 100).has_value(),
           "rejected remote selectors must submit no RBRP command");
    if (start.has_value()) {
        sendSubmitted(gateway, start->request_id, 16U);
        sendOutcome(gateway, start->request_id, RobotCommandKind::StartMotion,
                    16U, GatewayCommandOutcome::Accepted);
    }
    expect(pumpUntil([&] {
        return controller.motionState() == rb::MotionState::Running;
    }), "accepted remote Motion start must retain selector gate");
    gateway.disconnectPeer();
}

void testCommandTimeoutReleasesAuthority()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = gateway.port();
    controller.connectController(config);

    expect(gateway.accept(), "timeout test must connect to fake gateway");
    completeHello(gateway);
    expect(pumpUntil([&] { return controller.canAcquireControl(); }),
           "timeout test must complete Hello");
    expect(controller.acquireControl(),
           "timeout test must submit AcquireControl");
    completeAcquire(gateway);
    expect(pumpUntil([&] { return controller.isControlActive(); }),
           "timeout test must reach Active control");

    expect(controller.enableServo(rb::ServoId::FrontRight),
           "timeout test must submit one actuator command");
    const auto command = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(command.has_value(),
           "timeout test gateway must receive actuator command");

    const auto release = gateway.nextFrame(RbrpMessageKind::ReleaseControl, 3000);
    expect(release.has_value(),
           "uncertain remote command must release authority after client timeout");
    expect(pumpUntil([&] {
        return controller.authorityState() == rb::ControlAuthorityState::Unowned;
    }), "command timeout must fail closed to Unowned");
    expect(!controller.isServoEnabled(rb::ServoId::FrontRight),
           "uncertain command must never be treated as Servo enabled");
    expect(controller.monitor().timeoutCount >= 1U,
           "command timeout must be visible in protocol monitor");
    expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest, 200).has_value(),
           "uncertain command must never be automatically replayed");
}

} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    testRemoteControllerAndUi();
    testUserReleaseDoesNotReportAuthorityLoss();
    testSafetySupersessionIgnoresLateOutcomes();
    testRemoteGaitSelectorsAndMotionGate();
    testCommandTimeoutReleasesAuthority();
    if (failures == 0) {
        std::fprintf(stdout,
                     "All RemoteRobotController/MainWindow tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
