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

#include <cmath>
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
        if (peer_ != nullptr)
        {
            QObject::connect(peer_, &QTcpSocket::readyRead, peer_, [this] { receive(); });
            receive();
        }
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
            receive();
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
  void receive()
  {
      if (peer_ == nullptr || peer_->bytesAvailable() == 0)
          return;
      const QByteArray bytes = peer_->readAll();
      std::vector<RbrpFrame> decoded;
      const auto status = decoder_.feed(reinterpret_cast<const Byte *>(bytes.constData()),
                                        static_cast<std::size_t>(bytes.size()), decoded);
      expect(status == RbrpFeedStatus::Ok, "fake gateway must decode controller traffic");
      for (const auto &frame : decoded)
      {
          if (frame.kind != RbrpMessageKind::CommandRequest ||
              frame.payload != Bytes{static_cast<Byte>(RobotCommandKind::QueryCpgParameters)})
          {
              queued_.push_back(frame);
              continue;
          }
          // Acquire performs one read-only query. Complete it independently of
          // the actuator traffic observed by all existing safety assertions.
          constexpr quint16 querySequence = 0xff00;
          CommandSubmittedMessage submitted;
          submitted.status = CommandSubmittedStatus::Submitted;
          submitted.sequence = querySequence;
          send({frame.request_id, submitted});
          GatewayCommandOutcomeMessage outcome;
          outcome.command_kind = RobotCommandKind::QueryCpgParameters;
          outcome.event.outcome = GatewayCommandOutcome::Accepted;
          outcome.event.sequence = querySequence;
          outcome.event.result = 0;
          send({frame.request_id, outcome});
          robobeetle::protocol::CpgParametersSnapshot snapshot;
          snapshot.request_sequence = querySequence;
          snapshot.feature_level = 1;
          const auto payload = robobeetle::protocol::encode_cpg_snapshot(snapshot);
          expect(payload.has_value(), "fake parameter snapshot must encode");
          if (payload)
              send({0, GatewayCpgParametersTelemetry{1, 1000, *payload}});
      }
  }

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

    QPushButton *backward = buttonWithText(&window, QStringLiteral("Backward"));
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
    // Declared before the controller: the controller emits logMessage while it
    // is destroyed, and the lambda below writes into `logs`.
    QStringList logs;
    rb::RemoteRobotController controller;
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

void testStopRetirementPreservesReentrantRequest()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = gateway.port();
    controller.connectController(config);
    gateway.accept();
    completeHello(gateway);
    pumpUntil([&] { return controller.canAcquireControl(); });
    controller.acquireControl();
    completeAcquire(gateway);
    pumpUntil([&] { return controller.isControlActive(); });
    const auto first = controller.submitVisualMotion(rb::MotionMode::Stop);
    const auto second = controller.submitVisualMotion(rb::MotionMode::Stop);
    gateway.nextFrame(RbrpMessageKind::CommandRequest);
    gateway.nextFrame(RbrpMessageKind::CommandRequest);
    std::optional<quint32> reentrant;
    bool reentrantOk = false;
    const auto connection = QObject::connect(
        &controller, &rb::IConsoleController::commandTerminal,
        [&](quint32 id, rb::CommandTerminalResult result, quint8 raw, qint64) {
            if (first && id == *first && result == rb::CommandTerminalResult::Ok && raw == 0xff)
                reentrant = controller.submitVisualMotion(rb::MotionMode::Stop);
            if (reentrant && id == *reentrant && result == rb::CommandTerminalResult::Ok)
                reentrantOk = true;
        });
    if (first) sendSubmitted(gateway, *first, 80);
    if (second) {
        sendSubmitted(gateway, *second, 81);
        sendOutcome(gateway, *second, RobotCommandKind::StopMotion, 81,
                    GatewayCommandOutcome::Accepted);
    }
    expect(pumpUntil([&] { return reentrant.has_value(); }),
           "retirement callback can submit a new operator STOP");
    auto frame = gateway.nextFrame(RbrpMessageKind::CommandRequest);
    expect(frame && reentrant && frame->request_id == *reentrant,
           "reentrant STOP uses a fresh wire ID");
    pumpUntil([&] { return controller.motionState() == rb::MotionState::Stopped; });
    expect(controller.isMotionActive(),
           "new STOP from retirement callback remains pending after old interval cleanup");
    if (reentrant) {
        sendSubmitted(gateway, *reentrant, 82);
        sendOutcome(gateway, *reentrant, RobotCommandKind::StopMotion, 82,
                    GatewayCommandOutcome::Accepted);
    }
    expect(pumpUntil([&] { return reentrantOk; }),
           "reentrant STOP retains its own terminal outcome correlation");
    QObject::disconnect(connection);
    gateway.disconnectPeer();
}

void testMonotonicTerminalRtt()
{
    FakeGatewayPeer gateway;
    qint64 monotonicNow = 10;
    rb::RemoteRobotController controller(nullptr, [&] { return monotonicNow; });
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = gateway.port();
    controller.connectController(config);
    gateway.accept();
    completeHello(gateway);
    pumpUntil([&] { return controller.canAcquireControl(); });
    controller.acquireControl();
    completeAcquire(gateway);
    pumpUntil([&] { return controller.isControlActive(); });
    const auto id = controller.submitVisualMotion(rb::MotionMode::Stop);
    gateway.nextFrame(RbrpMessageKind::CommandRequest);
    std::optional<qint64> rtt;
    QObject::connect(&controller, &rb::IConsoleController::commandTerminal,
                     [&](quint32 received, rb::CommandTerminalResult, quint8, qint64 value) {
        if (id && received == *id) rtt = value;
    });
    monotonicNow = 85;
    if (id) {
        sendSubmitted(gateway, *id, 44);
        sendOutcome(gateway, *id, RobotCommandKind::StopMotion, 44,
                    GatewayCommandOutcome::Accepted);
    }
    expect(pumpUntil([&] { return rtt.has_value(); }), "monotonic terminal arrives");
    expect(rtt == 75, "terminal RTT uses injected monotonic send-to-outcome clock");
    gateway.disconnectPeer();
}

void testNegativeSubmissionTerminalMapping()
{
    for (auto status : {CommandSubmittedStatus::InvalidArgument,
                        CommandSubmittedStatus::PendingQualification,
                        CommandSubmittedStatus::NotActive,
                        CommandSubmittedStatus::PayloadTooLarge,
                        CommandSubmittedStatus::QueueFull,
                        CommandSubmittedStatus::TransportRejected}) {
        FakeGatewayPeer gateway;
        rb::RemoteRobotController controller;
        rb::ConsoleConnectionConfiguration config;
        config.endpoint = QStringLiteral("127.0.0.1");
        config.tcpPort = gateway.port();
        controller.connectController(config);
        expect(gateway.accept(), "negative submission connects");
        completeHello(gateway);
        pumpUntil([&] { return controller.canAcquireControl(); });
        controller.acquireControl();
        completeAcquire(gateway);
        pumpUntil([&] { return controller.isControlActive(); });
        const auto id = controller.submitVisualMotion(rb::MotionMode::Stop);
        gateway.nextFrame(RbrpMessageKind::CommandRequest);
        std::optional<rb::CommandTerminalResult> observed;
        bool activeAtOutcome = false;
        QObject::connect(&controller, &rb::IConsoleController::commandTerminal,
                         [&](quint32 received, rb::CommandTerminalResult result,
                             quint8 raw, qint64 rtt) {
            if (id && received == *id) {
                observed = result;
                activeAtOutcome = controller.isControlActive();
                expect(raw == 0xff && rtt >= 0,
                       "negative submission has no firmware result but retains RTT");
            }
        });
        if (id) gateway.send({*id, CommandSubmittedMessage{status, std::nullopt}});
        expect(pumpUntil([&] { return observed.has_value(); }),
               "negative submission emits terminal result");
        expect(observed == rb::CommandTerminalResult::Rejected,
               "known not-submitted status is Rejected rather than OutcomeUnknown");
        expect(activeAtOutcome, "negative submission terminal precedes cleanup");
        gateway.disconnectPeer();
    }
}

void testVisualTerminalMapping()
{
    struct Row { GatewayCommandOutcome outcome; quint8 raw; rb::CommandTerminalResult terminal; };
    const Row rows[]={{GatewayCommandOutcome::Accepted,0,rb::CommandTerminalResult::Ok},{GatewayCommandOutcome::Rejected,7,rb::CommandTerminalResult::Busy},{GatewayCommandOutcome::Rejected,6,rb::CommandTerminalResult::Rejected},{GatewayCommandOutcome::OutcomeUnknown,0,rb::CommandTerminalResult::OutcomeUnknown},{GatewayCommandOutcome::Cancelled,0,rb::CommandTerminalResult::OutcomeUnknown}};
    for(const auto row:rows) {
        FakeGatewayPeer gateway;rb::RemoteRobotController c;rb::ConsoleConnectionConfiguration cfg;
        cfg.endpoint=QStringLiteral("127.0.0.1");cfg.tcpPort=gateway.port();c.connectController(cfg);gateway.accept();completeHello(gateway);
        pumpUntil([&]{return c.canAcquireControl();});c.acquireControl();completeAcquire(gateway);pumpUntil([&]{return c.isControlActive();});
        auto id=c.submitVisualMotion(rb::MotionMode::Stop);auto frame=gateway.nextFrame(RbrpMessageKind::CommandRequest);
        expect(id&&frame&&frame->request_id==*id,"typed outcome uses actual wire request ID");
        std::optional<rb::CommandTerminalResult> observed;bool activeAtOutcome=false;quint8 raw=0xff;qint64 rtt=-1;
        QObject::connect(&c,&rb::IConsoleController::commandTerminal,[&](quint32 received,rb::CommandTerminalResult result,quint8 resultByte,qint64 time){if(id&&received==*id){observed=result;raw=resultByte;rtt=time;activeAtOutcome=c.isControlActive();}});
        if(id){sendSubmitted(gateway,*id,42);GatewayCommandOutcomeMessage m;m.command_kind=RobotCommandKind::StopMotion;m.event={row.outcome,42,row.raw};gateway.send({*id,m});}
        expect(pumpUntil([&]{return observed.has_value();}),"validated outcome emits terminal");
        expect(observed==row.terminal&&raw==row.raw&&rtt>=0,"terminal maps typed result/raw/RTT");
        expect(activeAtOutcome,"terminal emitted while authority available");gateway.disconnectPeer();
    }
}

void testActiveControllerDestruction()
{
    FakeGatewayPeer gateway;
    {
        rb::RemoteRobotController controller;
        rb::ConsoleConnectionConfiguration cfg;cfg.endpoint=QStringLiteral("127.0.0.1");cfg.tcpPort=gateway.port();
        controller.connectController(cfg);expect(gateway.accept(),"destruction loopback connects");completeHello(gateway);
        pumpUntil([&]{return controller.canAcquireControl();});controller.acquireControl();completeAcquire(gateway);
        expect(pumpUntil([&]{return controller.isControlActive();}),"destruction has active socket");
        controller.enableServo(rb::ServoId::FrontRight);
        gateway.nextFrame(RbrpMessageKind::CommandRequest);
        // Intentionally destroy while a correlated command is outstanding.
    }
    gateway.disconnectPeer();
}

void testVisualWireAndPoseInference()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1"); config.tcpPort = gateway.port();
    controller.connectController(config); expect(gateway.accept(), "visual loopback connects");
    completeHello(gateway); expect(pumpUntil([&]{return controller.canAcquireControl();}), "hello ready");
    expect(controller.isConnected() && !controller.isControlActive(),
           "no-authority submission test keeps TCP connected");
    for (const auto mode : {rb::MotionMode::Stop, rb::MotionMode::Forward,
                           rb::MotionMode::TurnLeft, rb::MotionMode::TurnRight})
        expect(!controller.submitVisualMotion(mode),
               "visual motion without control authority returns nullopt");
    expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest, 20),
           "no-authority visual submissions never reach the wire");

    expect(controller.acquireControl(), "acquire visual loopback"); completeAcquire(gateway);
    expect(pumpUntil([&]{return controller.isControlActive();}), "visual active");
    const auto first = controller.submitVisualMotion(rb::MotionMode::Stop);
    const auto second = controller.submitVisualMotion(rb::MotionMode::Stop);
    expect(first && second && first != second, "visual STOP always sends fresh IDs even stopped/pending");
    if (!first || !second) {gateway.disconnectPeer();return;}
    for (auto id : {*first,*second}) {
        auto frame=gateway.nextFrame(RbrpMessageKind::CommandRequest);
        expect(frame && frame->request_id==id && frame->payload==Bytes{static_cast<Byte>(RobotCommandKind::StopMotion)}, "visual STOP empty payload on existing socket");
        sendSubmitted(gateway,id,static_cast<quint16>(id));
        sendOutcome(gateway,id,RobotCommandKind::StopMotion,static_cast<quint16>(id),GatewayCommandOutcome::Accepted);
    }
    expect(pumpUntil([&]{return controller.motionState()==rb::MotionState::Stopped && !controller.isMotionActive();}), "visual STOP settlement waits for pending requests and ramp");
    auto acceptServo=[&](const std::function<bool()> &action, RobotCommandKind kind, quint16 seq){
        expect(action(), "pose command submitted"); auto f=gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (f) {
            bool terminalReceived = false;
            const auto id = f->request_id;
            const auto connection = QObject::connect(
                &controller, &rb::IConsoleController::commandTerminal,
                [&](quint32 received, rb::CommandTerminalResult, quint8, qint64) {
                    if (received == id) terminalReceived = true;
                });
            sendSubmitted(gateway, id, seq);
            sendOutcome(gateway, id, kind, seq, GatewayCommandOutcome::Accepted);
            expect(pumpUntil([&] { return terminalReceived; }),
                   "pose helper waits for the matching terminal ACK");
            QObject::disconnect(connection);
        }
    };
    acceptServo([&]{return controller.enableServo(rb::ServoId::FrontRight);},RobotCommandKind::EnableServos,100);
    expect(pumpUntil([&]{return controller.inferredPoseKnownMask()==1;}), "new Enable ACK establishes neutral pose");
    expect(controller.enableServo(rb::ServoId::FrontRight) && controller.inferredPoseKnownMask()==1,"repeated alreadyenabled Enable preserves known pose");
    acceptServo([&]{return controller.setServoPwm(rb::ServoId::FrontRight,1500);},RobotCommandKind::SetServoPwm,101);
    expect(pumpUntil([&]{return controller.inferredPoseKnownMask()==0;}), "PWM ACK invalidates pose even neutral pulse");
    expect(controller.enableServo(rb::ServoId::FrontRight), "repeated Enable succeeds locally");
    expect(controller.inferredPoseKnownMask()==0, "repeated Enable never restores unknown pose");
    acceptServo([&]{return controller.neutralServo(rb::ServoId::FrontRight);},RobotCommandKind::NeutralServos,102);
    expect(pumpUntil([&]{return controller.inferredPoseKnownMask()==1;}), "Neutral ACK establishes pose");
    acceptServo([&]{return controller.setServoAngle(rb::ServoId::FrontRight,0);},RobotCommandKind::SetServoAngle,103);
    expect(controller.inferredPoseKnownMask()==1, "Angle ACK establishes pose");
    acceptServo([&]{return controller.disableServo(rb::ServoId::FrontRight);},RobotCommandKind::DisableServos,104);
    expect(pumpUntil([&]{return controller.inferredPoseKnownMask()==0;}), "Disable ACK invalidates pose");
    acceptServo([&]{return controller.enableServo(rb::ServoId::FrontRight);},RobotCommandKind::EnableServos,105);
    expect(pumpUntil([&]{return controller.inferredPoseKnownMask()==1;}),"re-enable after Disable establishes pose");
    controller.setServoAngle(rb::ServoId::FrontRight,100);auto angle=gateway.nextFrame(RbrpMessageKind::CommandRequest);
    controller.setServoPwm(rb::ServoId::FrontRight,1500);auto pwm=gateway.nextFrame(RbrpMessageKind::CommandRequest);
    if(angle){sendSubmitted(gateway,angle->request_id,106);sendOutcome(gateway,angle->request_id,RobotCommandKind::SetServoAngle,106,GatewayCommandOutcome::Accepted);}
    if(pwm){sendSubmitted(gateway,pwm->request_id,107);sendOutcome(gateway,pwm->request_id,RobotCommandKind::SetServoPwm,107,GatewayCommandOutcome::Accepted);}
    expect(pumpUntil([&]{return controller.inferredPoseKnownMask()==0;}),"stale superseded Angle cannot restore newer PWM unknown");
    controller.neutralServo(rb::ServoId::FrontRight);auto rejected=gateway.nextFrame(RbrpMessageKind::CommandRequest);
    if(rejected){sendSubmitted(gateway,rejected->request_id,108);GatewayCommandOutcomeMessage m;m.command_kind=RobotCommandKind::NeutralServos;m.event={GatewayCommandOutcome::Rejected,108,6};gateway.send({rejected->request_id,m});}
    expect(controller.inferredPoseKnownMask()==0,"nonOK Neutral never establishes known pose");
    acceptServo([&]{return controller.neutralServo(rb::ServoId::FrontRight);},RobotCommandKind::NeutralServos,109);
    expect(pumpUntil([&]{return controller.inferredPoseKnownMask()==1;}),"neutral restores pose before session loss");
    gateway.disconnectPeer(); expect(controller.inferredPoseKnownMask()==0&&!controller.isServoEnabled(rb::ServoId::FrontRight),"disconnect invalidates pose and enabled inference");
    controller.connectController(config);expect(gateway.accept(),"reconnect same controller socket");completeHello(gateway);pumpUntil([&]{return controller.canAcquireControl();});controller.acquireControl();completeAcquire(gateway);pumpUntil([&]{return controller.isControlActive();});
    expect(controller.inferredPoseKnownMask()==0&&!controller.isServoEnabled(rb::ServoId::FrontRight),"reconstruction never restores previous pose/enabled");gateway.disconnectPeer();
}

// Task 06 PR-C: automatic dispatch may use Ascend/Descend, which need the front axis.
void testVisualAscendDescendAndFrontAxis()
{
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1"); config.tcpPort = gateway.port();
    controller.connectController(config);
    expect(gateway.accept(), "ascend loopback connects");
    completeHello(gateway);
    expect(pumpUntil([&] { return controller.canAcquireControl(); }), "ascend hello");
    expect(controller.acquireControl(), "ascend acquire"); completeAcquire(gateway);
    expect(pumpUntil([&] { return controller.isControlActive(); }), "ascend active");
    quint16 sequence = 200;
    const auto enable = [&](rb::ServoId servo) {
        expect(controller.enableServo(servo), "enable submitted");
        const auto frame = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (!frame) return;
        sendSubmitted(gateway, frame->request_id, sequence);
        sendOutcome(gateway, frame->request_id, RobotCommandKind::EnableServos, sequence,
                    GatewayCommandOutcome::Accepted);
        ++sequence;
        expect(pumpUntil([&] { return controller.isServoEnabled(servo); }), "enable accepted");
    };
    for (const auto servo : {rb::ServoId::FrontRight, rb::ServoId::FrontLeft,
                             rb::ServoId::RearRight, rb::ServoId::RearLeft})
        enable(servo);
    expect(controller.isMotionReady(rb::MotionMode::Forward), "four paddles are enough for Forward");
    for (const auto mode : {rb::MotionMode::Ascend, rb::MotionMode::Descend}) {
        expect(!controller.isMotionReady(mode), "Ascend/Descend need the front axis too");
        expect(!controller.submitVisualMotion(mode), "visual Ascend/Descend is refused without the front axis");
    }
    expect(!gateway.nextFrame(RbrpMessageKind::CommandRequest, 50), "refused submissions never reach the wire");

    enable(rb::ServoId::FrontAxis);
    for (const auto mode : {rb::MotionMode::Ascend, rb::MotionMode::Descend}) {
        expect(controller.isMotionReady(mode), "with the front axis enabled Ascend/Descend are ready");
        const auto id = controller.submitVisualMotion(mode);
        expect(id.has_value(), "visual Ascend/Descend is accepted");
        const auto frame = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        expect(frame && frame->request_id == id.value_or(0)
                   && frame->payload == Bytes({static_cast<Byte>(RobotCommandKind::StartMotion),
                                               static_cast<Byte>(mode)}),
               "Ascend/Descend go out as StartMotion with the mode byte");
        if (frame && id) {
            sendSubmitted(gateway, frame->request_id, sequence);
            sendOutcome(gateway, frame->request_id, RobotCommandKind::StartMotion, sequence,
                        GatewayCommandOutcome::Accepted);
            ++sequence;
            expect(pumpUntil([&] { return controller.motionMode() == mode; }), "mode confirmed");
        }
        // Back to Stop so the next mode starts from a settled state.
        const auto stop = controller.submitVisualMotion(rb::MotionMode::Stop);
        const auto stopFrame = gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (stop && stopFrame) {
            sendSubmitted(gateway, stopFrame->request_id, sequence);
            sendOutcome(gateway, stopFrame->request_id, RobotCommandKind::StopMotion, sequence,
                        GatewayCommandOutcome::Accepted);
            ++sequence;
            expect(pumpUntil([&] { return !controller.isMotionActive(); }), "stopped");
        }
    }
    expect(!controller.submitVisualMotion(rb::MotionMode::Backward),
           "other modes stay refused for automatic dispatch");
    gateway.disconnectPeer();
}

} // namespace


// ---- Control depth (PR-B) -------------------------------------------------

struct DepthFixture {
    FakeGatewayPeer gateway;
    rb::RemoteRobotController controller;
    qint64 now{100000};
    int changes{0};
    quint16 sequence{500};
    quint32 line{0};

    DepthFixture()
    {
        controller.setDepthClockForTesting([this] { return now; });
        QObject::connect(&controller,
                         &rb::IConsoleController::controlDepthSampleChanged,
                         &controller, [this] { ++changes; });
        rb::ConsoleConnectionConfiguration config;
        config.endpoint = QStringLiteral("127.0.0.1");
        config.tcpPort = gateway.port();
        controller.connectController(config);
        expect(gateway.accept(), "depth fixture must connect");
        completeHello(gateway);
        expect(pumpUntil([&] { return controller.canAcquireControl(); }),
               "depth fixture must complete Hello");
    }

    // Sends one DepthSnapshot and waits until the controller has processed it.
    void send(double depthM, quint16 ageMs, bool newLine = true,
              quint8 flags = 0x03)
    {
        if (newLine) ++line;
        GatewayDepthTelemetry depth;
        depth.sequence = sequence++;
        depth.schema_version = 1;
        depth.validity_flags = flags;
        depth.depth_mm = static_cast<std::int32_t>(depthM * 1000.0 + (depthM < 0 ? -0.5 : 0.5));
        depth.temperature_centi_c = (flags & 0x02) ? 2000 : 0;
        depth.sample_age_ms = ageMs;
        depth.diagnostics.valid_line_count = line;
        const int before = changes;
        gateway.send({0, depth});
        expect(pumpUntil([&] { return changes > before; }),
               "depth telemetry must emit controlDepthSampleChanged");
    }

    void feedSteady(int count, double depthM, qint64 spacingMs)
    {
        for (int i = 0; i < count; ++i) {
            send(depthM, 10);
            now += spacingMs;
        }
    }
};

void testControlDepthAgeAndFreshness()
{
    DepthFixture f;
    const auto &cfg = f.controller.depthControlConfig();
    expect(!f.controller.controlDepthSample().has_value(),
           "no control sample before any DepthSnapshot");

    f.send(0.123, 40);
    auto s = f.controller.controlDepthSample();
    expect(s.has_value() && s->ageMs == 40,
           "age at receipt equals the firmware sample age");
    expect(s && s->rawDepthM > 0.1229 && s->rawDepthM < 0.1231,
           "raw depth converts millimetres to metres");
    expect(s && !s->calibratedDepthM.has_value(),
           "calibrated depth is empty before zeroing");

    f.now += 100;
    s = f.controller.controlDepthSample();
    expect(s && s->ageMs == 140, "age adds local elapsed time to firmware age");

    f.now += cfg.controlFreshMs - 140 - 40 + 40; // age == controlFreshMs
    s = f.controller.controlDepthSample();
    expect(s && s->ageMs == cfg.controlFreshMs && s->fresh,
           "age == controlFreshMs is still fresh");
    f.now += 1;
    s = f.controller.controlDepthSample();
    expect(s && s->ageMs == cfg.controlFreshMs + 1 && !s->fresh,
           "age == controlFreshMs + 1 is stale");

    f.send(0.2, 0xffff);
    s = f.controller.controlDepthSample();
    expect(s && s->ageMs < 0 && !s->fresh,
           "unknown firmware sample age is never fresh");

    f.send(0.0, 0xffff, true, 0x00);
    s = f.controller.controlDepthSample();
    expect(s && !s->fresh && s->rawDepthM == 0.0,
           "invalid depth flag is never fresh");

    f.gateway.disconnectPeer();
    expect(pumpUntil([&] { return !f.controller.isConnected(); }),
           "depth fixture must see the disconnect");
    expect(!f.controller.controlDepthSample().has_value(),
           "disconnect clears the control depth sample");
}

void testZeroDepth()
{
    DepthFixture f;
    QString error;
    expect(!f.controller.zeroDepth(&error) && !error.isEmpty(),
           "zeroing without any sample must be refused with a reason");

    for (int i = 0; i < 4; ++i) {
        f.send(0.115 + 0.001 * i, 10);
        f.now += 100;
    }
    expect(!f.controller.zeroDepth(&error) && error.contains(QStringLiteral("not enough")),
           "four samples are not enough for zeroing");

    // Re-delivery of the same firmware sample must not count twice.
    f.send(0.118, 10, /*newLine=*/false);
    expect(!f.controller.zeroDepth(&error) && error.contains(QStringLiteral("not enough")),
           "a repeated sample must not count as a distinct sample");

    f.send(0.119, 10);
    const int before = f.changes;
    expect(f.controller.zeroDepth(&error), "five steady samples must zero");
    expect(f.changes > before, "zeroing emits controlDepthSampleChanged");
    expect(f.controller.depthZeroedAtMs() == f.now, "zero time is recorded");
    const double mean = (0.115 + 0.116 + 0.117 + 0.118 + 0.119) / 5.0;
    f.send(0.300, 10);
    auto s = f.controller.controlDepthSample();
    expect(s && s->calibratedDepthM
               && std::abs(*s->calibratedDepthM - (0.300 - mean)) < 1e-6,
           "calibrated = raw - mean of the zeroing samples");

    // Offset survives a reconnect; buffer and sample do not.
    f.gateway.disconnectPeer();
    expect(pumpUntil([&] { return !f.controller.isConnected(); }),
           "zero fixture must see the disconnect");
    expect(!f.controller.controlDepthSample().has_value(),
           "disconnect clears the sample");
    expect(f.controller.depthZeroedAtMs().has_value(),
           "disconnect keeps the zero offset");
    rb::ConsoleConnectionConfiguration config;
    config.endpoint = QStringLiteral("127.0.0.1");
    config.tcpPort = f.gateway.port();
    f.controller.connectController(config);
    expect(f.gateway.accept(), "zero fixture must reconnect");
    completeHello(f.gateway);
    expect(pumpUntil([&] { return f.controller.canAcquireControl(); }),
           "zero fixture must complete the second Hello");
    f.send(0.400, 10);
    s = f.controller.controlDepthSample();
    expect(s && s->calibratedDepthM
               && std::abs(*s->calibratedDepthM - (0.400 - mean)) < 1e-6,
           "offset still applies after reconnect");
    expect(!f.controller.zeroDepth(&error) && error.contains(QStringLiteral("not enough")),
           "the sample buffer is empty after reconnect");
}

// The sensor resolves 1 cm: a steady reading that flips between two adjacent
// values (0.12 / 0.13, which differ by slightly more than 0.01 in floating
// point) must still be zeroable with the default configuration.
void testZeroDepthToleratesOneSensorStep()
{
    DepthFixture f;
    QString error;
    const double values[] = {0.12, 0.13, 0.12, 0.13, 0.12, 0.13};
    for (const double v : values) {
        f.send(v, 10);
        f.now += 100;
    }
    f.now -= 100;
    expect(f.controller.zeroDepth(&error), "alternating 0.12 / 0.13 samples must zero with default settings");
    // Two steps (2 cm) is genuinely unsteady and stays refused.
    DepthFixture g;
    const double jumpy[] = {0.12, 0.14, 0.12, 0.14, 0.12};
    for (const double v : jumpy) {
        g.send(v, 10);
        g.now += 100;
    }
    g.now -= 100;
    expect(!g.controller.zeroDepth(&error) && error.contains(QStringLiteral("not steady")),
           "a 2 cm swing is still refused");
}

void testZeroDepthRejectsUnsteadySamples()
{
    DepthFixture f;
    QString error;
    f.send(0.100, 10); f.now += 100;
    f.send(0.101, 10); f.now += 100;
    f.send(0.140, 10); f.now += 100;
    f.send(0.100, 10); f.now += 100;
    f.send(0.101, 10);
    expect(!f.controller.zeroDepth(&error) && error.contains(QStringLiteral("not steady")),
           "range above zeroMaxRangeM must be refused");
    expect(!f.controller.depthZeroedAtMs().has_value(), "refused zero leaves no offset");
}

void testZeroDepthWindowFollowsSamplePeriod()
{
    // Five samples 600 ms apart span 2400 ms.
    auto run = [](std::int64_t period, std::int64_t fresh) {
        DepthFixture f;
        rb::DepthControlConfig cfg;
        cfg.nominalSamplePeriodMs = period;
        cfg.controlFreshMs = fresh;
        f.controller.setDepthControlConfig(cfg);
        f.feedSteady(5, 0.2, 600);
        f.now -= 600; // newest sample is fresh again
        QString error;
        const bool ok = f.controller.zeroDepth(&error);
        return std::make_pair(ok, error);
    };
    const auto fast = run(250, 700);
    expect(!fast.first && fast.second.contains(QStringLiteral("span")),
           "2400 ms span exceeds the 250 ms-period window (1500 ms)");
    const auto slow = run(500, 1200);
    expect(slow.first, "the window widens with a 500 ms nominal period (3000 ms)");
}

void testZeroDepthRefusedWhileMoving()
{
    DepthFixture f;
    expect(f.controller.acquireControl(), "move fixture must acquire control");
    completeAcquire(f.gateway);
    expect(pumpUntil([&] { return f.controller.isControlActive(); }),
           "move fixture must reach active control");
    const std::pair<rb::ServoId, quint16> servos[] = {
        {rb::ServoId::FrontRight, 12U}, {rb::ServoId::FrontLeft, 13U},
        {rb::ServoId::RearRight, 14U}, {rb::ServoId::RearLeft, 15U},
    };
    for (const auto &[servo, seq] : servos) {
        expect(f.controller.enableServo(servo), "move fixture enables paddles");
        const auto enable = f.gateway.nextFrame(RbrpMessageKind::CommandRequest);
        if (enable.has_value()) {
            sendSubmitted(f.gateway, enable->request_id, seq);
            sendOutcome(f.gateway, enable->request_id, RobotCommandKind::EnableServos,
                        seq, GatewayCommandOutcome::Accepted);
            expect(pumpUntil([&] { return f.controller.isServoEnabled(servo); }),
                   "paddle enable accepted");
        }
    }
    f.feedSteady(5, 0.2, 50);
    f.now -= 50;
    expect(f.controller.startMotion(rb::MotionMode::Forward), "move fixture starts motion");
    expect(f.gateway.nextFrame(RbrpMessageKind::CommandRequest).has_value(),
           "gateway receives StartMotion");
    QString error;
    expect(!f.controller.zeroDepth(&error) && error.contains(QStringLiteral("moving")),
           "zeroing must be refused while motion is active");
    f.gateway.disconnectPeer();
}

void testControlDepthCadenceAndAge()
{
    // 10 Hz telemetry: every frame emits exactly one change and the age of
    // each new sample equals its firmware age.
    DepthFixture f;
    const int before = f.changes;
    for (int i = 0; i < 20; ++i) {
        f.send(0.2, 20);
        const auto s = f.controller.controlDepthSample();
        expect(s && s->ageMs == 20 && s->fresh, "fresh 10 Hz sample age");
        f.now += 100;
    }
    expect(f.changes - before == 20, "one controlDepthSampleChanged per sample");
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    testStopRetirementPreservesReentrantRequest();
    testMonotonicTerminalRtt();
    testNegativeSubmissionTerminalMapping();
    testVisualTerminalMapping();
    testActiveControllerDestruction();
    testVisualWireAndPoseInference();
    testVisualAscendDescendAndFrontAxis();
    testRemoteControllerAndUi();
    testUserReleaseDoesNotReportAuthorityLoss();
    testSafetySupersessionIgnoresLateOutcomes();
    testRemoteGaitSelectorsAndMotionGate();
    testCommandTimeoutReleasesAuthority();
    testControlDepthAgeAndFreshness();
    testZeroDepth();
    testZeroDepthToleratesOneSensorStep();
    testZeroDepthRejectsUnsteadySamples();
    testZeroDepthWindowFollowsSamplePeriod();
    testZeroDepthRefusedWhileMoving();
    testControlDepthCadenceAndAge();
    if (failures == 0) {
        std::fprintf(stdout,
                     "All RemoteRobotController/MainWindow tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
