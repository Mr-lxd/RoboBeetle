#pragma once
#include "controller/IConsoleController.h"
#include <vector>
#include <functional>
namespace rb::test {
class VisualControllerFixture : public IConsoleController {
public:
    ConsoleBackendKind backend{ConsoleBackendKind::RemoteRbrp};
    bool connected{true},active{true},synchronous{false};
    quint16 enabled{0x1b},known{0x1b};
    quint32 next{1};
    bool motionActive{false};
    std::function<void()> beforeManual;
    std::vector<QString> manualRecords;
    bool manual(const QString &action) { if (beforeManual) beforeManual(); manualRecords.push_back(action); return true; }
    std::vector<std::pair<quint32,MotionMode>> sends;
    ConsoleBackendKind backendKind()const noexcept override{return backend;}
    void refreshSerialPorts()override{}
    void connectController(const ConsoleConnectionConfiguration&)override{}
    void disconnectController()override{connected=false;active=false;emit connectionStateChanged(TransportState::Disconnected);}
    void shutdown()override{}
    bool canAcquireControl()const override{return !active;}
    bool acquireControl()override{return false;}
    bool releaseControl()override{return false;}
    bool enableServo(ServoId id)override{return manual(QStringLiteral("enable:%1").arg(static_cast<int>(id)));}
    bool disableServo(ServoId id)override{return manual(QStringLiteral("disable:%1").arg(static_cast<int>(id)));}
    bool disableAll()override{return manual(QStringLiteral("disableAll"));}
    bool setServoPwm(ServoId,quint16)override{return manual(QStringLiteral("pwm"));}
    bool setServoAngle(ServoId,qint16)override{return manual(QStringLiteral("angle"));}
    bool neutralServo(ServoId id)override{return manual(QStringLiteral("neutral:%1").arg(static_cast<int>(id)));}
    bool startMotion(MotionMode)override{const bool sent=manual(QStringLiteral("start"));motionActive=sent;return sent;}
    bool stopMotion()override{const bool sent=manual(QStringLiteral("stop"));motionActive=false;return sent;}
    bool setGaitBackend(GaitBackend)override{return manual(QStringLiteral("gait"));}
    bool setFrontRearCoordination(FrontRearCoordination)override{return manual(QStringLiteral("coordination"));}
    std::optional<quint32> submitVisualMotion(MotionMode mode)override{if(!active)return {};auto id=next++;sends.emplace_back(id,mode);if(synchronous)emit commandTerminal(id,CommandTerminalResult::Ok,0,1);return id;}
    quint16 inferredPoseKnownMask()const override{return known;}
    bool isConnected()const override{return connected;}
    bool isControlActive()const override{return active;}
    ControlAuthorityState authorityState()const override{return active?ControlAuthorityState::Owned:ControlAuthorityState::Unowned;}
    bool supportsRawPwm()const noexcept override{return true;}
    bool isServoSupported(ServoId)const override{return true;}
    bool isServoEnabled(ServoId id)const override{return (enabled&servoMask(id))!=0;}
    bool isServoDisablePending(ServoId)const override{return false;}
    LeakState leakState()const override{return LeakState::Unknown;}
    const ImuMonitorState& imuState()const override{static ImuMonitorState s;return s;}
    const DepthMonitorState& depthState()const override{static DepthMonitorState s;return s;}
    ProtocolMonitor monitor()const override{return {};}
    MotionState motionState()const override{return motionActive?MotionState::Running:MotionState::Stopped;}
    MotionMode motionMode()const override{return MotionMode::Stop;}
    std::optional<GaitBackend> confirmedGaitBackend()const override{return {};}
    std::optional<GaitBackend> requestedGaitBackend()const override{return {};}
    bool isGaitBackendChangePending()const override{return false;}
    std::optional<FrontRearCoordination> confirmedFrontRearCoordination()const override{return {};}
    std::optional<FrontRearCoordination> requestedFrontRearCoordination()const override{return {};}
    bool isFrontRearCoordinationChangePending()const override{return false;}
    bool isMotionActive()const override{return motionActive;}
    bool isMotionReady(MotionMode)const override{return active;}
    bool isMotionTransitioning()const override{return false;}
    void ack(quint32 id,CommandTerminalResult result,quint8 raw=0){emit commandTerminal(id,result,raw,5);}
};
}
