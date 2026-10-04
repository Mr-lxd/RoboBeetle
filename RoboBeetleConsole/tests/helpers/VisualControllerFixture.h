#pragma once
#include "controller/IConsoleController.h"
#include <vector>
namespace rb::test {
class VisualControllerFixture : public IConsoleController {
public:
    ConsoleBackendKind backend{ConsoleBackendKind::RemoteRbrp};
    bool connected{true},active{true},synchronous{false};
    quint16 enabled{0x1b},known{0x1b};
    quint32 next{1};
    std::vector<std::pair<quint32,MotionMode>> sends;
    ConsoleBackendKind backendKind()const noexcept override{return backend;}
    void refreshSerialPorts()override{}
    void connectController(const ConsoleConnectionConfiguration&)override{}
    void disconnectController()override{connected=false;active=false;emit connectionStateChanged(TransportState::Disconnected);}
    void shutdown()override{}
    bool canAcquireControl()const override{return !active;}
    bool acquireControl()override{return false;}
    bool releaseControl()override{return false;}
    bool enableServo(ServoId)override{return false;}
    bool disableServo(ServoId)override{return false;}
    bool disableAll()override{return false;}
    bool setServoPwm(ServoId,quint16)override{return false;}
    bool setServoAngle(ServoId,qint16)override{return false;}
    bool neutralServo(ServoId)override{return false;}
    bool startMotion(MotionMode)override{return false;}
    bool stopMotion()override{return false;}
    bool setGaitBackend(GaitBackend)override{return false;}
    bool setFrontRearCoordination(FrontRearCoordination)override{return false;}
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
    MotionState motionState()const override{return MotionState::Stopped;}
    MotionMode motionMode()const override{return MotionMode::Stop;}
    std::optional<GaitBackend> confirmedGaitBackend()const override{return {};}
    std::optional<GaitBackend> requestedGaitBackend()const override{return {};}
    bool isGaitBackendChangePending()const override{return false;}
    std::optional<FrontRearCoordination> confirmedFrontRearCoordination()const override{return {};}
    std::optional<FrontRearCoordination> requestedFrontRearCoordination()const override{return {};}
    bool isFrontRearCoordinationChangePending()const override{return false;}
    bool isMotionActive()const override{return false;}
    bool isMotionReady(MotionMode)const override{return active;}
    bool isMotionTransitioning()const override{return false;}
    void ack(quint32 id,CommandTerminalResult result,quint8 raw=0){emit commandTerminal(id,result,raw,5);}
};
}
