#include "controller/IConsoleController.h"
#include "remote/RemoteRobotController.h"
#include "robot/RobotController.h"
#include "transport/SerialTransport.h"
#include "ui/MainWindow.h"
#include "vision/DetectionClient.h"
#include "vision/VisionClient.h"
#include "vision/VisionControlClient.h"

#include <QApplication>
#include <QCoreApplication>

#include <memory>

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("RoboBeetleConsole"));
    QCoreApplication::setOrganizationName(QStringLiteral("RoboBeetle"));

    const QStringList arguments = QCoreApplication::arguments();
    const bool directMaintenance =
        arguments.contains(QStringLiteral("--direct"))
        || arguments.contains(QStringLiteral("--direct-serial"));

    std::unique_ptr<rb::SerialTransport> serialTransport;
    std::unique_ptr<rb::IConsoleController> controller;
    if (directMaintenance) {
        serialTransport = std::make_unique<rb::SerialTransport>();
        controller = std::make_unique<rb::RobotController>(
            serialTransport.get(),
            rb::RobotControllerConfig::apc220Provisional(),
            [] { return rb::SerialTransport::availablePortNames(); });
    } else {
        controller = std::make_unique<rb::RemoteRobotController>();
    }

    rb::vision::VisionClient visionClient;
    rb::vision::VisionControlClient visionControlClient;
    rb::vision::DetectionClient detectionClient;
    rb::MainWindow window(
        controller.get(),
        &visionClient,
        &visionControlClient,
        &detectionClient);
    // Hidden widgets under-report their size hints, so show once off-screen to
    // get real metrics, then settle the default geometry before the first
    // visible frame: the size where Operator tools are fully expanded, or a
    // maximized window if the screen cannot hold it.
    window.setAttribute(Qt::WA_DontShowOnScreen, true);
    window.show();
    QCoreApplication::processEvents();
    window.hide();
    window.setAttribute(Qt::WA_DontShowOnScreen, false);
    window.settleStartupGeometry();
    if (window.startupWantsMaximized()) window.showMaximized();
    else window.show();
    return application.exec();
}
