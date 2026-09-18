#include "controller/IConsoleController.h"
#include "remote/RemoteRobotController.h"
#include "robot/RobotController.h"
#include "transport/SerialTransport.h"
#include "ui/MainWindow.h"

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

    rb::MainWindow window(controller.get());
    window.show();
    return application.exec();
}
