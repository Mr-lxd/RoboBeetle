#include "robot/RobotController.h"
#include "transport/SerialTransport.h"
#include "ui/MainWindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("RoboBeetleConsole"));
    QCoreApplication::setOrganizationName(QStringLiteral("RoboBeetle"));

    rb::SerialTransport transport;
    rb::RobotController controller(
        &transport,
        rb::RobotControllerConfig::apc220Provisional(),
        [] { return rb::SerialTransport::availablePortNames(); });
    rb::MainWindow window(&controller);
    window.show();
    return application.exec();
}

