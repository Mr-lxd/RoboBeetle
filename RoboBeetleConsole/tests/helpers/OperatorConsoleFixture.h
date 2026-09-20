#pragma once

#include "remote/RemoteRobotController.h"
#include "robot/RobotController.h"
#include "transport/FakeTransport.h"
#include "vision/VisionClient.h"
#include "vision/VisionControlClient.h"

#include <QImage>
#include <QPainter>
#include <QTcpServer>
#include <QTcpSocket>

#include <memory>

namespace rb::test {

// Test-only owner for deterministic console dependencies.  It never enumerates
// host serial ports and never opens a physical transport.
struct OperatorConsoleFixture final {
    FakeTransport directTransport;
    std::unique_ptr<RobotController> directController;
    std::unique_ptr<RemoteRobotController> remoteController;
    std::unique_ptr<vision::VisionClient> visionClient;
    std::unique_ptr<vision::VisionControlClient> visionControlClient;

    void create(bool direct = false, bool withVision = false)
    {
        if (direct) {
            directController = std::make_unique<RobotController>(
                &directTransport,
                RobotControllerConfig::bringUpProvisional(),
                [](void) { return QStringList{}; });
        } else {
            remoteController = std::make_unique<RemoteRobotController>();
        }
        if (withVision) {
            visionClient = std::make_unique<vision::VisionClient>();
            visionControlClient = std::make_unique<vision::VisionControlClient>();
        }
    }

    IConsoleController *controller() const
    {
        return directController != nullptr
            ? static_cast<IConsoleController *>(directController.get())
            : static_cast<IConsoleController *>(remoteController.get());
    }

    static QImage syntheticVideo()
    {
        QImage image(640, 480, QImage::Format_RGB32);
        image.fill(QColor(QStringLiteral("#101820")));
        QPainter painter(&image);
        painter.setPen(QColor(QStringLiteral("#7FD7FF")));
        painter.setFont(QFont(QStringLiteral("Segoe UI"), 28, QFont::Bold));
        painter.drawText(image.rect(), Qt::AlignCenter, QStringLiteral("SIMULATED VIDEO"));
        painter.end();
        return image;
    }
};

// A loopback-only response helper used by preview/tests when the real
// VisionControlClient must exercise its HTTP path.  It never connects outside
// localhost and can be left idle for purely structural UI cases.
class LoopbackVisionStatusServer final : public QObject {
public:
    explicit LoopbackVisionStatusServer(QObject *parent = nullptr)
        : QObject(parent)
    {
        connect(&server_, &QTcpServer::newConnection, this, [this] {
            while (server_.hasPendingConnections()) {
                auto *socket = server_.nextPendingConnection();
                connect(socket, &QTcpSocket::readyRead, socket, [this, socket] {
                    if (!socket->property("rb_replied").toBool()
                        && socket->readAll().contains("\r\n\r\n")) {
                        socket->setProperty("rb_replied", true);
                        const QByteArray body = body_.isEmpty()
                            ? QByteArrayLiteral(
                                  "{\"ok\":true,\"camera\":{\"running\":true},"
                                  "\"capture\":{\"state\":\"idle\",\"recording\":false},"
                                  "\"inference\":{\"configured\":true,"
                                  "\"control_supported\":true,\"operation\":null,"
                                  "\"state\":\"disabled\"}}")
                            : body_;
                        const QByteArray response =
                            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                            "Content-Length: " + QByteArray::number(body.size())
                            + "\r\nConnection: close\r\n\r\n" + body;
                        socket->write(response);
                        socket->flush();
                        socket->disconnectFromHost();
                    }
                });
                connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
            }
        });
    }

    bool listen()
    {
        return server_.listen(QHostAddress::LocalHost, 0);
    }

    quint16 port() const { return server_.serverPort(); }

    void setBody(const QByteArray &body) { body_ = body; }

private:
    QTcpServer server_;
    QByteArray body_;
};

} // namespace rb::test
