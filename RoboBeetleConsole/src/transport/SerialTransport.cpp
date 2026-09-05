#include "transport/SerialTransport.h"

#include <QSerialPort>
#include <QSerialPortInfo>

namespace rb {

SerialTransport::SerialTransport(QObject *parent)
    : ITransport(parent), serialPort_(new QSerialPort(this))
{
    connect(serialPort_, &QSerialPort::readyRead, this, [this] {
        emit bytesReceived(serialPort_->readAll());
    });
    connect(serialPort_, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error == QSerialPort::NoError) {
            return;
        }
        emit errorOccurred(serialPort_->errorString());
        if (error == QSerialPort::ResourceError) {
            emit stateChanged(TransportState::Error);
        }
    });
}

SerialTransport::~SerialTransport() = default;

void SerialTransport::open(const TransportConfiguration &configuration)
{
    if (serialPort_->isOpen()) {
        close();
    }
    if (configuration.endpoint.trimmed().isEmpty() || configuration.baudRate <= 0) {
        emit errorOccurred(QStringLiteral("Serial endpoint and baud rate are required"));
        emit stateChanged(TransportState::Error);
        return;
    }

    emit stateChanged(TransportState::Opening);
    serialPort_->setPortName(configuration.endpoint);
    serialPort_->setBaudRate(configuration.baudRate);
    serialPort_->setDataBits(QSerialPort::Data8);
    serialPort_->setParity(QSerialPort::NoParity);
    serialPort_->setStopBits(QSerialPort::OneStop);
    serialPort_->setFlowControl(QSerialPort::NoFlowControl);
    if (!serialPort_->open(QIODevice::ReadWrite)) {
        emit errorOccurred(QStringLiteral("Failed to open %1: %2")
                               .arg(configuration.endpoint, serialPort_->errorString()));
        emit stateChanged(TransportState::Error);
        return;
    }
    emit stateChanged(TransportState::Connected);
}

void SerialTransport::close()
{
    emit stateChanged(TransportState::Closing);
    if (serialPort_->isOpen()) {
        serialPort_->flush();
        if (serialPort_->bytesToWrite() > 0 && !serialPort_->waitForBytesWritten(100)) {
            emit errorOccurred(QStringLiteral("Serial close before all pending bytes were confirmed written: %1")
                                   .arg(serialPort_->errorString()));
        }
        serialPort_->close();
    }
    emit stateChanged(TransportState::Disconnected);
}

bool SerialTransport::write(const QByteArray &bytes)
{
    if (!serialPort_->isOpen()) {
        emit errorOccurred(QStringLiteral("Serial write rejected: port is not open"));
        return false;
    }
    const qint64 accepted = serialPort_->write(bytes);
    if (accepted != bytes.size()) {
        emit errorOccurred(QStringLiteral("Serial write accepted %1 of %2 bytes")
                               .arg(accepted)
                               .arg(bytes.size()));
        return false;
    }
    return true;
}

QStringList SerialTransport::availablePortNames()
{
    QStringList names;
    for (const QSerialPortInfo &port : QSerialPortInfo::availablePorts()) {
        names.append(port.portName());
    }
    names.sort(Qt::CaseInsensitive);
    return names;
}

} // namespace rb
