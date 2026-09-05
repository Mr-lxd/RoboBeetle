#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>

namespace rb {

struct TransportConfiguration {
    QString endpoint;
    qint32 baudRate{0};
};

enum class TransportState {
    Disconnected,
    Opening,
    Connected,
    Closing,
    Error,
};

class ITransport : public QObject {
    Q_OBJECT

public:
    explicit ITransport(QObject *parent = nullptr) : QObject(parent) {}
    ~ITransport() override = default;

    virtual void open(const TransportConfiguration &configuration) = 0;
    virtual void close() = 0;
    virtual bool write(const QByteArray &bytes) = 0;

signals:
    void bytesReceived(const QByteArray &bytes);
    void stateChanged(rb::TransportState state);
    void errorOccurred(const QString &message);
};

} // namespace rb

Q_DECLARE_METATYPE(rb::TransportState)

