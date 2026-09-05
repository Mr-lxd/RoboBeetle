#pragma once

#include "transport/ITransport.h"

#include <QStringList>

class QSerialPort;

namespace rb {

class SerialTransport final : public ITransport {
    Q_OBJECT

public:
    explicit SerialTransport(QObject *parent = nullptr);
    ~SerialTransport() override;

    void open(const TransportConfiguration &configuration) override;
    void close() override;
    bool write(const QByteArray &bytes) override;

    static QStringList availablePortNames();

private:
    QSerialPort *serialPort_;
};

} // namespace rb

