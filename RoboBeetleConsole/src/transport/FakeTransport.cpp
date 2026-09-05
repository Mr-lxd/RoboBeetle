#include "transport/FakeTransport.h"

namespace rb {

FakeTransport::FakeTransport(QObject *parent) : ITransport(parent) {}

void FakeTransport::open(const TransportConfiguration &configuration)
{
    lastConfiguration_ = configuration;
    emit stateChanged(TransportState::Opening);
}

void FakeTransport::close()
{
    ++closeCallCount_;
    emit stateChanged(TransportState::Closing);
    emit stateChanged(TransportState::Disconnected);
}

bool FakeTransport::write(const QByteArray &bytes)
{
    if (!writeSucceeds_) {
        emit errorOccurred(QStringLiteral("Fake transport write failure"));
        return false;
    }
    writes_.append(bytes);
    return true;
}

void FakeTransport::simulateConnected()
{
    emit stateChanged(TransportState::Connected);
}

void FakeTransport::simulateError(const QString &message)
{
    emit errorOccurred(message);
    emit stateChanged(TransportState::Error);
}

void FakeTransport::injectBytes(const QByteArray &bytes)
{
    emit bytesReceived(bytes);
}

void FakeTransport::setWriteSucceeds(bool succeeds)
{
    writeSucceeds_ = succeeds;
}

} // namespace rb
