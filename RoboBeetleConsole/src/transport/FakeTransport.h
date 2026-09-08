#pragma once

#include "transport/ITransport.h"

#include <QList>

#include <functional>

namespace rb {

class FakeTransport final : public ITransport {
    Q_OBJECT

public:
    explicit FakeTransport(QObject *parent = nullptr);

    void open(const TransportConfiguration &configuration) override;
    void close() override;
    bool write(const QByteArray &bytes) override;

    void simulateConnected();
    void simulateError(const QString &message);
    void injectBytes(const QByteArray &bytes);
    void setWriteSucceeds(bool succeeds);
    void setWriteErrorSignals(bool emitsError);
    void setWriteCallback(std::function<void(const QByteArray &)> callback);

    [[nodiscard]] const QList<QByteArray> &writes() const { return writes_; }
    [[nodiscard]] int closeCallCount() const { return closeCallCount_; }
    [[nodiscard]] TransportConfiguration lastConfiguration() const { return lastConfiguration_; }

private:
    QList<QByteArray> writes_;
    TransportConfiguration lastConfiguration_;
    int closeCallCount_{0};
    bool writeSucceeds_{true};
    bool writeErrorSignals_{true};
    std::function<void(const QByteArray &)> writeCallback_;
};

} // namespace rb
