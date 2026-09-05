#pragma once

#include <QByteArrayView>
#include <QtGlobal>

namespace rb {

quint16 crc16CcittFalse(QByteArrayView data);

} // namespace rb

