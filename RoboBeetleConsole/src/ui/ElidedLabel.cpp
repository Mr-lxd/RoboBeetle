#include "ui/ElidedLabel.h"

#include <QFontMetrics>
#include <QResizeEvent>

namespace rb::ui {

ElidedLabel::ElidedLabel(QWidget *parent)
    : QLabel(parent)
{
    setTextFormat(Qt::PlainText);
    setWordWrap(false);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    setMinimumWidth(0);
    setAccessibleDescription(QStringLiteral("Single-line diagnostic text"));
}

void ElidedLabel::setFullText(const QString &text)
{
    if (fullText_ == text) {
        setVisible(!text.isEmpty());
        return;
    }
    fullText_ = text;
    setToolTip(fullText_);
    setVisible(!fullText_.isEmpty());
    updateElision();
}

QSize ElidedLabel::sizeHint() const
{
    const QFontMetrics metrics(font());
    return QSize(qMin(metrics.horizontalAdvance(QStringLiteral("0000-00-00 00:00")) + 12,
                      qMax(80, metrics.horizontalAdvance(fullText_) + 12)),
                 metrics.lineSpacing() + 6);
}

QSize ElidedLabel::minimumSizeHint() const
{
    const QFontMetrics metrics(font());
    return QSize(0, metrics.lineSpacing() + 4);
}

void ElidedLabel::resizeEvent(QResizeEvent *event)
{
    QLabel::resizeEvent(event);
    updateElision();
}

void ElidedLabel::changeEvent(QEvent *event)
{
    QLabel::changeEvent(event);
    if (event->type() == QEvent::FontChange
        || event->type() == QEvent::StyleChange
        || event->type() == QEvent::ApplicationFontChange) {
        updateElision();
    }
}

void ElidedLabel::updateElision()
{
    const QFontMetrics metrics(font());
    const int available = qMax(0, contentsRect().width());
    QLabel::setText(metrics.elidedText(fullText_, Qt::ElideRight, available));
    if (toolTip() != fullText_) {
        setToolTip(fullText_);
    }
}

} // namespace rb::ui
