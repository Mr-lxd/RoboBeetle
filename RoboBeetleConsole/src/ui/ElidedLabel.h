#pragma once

#include <QLabel>

namespace rb::ui {

class ElidedLabel final : public QLabel {
    Q_OBJECT
    Q_PROPERTY(QString fullText READ fullText WRITE setFullText)

public:
    explicit ElidedLabel(QWidget *parent = nullptr);

    void setFullText(const QString &text);
    [[nodiscard]] QString fullText() const { return fullText_; }

    [[nodiscard]] QSize sizeHint() const override;
    [[nodiscard]] QSize minimumSizeHint() const override;

protected:
    void resizeEvent(QResizeEvent *event) override;
    void changeEvent(QEvent *event) override;

private:
    void updateElision();

    QString fullText_;
};

} // namespace rb::ui
