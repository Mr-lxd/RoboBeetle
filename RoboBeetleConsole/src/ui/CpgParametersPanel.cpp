#include "ui/CpgParametersPanel.h"
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QMouseEvent>
#include <QPushButton>
#include <QVBoxLayout>
#include <cmath>
#include <functional>
namespace rb::ui {
class CouplingDiagram final : public QWidget
{
  public:
    explicit CouplingDiagram(QWidget *parent) : QWidget(parent)
    {
        setFixedSize(150, 120);
        setMouseTracking(true);
        setToolTip(QStringLiteral(
            "Solid: coupled / Dashed: uncoupled. Click a line to toggle both directions."));
    }
    quint8 mask{0x3c};
    std::function<void()> changed;

  private:
    std::array<QPointF, 4> nodes() const
    {
        return {QPointF(25, 20), QPointF(width() - 25, 20), QPointF(25, height() - 20),
                QPointF(width() - 25, height() - 20)};
    }
    static constexpr int pairs[6][2] = {{0, 1}, {2, 3}, {0, 2}, {1, 3}, {0, 3}, {1, 2}};
    int hit(QPointF pos) const
    {
        const auto n = nodes();
        int chosen = -1;
        for (int i = 0; i < 6; ++i)
        {
            const auto a = n[pairs[i][0]], d = n[pairs[i][1]] - a;
            const double t = QPointF::dotProduct(pos - a, d) / QPointF::dotProduct(d, d);
            if (t < .12 || t > .88)
                continue;
            const auto diff = pos - (a + t * d);
            if (std::hypot(diff.x(), diff.y()) <= 6)
            {
                if (chosen >= 0)
                    return -1;
                chosen = i;
            }
        }
        return chosen;
    }
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        const auto n = nodes();
        for (int i = 0; i < 6; ++i)
        {
            p.setPen(QPen(isEnabled() ? QColor("#405A6B") : QColor("#AABAC6"), 2,
                          (mask & (1 << i)) ? Qt::SolidLine : Qt::DashLine));
            p.drawLine(n[pairs[i][0]], n[pairs[i][1]]);
        }
        const char *names[]{"FL", "FR", "RL", "RR"};
        for (int i = 0; i < 4; ++i)
        {
            p.setBrush(palette().window());
            p.setPen(Qt::NoPen);
            p.drawEllipse(n[i], 15, 12);
            p.setPen(isEnabled() ? QColor("#294252") : QColor("#8A98A2"));
            p.drawText(QRectF(n[i] - QPointF(15, 12), QSizeF(30, 24)), Qt::AlignCenter,
                       QString::fromLatin1(names[i]));
        }
    }
    void mousePressEvent(QMouseEvent *e) override
    {
        if (e->button() != Qt::LeftButton)
            return;
        const auto i = hit(e->position());
        if (i < 0)
            return;
        mask ^= 1 << i;
        update();
        if (changed)
            changed();
    }
};
CpgParametersPanel::CpgParametersPanel(QWidget *parent)
    : QGroupBox(QStringLiteral("CPG parameters"), parent)
{
    setObjectName(QStringLiteral("cpgParametersPanel"));
    auto *grid = new QGridLayout(this);
    grid->setContentsMargins(8, 8, 8, 6);
    grid->setSpacing(4);
    const QString labels[]{
        QStringLiteral("Front amp (deg)"),        QStringLiteral("Rear amp (deg)"),
        QStringLiteral("Nominal period (s)"),     QStringLiteral("β"),
        QStringLiteral("Front–rear phase (deg)"), QStringLiteral("Left–right phase (deg)"),
        QStringLiteral("Coupling strength")};
    const double mins[]{0, 0, .5, .1, -180, -180, 0}, maxs[]{28, 30, 10, .9, 180, 180, 5};
    const int decimals[]{1, 1, 4, 2, 0, 0, 1};
    const double steps[]{.1, .1, .0001, .01, 1, 1, .1};
    for (int i = 0; i < 7; ++i)
    {
        auto *label = new QLabel(labels[i], this);
        fields_[i] = new QDoubleSpinBox(this);
        fields_[i]->setObjectName(QStringLiteral("cpgParameter%1").arg(i));
        fields_[i]->setRange(mins[i], maxs[i]);
        fields_[i]->setDecimals(decimals[i]);
        fields_[i]->setSingleStep(steps[i]);
        fields_[i]->setKeyboardTracking(false);
        if (i == 4 || i == 5)
            fields_[i]->setToolTip(
                QStringLiteral("Oscillator phase; independent of stroke direction"));
        // Labels sit beside their inputs. Left pair of columns: amplitudes and
        // period; right pair: beta, phases, coupling.
        const int row = i < 3 ? i : i - 3;
        const int column = i < 3 ? 1 : 3;
        grid->addWidget(label, row, column);
        grid->addWidget(fields_[i], row, column + 1);
        connect(fields_[i], qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] {
            if (!loading_)
            {
                editing_ = true;
                applied_->setText(QStringLiteral("Pending"));
            }
        });
    }
    diagram_ = new CouplingDiagram(this);
    diagram_->setObjectName(QStringLiteral("cpgCouplingDiagram"));
    grid->addWidget(diagram_, 0, 0, 4, 1, Qt::AlignLeft | Qt::AlignVCenter);
    grid->setColumnStretch(2, 1);
    grid->setColumnStretch(4, 1);
    diagram_->changed = [this] {
        editing_ = true;
        applied_->setText(QStringLiteral("Pending"));
    };
    grid->addWidget(new QLabel(QStringLiteral("Measured period (FL)"), this), 3, 1);
    period_ = new QLabel(QStringLiteral("--"), this);
    period_->setObjectName(QStringLiteral("cpgMeasuredPeriod"));
    period_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    grid->addWidget(period_, 3, 2);
    auto *appliedCell = new QWidget(this);
    auto *appliedLayout = new QHBoxLayout(appliedCell);
    appliedLayout->setContentsMargins(0, 0, 0, 0);
    appliedLayout->setSpacing(6);
    appliedLayout->addWidget(new QLabel(QStringLiteral("Applied:"), appliedCell));
    applied_ = new QLabel(QStringLiteral("Pending"), appliedCell);
    applied_->setWordWrap(true);
    applied_->setObjectName(QStringLiteral("cpgAppliedStatus"));
    appliedLayout->addWidget(applied_, 1);
    grid->addWidget(appliedCell, 4, 0);
    apply_ = new QPushButton(QStringLiteral("Apply"), this);
    apply_->setObjectName(QStringLiteral("cpgApplyButton"));
    reset_ = new QPushButton(QStringLiteral("Reset defaults"), this);
    reset_->setObjectName(QStringLiteral("cpgResetDefaultsButton"));
    for (auto *button : {apply_, reset_})
        button->setStyleSheet(
            QStringLiteral("QPushButton { min-height: 22px; max-height: 22px; padding: 2px 8px; }"));
    grid->addWidget(apply_, 4, 1, 1, 2);
    grid->addWidget(reset_, 4, 3, 1, 2);
    connect(apply_, &QPushButton::clicked, this, &CpgParametersPanel::applyRequested);
    connect(reset_, &QPushButton::clicked, this, [this] {
        setParameters({});
        editing_ = true;
        applied_->setText(QStringLiteral("Pending"));
    });
    setParameters({});
    apply_->setEnabled(false);
}
void CpgParametersPanel::setParameters(const IConsoleController::CpgParameters &p)
{
    loading_ = true;
    const double values[]{p.front_amp,        p.rear_amp,         p.nominal_period,   p.beta,
                          p.front_rear_phase, p.left_right_phase, p.coupling_strength};
    for (int i = 0; i < 7; ++i)
        fields_[i]->setValue(values[i]);
    diagram_->mask = p.coupling_mask;
    diagram_->update();
    loading_ = false;
}
IConsoleController::CpgParameters CpgParametersPanel::parameters() const
{
    return {fields_[0]->value(), fields_[1]->value(), fields_[2]->value(), fields_[3]->value(),
            fields_[4]->value(), fields_[5]->value(), fields_[6]->value(), diagram_->mask};
}
void CpgParametersPanel::refresh(const IConsoleController &c)
{
    const auto actual = c.cpgParameters();
    if (!actual)
        haveReadback_ = false;
    if (actual && !haveReadback_ && !editing_)
        setParameters(*actual);
    haveReadback_ = actual.has_value();
    const bool backend = c.confirmedGaitBackend() == GaitBackend::CPG;
    const bool supported = c.hasCpgSchema2() && c.cpgFeatureLevel() >= 1 && actual;
    const bool editable = backend && supported;
    const bool stopped = c.motionState() == MotionState::Stopped;
    for (auto *f : fields_)
        f->setEnabled(editable && stopped);
    diagram_->setEnabled(editable && stopped);
    reset_->setEnabled(editable && stopped);
    QString reason;
    if (!backend)
        reason = QStringLiteral("Select CPG to edit parameters");
    else if (!supported)
        reason = QStringLiteral("Waiting for firmware schema2 and parameter readback");
    else if (!c.isControlActive())
        reason = QStringLiteral("Acquire control first");
    else if (!stopped)
        reason = QStringLiteral("Stop motion before applying parameters");
    else if (c.isCpgParametersPending())
        reason = QStringLiteral("Waiting for firmware readback");
    apply_->setEnabled(reason.isEmpty());
    apply_->setToolTip(reason.isEmpty() ? QStringLiteral("Apply parameters in RAM") : reason);
    setToolTip(reason);
    if (!c.cpgParametersError().isEmpty())
        applied_->setText(QStringLiteral("Rejected: %1").arg(c.cpgParametersError()));
    else if (actual && parameters() == *actual && !c.isCpgParametersPending())
    {
        applied_->setText(QStringLiteral("Matches firmware"));
        editing_ = false;
    } else
        applied_->setText(QStringLiteral("Pending"));
    const auto period = c.measuredCpgPeriod();
    period_->setText(period ? QStringLiteral("%1 s%2").arg(*period, 0, 'f', 3)
                                .arg(c.motionState() == MotionState::Running
                                    ? QString{} : QStringLiteral(" (last)"))
                            : QStringLiteral("--"));
}
} // namespace rb::ui
