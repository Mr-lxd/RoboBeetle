#pragma once
#include "controller/IConsoleController.h"
#include <QGroupBox>
#include <array>
class QDoubleSpinBox;
class QLabel;
class QPushButton;
namespace rb::ui {
class CouplingDiagram;
class CpgParametersPanel final : public QGroupBox
{
    Q_OBJECT
  public:
    explicit CpgParametersPanel(QWidget *parent = nullptr);
    void refresh(const IConsoleController &);
    IConsoleController::CpgParameters parameters() const;
  signals:
    void applyRequested();

  private:
    void setParameters(const IConsoleController::CpgParameters &);
    std::array<QDoubleSpinBox *, 7> fields_{};
    CouplingDiagram *diagram_{};
    QLabel *applied_{}, *period_{};
    QPushButton *apply_{}, *reset_{};
    bool editing_{false}, haveReadback_{false}, loading_{false};
};
} // namespace rb::ui
