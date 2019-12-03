#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "SkeletonStructure.h"
#include "InputGraphicsItem.h"
#include "KineticTriangulationGraphicsItem.h"

#include <QMainWindow>
#include <CGAL/Qt/DemosMainWindow.h>
#include <QGraphicsScene>
#include <QLabel>

namespace Ui {
  class MainWindow;
}

class MainWindow : public CGAL::Qt::DemosMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(
      std::string title,
      std::istream& is,
      unsigned skip_to=0,
      bool skip_all=false,
      std::string skip_until_time="",
      std::string skoffset="",
      int restrict_component = -1);
    ~MainWindow();

  private:
    bool first_show_event = true;
    bool did_finish = false;
  private slots:
    void showEvent(QShowEvent *);
    void mousePressEvent(QMouseEvent *event);

    void on_actionQuit_triggered() { close(); };
    void on_actionVisToggleInput_triggered() { updateVisibilities(); };
    void on_actionVisToggleInputLabels_triggered() { updateVisibilities(); };
    void on_actionVisToggleWavefront_triggered() { updateVisibilities(); };
    void on_actionVisToggleKineticTriangulation_triggered() { updateVisibilities(); };
    void on_actionVisToggleKineticTriangulationLabels_triggered() { updateVisibilities(); };
    void on_actionVisToggleArcs_triggered() { updateVisibilities(); };
    void on_actionVisToggleHighlightCircle_triggered() { updateVisibilities(); };
    void on_actionResize_triggered();

    void on_actionTimeBackward_triggered();
    void on_actionTimeForward_triggered();
    void on_actionTimeForwardThrough_triggered();
    void on_actionTimeForwardNext_triggered();
    void on_actionTimeReset_triggered();
    void on_actionEventStep_triggered();
    void on_actionEventStepEnd_triggered();

    void on_actionTimeOffsetMinus_triggered();
    void on_actionTimeOffsetPlus_triggered();
    void on_actionTimeOffsetReset_triggered();

  private:
    std::string title;
    std::unique_ptr<Ui::MainWindow> ui;
    QGraphicsScene scene;
    QLabel* time_label;
    NT drawing_time_offset_increment;

    SkeletonStructure s;

    std::shared_ptr<InputGraphicsItem> input_gi;
    std::shared_ptr<KineticTriangulationGraphicsItem> kinetic_triangulation_gi;

    void updateVisibilities();
    void update_time_label();
    void time_changed();
    void simulation_has_finished();
};

#endif // MAINWINDOW_H
