/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2018, 2019 Peter Palfraader
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "SkeletonStructure.h"
#include "InputGraphicsItem.h"
#include "KineticTriangulationGraphicsItem.h"
#include "OffsetsGraphicsItem.h"

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
    void on_actionVisToggleOffsets_triggered() { updateVisibilities(); };
    void on_actionVisToggleArcs_triggered() { updateVisibilities(); };
    void on_actionVisToggleHighlightCircle_triggered() { updateVisibilities(); };
    void on_actionResize_triggered();
    void on_actionUpdateOffsets_triggered();

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
    std::string skoffset;
    std::unique_ptr<Ui::MainWindow> ui;
    QGraphicsScene scene;
    QLabel* time_label;
    NT drawing_time_offset_increment;

    SkeletonStructure s;

    std::shared_ptr<InputGraphicsItem> input_gi;
    std::shared_ptr<KineticTriangulationGraphicsItem> kinetic_triangulation_gi;
    std::shared_ptr<OffsetsGraphicsItem> offsets_gi;

    void update_offsets();
    void updateVisibilities();
    void update_time_label();
    void time_changed();
    void simulation_has_finished();
};

#endif // MAINWINDOW_H
