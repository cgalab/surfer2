#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "BGLGraph.h"

#include <QInputDialog>

MainWindow::MainWindow(std::string p_title, std::istream& is, unsigned skip_to, bool skip_all, std::string skip_until_time, std::string skoffset, int restrict_component) :
  CGAL::Qt::DemosMainWindow(),
  title(p_title),
  ui(new Ui::MainWindow)
{
  const NT skip_until_t(skip_until_time == "" ? CORE_ZERO : string_to_maybe_NT(skip_until_time));
  (void) skoffset; // XXX unused;

  ui->setupUi(this);
  setWindowTitle(QString::fromStdString(title));
  scene.setItemIndexMethod(QGraphicsScene::NoIndex);
  ui->gV->setScene(&scene);
  ui->gV->setMouseTracking(true);
  ui->gV->scale(1, -1);
  ui->gV->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
  ui->gV->setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
  //addNavigation(ui->gV);

  /* add navigation */
  navigation = new MyGraphicsViewNavigation();
  ui->gV->viewport()->installEventFilter(navigation);
  ui->gV->installEventFilter(navigation);
  QObject::connect(navigation, SIGNAL(mouseCoordinates(QString)),
                   xycoord, SLOT(setText(QString)));
  this->view = ui->gV;



  BGLGraph graph;
  graph = BGLGraph::create_from_graphml(is);
  s.add_graph(graph);
  s.initialize(restrict_component);

  input_gi = std::make_shared<InputGraphicsItem>(&s.get_input());
  scene.addItem(input_gi.get());

  auto input_size = input_gi->boundingRect().size();
  auto size_avg = (input_size.width() + input_size.height() ) /2.0;
  s.wp.set_increment(size_avg/5000.);
  drawing_time_offset_increment = size_avg/5000.;

  kinetic_triangulation_gi = std::make_shared<KineticTriangulationGraphicsItem>(&s.get_kt(), size_avg/200.);
  scene.addItem(kinetic_triangulation_gi.get());

  time_label = new QLabel("", this);
  update_time_label();
  time_label->setAlignment(::Qt::AlignHCenter);
  time_label->setMinimumSize(time_label->sizeHint());
  ui->statusBar->addWidget(new QLabel("C-S-<right button: center; C-<right button>: drag; C-<left buttom>: select-zoom"), 1);
  ui->statusBar->addWidget(new QLabel(this), 1);
  ui->statusBar->addWidget(time_label, 0);
  ui->statusBar->addWidget(xycoord, 0);

  s.wp.do_initial_skips(skip_all, skip_to, skip_until_t);
  time_changed();
}

MainWindow::~MainWindow()
{
}

void
MainWindow::updateVisibilities() {
  input_gi->setVisibleLabels(ui->actionVisToggleInputLabels->isChecked());
  input_gi->setVisible(ui->actionVisToggleInput->isChecked());
/*
  triangulation_gi->setVisible(ui->actionVisToggleTriangulation->isChecked());
*/
  kinetic_triangulation_gi->setVisible(ui->actionVisToggleKineticTriangulation->isChecked() ||
                                       ui->actionVisToggleWavefront->isChecked() ||
                                       ui->actionVisToggleArcs->isChecked());
  kinetic_triangulation_gi->setVisibleEdges(ui->actionVisToggleKineticTriangulation->isChecked());
  kinetic_triangulation_gi->setVisibleConstraints(ui->actionVisToggleWavefront->isChecked());
  kinetic_triangulation_gi->setVisibleLabels(ui->actionVisToggleKineticTriangulationLabels->isChecked());
  kinetic_triangulation_gi->setVisibleArcs(ui->actionVisToggleArcs->isChecked());
  kinetic_triangulation_gi->setVisibleHighlightCircle(ui->actionVisToggleHighlightCircle->isChecked());
/*
  skeleton_gi->setVisible(ui->actionVisToggleSkeleton->isChecked());
  offset_gi->setVisible(ui->actionVisToggleOffset->isChecked());
*/
}

void
MainWindow::on_actionResize_triggered() {
  auto br = input_gi->boundingRect();
  br |= kinetic_triangulation_gi->boundingRect();
/*
  if (instance_triangulation_gi) {
    br |= instance_triangulation_gi->boundingRect();
  };
  br |= skeleton_gi->boundingRect();
  br |= offset_gi->boundingRect();
*/

  ui->gV->setSceneRect(br);
  ui->gV->fitInView(br, Qt::KeepAspectRatio);
}

void
MainWindow::
showEvent(QShowEvent *) {
  if (first_show_event) {
    on_actionResize_triggered();
    first_show_event = false;
  }
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
  (void) event;

  // Kernel::Point_2 p( ui->gV->getMousePoint().x(),  ui->gV->getMousePoint().y() );
  // XXX s.kinetic_triangulation->showInfoAt(p);
}

void
MainWindow::
update_time_label() {
  auto t = CGAL::to_double( s.wp.get_time() );
  time_label->setText(QString("e#%1; t: %2 (%3 %4); ").
    arg(s.wp.event_ctr()).
    arg(t , 9, 'f', 5).
    arg((kinetic_triangulation_gi->drawing_time_offset() < 0) ? "-" : "+" ).
    arg(abs(CGAL::to_double(kinetic_triangulation_gi->drawing_time_offset())), 9, 'f', 5)
    );
}

void
MainWindow::
time_changed() {
  if (s.wp.propagation_complete()) {
    simulation_has_finished();
  }
  update_time_label();
  kinetic_triangulation_gi->setTime(s.wp.get_time());
  kinetic_triangulation_gi->setComponent(s.wp.get_current_component());
  kinetic_triangulation_gi->highlighted_clear();
  if (!s.wp.propagation_complete()) {
    const std::shared_ptr<const EventQueueItem> next = s.wp.peak();
    kinetic_triangulation_gi->highlighted_add(next->get_priority().t);
  }
}

void
MainWindow::on_actionTimeBackward_triggered() {
  s.wp.reverse_time(); // b - move back in time
  time_changed();
}

void
MainWindow::on_actionTimeForwardThrough_triggered() {
  s.wp.advance_time_ignore_event(); // M - Move forward in time, but ignore any event that may have happened
  time_changed();
}

void
MainWindow::on_actionTimeForward_triggered() {
  s.wp.advance_time(); // N - Move forward in time by the increment, or until the next event and handle it
  time_changed();
}

void
MainWindow::on_actionTimeForwardNext_triggered() {
  s.wp.advance_time_next(); // , - Move forward in time to the next event but do not handle it yet
  time_changed();
}

void
MainWindow::on_actionTimeReset_triggered() {
  s.wp.reset_time_to_last_event(); // backspace -- reset to last event time
  time_changed();
}

void
MainWindow::on_actionEventStep_triggered() {
  s.wp.advance_step(); // n - Move forward in time to the next event and handle it
  time_changed();
}

void
MainWindow::on_actionEventStepEnd_triggered() {
  s.wp.advance_to_end();

  auto size = kinetic_triangulation_gi->boundingRect().size();
  auto size_avg = (size.width() + size.height() ) /2.0;
  s.wp.advance_time_ignore_event(s.wp.get_time() + size_avg/5.);

  time_changed();
}

void
MainWindow::on_actionTimeOffsetMinus_triggered() {
  kinetic_triangulation_gi->setDrawingOffset(kinetic_triangulation_gi->drawing_time_offset() - drawing_time_offset_increment);
  time_changed();
}
void
MainWindow::on_actionTimeOffsetPlus_triggered() {
  kinetic_triangulation_gi->setDrawingOffset(kinetic_triangulation_gi->drawing_time_offset() + drawing_time_offset_increment);
  time_changed();
}
void
MainWindow::on_actionTimeOffsetReset_triggered() {
  kinetic_triangulation_gi->setDrawingOffset(CORE_ZERO);
  time_changed();
}

void
MainWindow::simulation_has_finished() {
  if (did_finish) return;
  did_finish = true;
  ui->actionVisToggleWavefront->setChecked(false);
  ui->actionVisToggleKineticTriangulation->setChecked(false);
  updateVisibilities();
}

