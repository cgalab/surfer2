/*
 * Copyright (c) 2015 Peter Palfrader
 *
 * All Rights reserved.
 */

#ifndef _MYQTTHINGS_H_
#define _MYQTTHINGS_H_

#include <QGraphicsView>
#include <QMouseEvent>
#include <QApplication>

#include <CGAL/Qt/GraphicsViewNavigation.h>

class MyQGraphicsView : public QGraphicsView {
  using QGraphicsView::QGraphicsView;

  private:
    QPointF mousePoint;

    void mousePressEvent(QMouseEvent *event) {
      (void) event;
      /*
      std::cout << "QGraphicsView::mousePressEvent" << std::endl;
      std::cout << " rect     (*): ("
        << sceneRect().x() << ","
        << sceneRect().y() << ","
        << sceneRect().x() + sceneRect().width() << ","
        << sceneRect().y() + sceneRect().width() << ")"
        << std::endl;
       */
      //LOG(DEBUG) << "In MyQGraphicsView::mousePressEvent()";
      //LOG(DEBUG) << " eventPos: (" << event->pos().x() << ", " << event->pos().y() << ")";

      mousePoint = mapToScene( event->pos() );
      //LOG(DEBUG) << " mapToScene: (" << mousePoint.x() << ", " << mousePoint.y() << ")";

      event->ignore();
    }
  public:
    const QPointF& getMousePoint() { return mousePoint; };
};

class MyGraphicsViewNavigation : public CGAL::Qt::GraphicsViewNavigation {
  private:
    using Base = CGAL::Qt::GraphicsViewNavigation;
  public:
    inline void update_mouse(const QPointF &pos) {
      QString xy = QString(" ") + QString::number(pos.x(),'g', 6) + " , " + QString::number(pos.y(),'g', 6) + " ";
      Q_EMIT mouseCoordinates(xy);
    }

    inline bool eventFilter(QObject *obj, QEvent *event) {
      if (event->type() != QEvent::Wheel) {
        return Base::eventFilter(obj, event);
      }

      // std::cout << "MyGraphicsViewNavigation::eventFilter - QEvent::Wheel" << std::endl;
      QGraphicsView* v = qobject_cast<QGraphicsView*>(obj);
      if (v == NULL) {
        QWidget* viewport = qobject_cast<QWidget*>(obj);
        if (viewport == NULL) {
          return false;
        }
        v = qobject_cast<QGraphicsView*>(viewport->parent());
        if(v == NULL) {
          return false;
        }
      }

      QWheelEvent *wheel_event = static_cast<QWheelEvent*>(event);
      if (wheel_event->orientation() != ::Qt::Vertical) {
        return false;
      }


      /* We want to zoom in/out while keeping the object under the mouse
       * under the mouse.
       *
       * Unfortunately, CGAL zooms the viewport center, and it hides
       * its helpers in private methods.
       *
       * So we do the following:
       *   (1) we send an object to put the object under the mouse in the center
       *   (2) then, we scale correctly (different wheel direction than CGAL too)
       *   (3) then we move things back.  We compute how much in scene coordinates
       *       the location under the mouse has moved, and add that offset to
       *       the new scene center, then we translate that back to
       *       mouse/screen coordinates and put that in the center.
       */
      /* 1) put object under the mouse in the center */
      const auto mouse_pos = wheel_event->pos();
      const auto scene_pos_pre = v->mapToScene(mouse_pos);
      QMouseEvent move_click_pos_to_center(
        QEvent::MouseButtonPress,
        mouse_pos,
        ::Qt::RightButton,
        ::Qt::RightButton,
        (::Qt::ControlModifier | ::Qt::ShiftModifier));
      Base::eventFilter(obj, &move_click_pos_to_center);

      /* 2) zoom */
      double zoom_ratio = 120.0;
      qreal scaleFactor = pow((double)2, wheel_event->delta() / zoom_ratio);
      v->scale(scaleFactor, scaleFactor);

      /* 3) translate back */
      const auto scene_pos_post = v->mapToScene(mouse_pos);
      const auto scene_offset = scene_pos_pre - scene_pos_post;
      const auto scene_center = v->mapToScene(v->viewport()->rect().center());
      const auto center_on = v->mapFromScene(scene_center + scene_offset);
      QMouseEvent move_click_back(
        QEvent::MouseButtonPress,
        center_on,
        ::Qt::RightButton,
        ::Qt::RightButton,
        (::Qt::ControlModifier | ::Qt::ShiftModifier));
      Base::eventFilter(obj, &move_click_back);

      update_mouse(v->mapToScene(wheel_event->pos()));
      event->accept();

      return true;
    }

};
#endif /* _MYQTTHINGS_H_ */
