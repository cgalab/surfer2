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
#include "KineticTriangulationGraphicsItem.h"

#include <QPainter>

KineticTriangulationGraphicsItem::KineticTriangulationGraphicsItem(const KineticTriangulation * const p_kt, const NT& p_infinite_edges_extra_time)
  : Base()
  , kt(p_kt)
  , infinite_edges_extra_time(p_infinite_edges_extra_time)
  , vertices_pen(QPen(::Qt::darkGreen, 3))
  , vertices_reflex_pen(QPen(::Qt::darkRed, 3))
  , vertices_collinear_pen(QPen(::Qt::darkGray, 3))
  , edges_pen(QPen(Qt::gray, 0, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin))
  , constraints_pen(QPen(Qt::black, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , labels_pen(QPen(Qt::black, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , arcs_pen(QPen(Qt::blue, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , vertices_moving_pen(QPen(QColor("#5599ff"), 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , highlight_circle_pen(QPen(::Qt::darkRed, 3))
  , highlight_face_brush(QBrush(Qt::yellow))
  , now(0.0)
  , drawing_time_offset_(0.0)
{
  setZValue(3);
  modelChanged();
}

/** figure out if we want to draw this element depending on which component we currently handle */
bool
KineticTriangulationGraphicsItem::
filter_component(const KineticTriangle& t) const {
  if (kt->restrict_component() >= 0 || current_component < 0) {
    return true;
  } else {
    return t.component == current_component;
  }
}

/** figure out if we want to draw this element depending on which component we currently handle */
bool
KineticTriangulationGraphicsItem::
filter_component(const WavefrontVertex& v) const {
  const WavefrontEdge * edge = v.incident_wavefront_edge(0);
  assert(edge);
  const KineticTriangle * t = edge->incident_triangle();
  assert(t);
  return filter_component(*t);
}


void
KineticTriangulationGraphicsItem::
paint_arcs(QPainter *painter, PainterOstream painterostream) const {
  if (! visible_arcs) {
    return;
  }

  for (auto i = kt->vertices_begin(); i != kt->vertices_end(); ++i) {
    if (i->is_infinite) continue;
    if (draw_time() < i->time_start) continue;
    if (! filter_component(*i) ) continue;
    painter->setPen(
      i->has_stopped() ? arcsPen()
                       : verticesMovingPen() );
    const Point_2 pu(i->pos_start);
    const Point_2 pv(i->p_at_draw(draw_time()));
    painterostream << Segment_2(pu, pv);
  }
}

void
KineticTriangulationGraphicsItem::
paint_highlighted(QPainter *painter, PainterOstream painterostream) const {
  if (! visible_edges) {
    return;
  };

  Converter convert;
  auto transform = painter->worldTransform();
  painter->resetTransform();

  for (auto highlithed_it = highlighted.begin(); highlithed_it != highlighted.end(); ++highlithed_it) {
    const KineticTriangle* t = *highlithed_it;
    if (t->is_dead()) continue;
    //if (! highlighted_has(&*t)) continue;

    Point_2 cp[4];
    unsigned num_pts;
    if (t->unbounded()) {
      unsigned idx = t->infinite_vertex_idx();
      const WavefrontVertex * const u = t->vertex(t->ccw(idx));
      const WavefrontVertex * const v = t->vertex(t->cw (idx));
      cp[0] = u->p_at_draw(draw_time() + infinite_edges_extra_time);
      cp[1] = u->p_at_draw(draw_time()                            );
      cp[2] = v->p_at_draw(draw_time()                            );
      cp[3] = v->p_at_draw(draw_time() + infinite_edges_extra_time);
      num_pts = 4;
    } else {
      num_pts = 3;
      for (unsigned i=0; i<num_pts; ++i) {
        cp[i] = t->vertex(i)->p_at_draw(draw_time());
      }
    }

    QPainterPath path;
    for (unsigned i=0; i<num_pts; ++i) {
      const QPointF qp = transform.map(convert(cp[i]));
      if (i==0) { path.moveTo( qp.x(), qp.y() ); }
           else { path.lineTo( qp.x(), qp.y() ); }
    }
    painter->setPen (Qt :: NoPen);
    painter->fillPath(path, highlight_face_brush);

    if (visible_highlight_circle) {
      painter->setPen(highlightCirclePen());
      Point_2 p = triangle_center(*t);
      const QPointF qp = transform.map(convert(p));
      painter->drawEllipse(qp, highlightCircleRadius, highlightCircleRadius);
    }
  }

  painter->setWorldTransform(transform);
}

void
KineticTriangulationGraphicsItem::
paint_vertices(QPainter *painter, PainterOstream painterostream) const {
  if (! (visible_arcs || visible_constraints)) {
    return;
  }

  Converter convert;
  auto transform = painter->worldTransform();
  painter->resetTransform();

  for (auto i = kt->vertices_begin(); i != kt->vertices_end(); ++i) {
    if (i->is_infinite) continue;
    if (draw_time() < i->time_start) continue;
    if (! filter_component(*i) ) continue;
    if ((i->has_stopped() && visible_arcs) ||
        (!i->has_stopped() && visible_constraints)) {

      switch (i->angle) {
        case WavefrontVertex::CONVEX:
          painter->setPen(verticesPen());
          break;
        case WavefrontVertex::STRAIGHT:
          painter->setPen(verticesCollinearPen());
          break;
        case WavefrontVertex::REFLEX:
          painter->setPen(verticesReflexPen());
          break;
      }
      const Point_2 p(i->p_at_draw(draw_time()));
      QPointF qp = transform.map(convert(p));
      painter->drawPoint(qp);
    }
  }

  painter->setWorldTransform(transform);
}

/** paints constraints (wavefront edges) and triangulation spokes. */
void
KineticTriangulationGraphicsItem::
paint_edges(QPainter *painter, PainterOstream painterostream) const {
  for (auto t = kt->triangles_begin(); t != kt->triangles_end(); ++t) {
    if (t->is_dead()) continue;
    if (! filter_component(*t) ) continue;
    for (unsigned i=0; i<3; ++i) {
      const WavefrontVertex * const u = t->vertex(t->ccw(i));
      const WavefrontVertex * const v = t->vertex(t->cw (i));

      assert(!u->is_infinite || !v->is_infinite);

      // edge or constraint across i
      if (t->is_constrained(i) && visible_constraints) {
        painter->setPen(constraintsPen());
      } else if (t->is_constrained(i) && visible_edges) {
        painter->setPen(edgesPen());
      } else if (visible_edges && !t->is_constrained(i) && u < v) {
        painter->setPen(edgesPen());
      } else {
        continue;
      }
      const Point_2 pu = !u->is_infinite ? u->p_at_draw(draw_time()) : v->p_at_draw(draw_time() + infinite_edges_extra_time);
      const Point_2 pv = !v->is_infinite ? v->p_at_draw(draw_time()) : u->p_at_draw(draw_time() + infinite_edges_extra_time);
      painterostream << Segment_2(pu, pv);
    }
  }
}

Point_2
KineticTriangulationGraphicsItem::
triangle_center(const KineticTriangle& t) const {
  if (t.unbounded()) {
    unsigned idx = t.infinite_vertex_idx();
    const WavefrontVertex * const u = t.vertex(t.ccw(idx));
    const WavefrontVertex * const v = t.vertex(t.cw (idx));

    return CGAL::midpoint(u->p_at_draw(draw_time()+infinite_edges_extra_time), v->p_at_draw(draw_time()+infinite_edges_extra_time));
  } else {
    return CGAL::centroid(
      t.vertex(0)->p_at_draw(draw_time()),
      t.vertex(1)->p_at_draw(draw_time()),
      t.vertex(2)->p_at_draw(draw_time())
      );
  }
}

void
KineticTriangulationGraphicsItem::
paint_labels(QPainter *painter, PainterOstream painterostream) const {
  if (! visible_labels) {
    return;
  }

  Converter convert;
  auto transform = painter->worldTransform();
  painter->resetTransform();

  painter->setPen(labelsPen());
  QFont font(painter->font());

  font.setPointSize(8);
  painter->setFont(font);
  #ifndef SURF_NDEBUG
  for (auto i = kt->vertices_begin(); i != kt->vertices_end(); ++i) {
    if (i->is_infinite) continue;
    if (! filter_component(*i) ) continue;
    if (draw_time() < i->time_start) continue;
    const Point_2 p(i->p_at_draw(draw_time()));
    QPointF qp = transform.map(convert(p));
    std::string t = "kv#"+std::to_string(i->id);
    painter->drawText(qp.x()+4, qp.y(), QString::fromStdString(t));
  }
  #endif

  for (auto t = kt->triangles_begin(); t != kt->triangles_end(); ++t) {
    if (t->is_dead()) continue;
    if (! filter_component(*t) ) continue;
    std::string label = t->get_name();

    Point_2 p = triangle_center(*t);
    const QPointF qp = transform.map(convert(p));
    painter->drawText(qp, QString::fromStdString(label));
  }

  painter->setWorldTransform(transform);
}



void
KineticTriangulationGraphicsItem::
paint(QPainter *painter, const QStyleOptionGraphicsItem * option, QWidget * widget) {
  PainterOstream painterostream(painter);

  paint_arcs(painter, painterostream);
  paint_highlighted(painter, painterostream);
  paint_edges(painter, painterostream);
  paint_vertices(painter, painterostream);
  paint_labels(painter, painterostream);
}

void
KineticTriangulationGraphicsItem::
updateBoundingBox() {
  GConverter convert;
  prepareGeometryChange();

  bool first = true;
  CGAL::Bbox_2 bb;
  if (kt->vertices_begin() == kt->vertices_end()) return;

  for (auto i = kt->vertices_begin(); i != kt->vertices_end(); ++i) {
    if (i->is_infinite) continue;
    if (first) {
      bb = GuiPoint(i->pos_start).bbox();
      first = false;
    }

    bb += GuiPoint(i->p_at_draw(draw_time())).bbox();
  }
  if (first) {
    return;
  }

  for (auto t = kt->triangles_begin(); t != kt->triangles_end(); ++t) {
    if (t->is_dead()) continue;
    if (! t->unbounded()) continue;

    unsigned idx = t->infinite_vertex_idx();
    const WavefrontVertex * const u = t->vertex(t->ccw(idx));
    // const WavefrontVertex * const v = t->vertex(t->cw (idx));

    bb += GuiPoint(u->p_at_draw(draw_time()+infinite_edges_extra_time)).bbox();
    // bb += v->p_at_draw(draw_time()+infinite_edges_extra_time).bbox();
  }

  //std::cout << "bb kt " << bb
  //  << std::endl;
  bounding_rect = convert(bb);
}

void
KineticTriangulationGraphicsItem::
modelChanged() {
  prepareGeometryChange();
  updateBoundingBox();
}
