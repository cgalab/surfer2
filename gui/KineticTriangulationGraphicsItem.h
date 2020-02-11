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
#pragma once

#include "KineticTriangulation.h"
#include "TriangulationUtils.h"

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include <QBrush>
#include <unordered_set>
#include <tuple>

class KineticTriangulationGraphicsItem :
  public CGAL::Qt::GraphicsItem
{
  private:
    using Base = CGAL::Qt::GraphicsItem;
    using PainterOstream = CGAL::Qt::PainterOstream<Kernel>;

  private:
    static inline int cw (int i) { return TriangulationUtils::cw (i); }
    static inline int ccw(int i) { return TriangulationUtils::ccw(i); }

    const KineticTriangulation * const kt;
    const NT infinite_edges_extra_time;

    QPen vertices_pen;
    QPen vertices_reflex_pen;
    QPen vertices_collinear_pen;
    QPen edges_pen;
    QPen constraints_pen;
    QPen labels_pen;
    QPen arcs_pen;
    QPen vertices_moving_pen;
    QPen highlight_circle_pen;
    QBrush highlight_face_brush;
    bool visible_labels = false;
    bool visible_edges = true;
    bool visible_constraints = true;
    bool visible_arcs = true;
    bool visible_highlight_circle = false;

    NT now;
    NT drawing_time_offset_;
    std::unordered_set<const KineticTriangle *> highlighted;

    Point_2 triangle_center(const KineticTriangle& t) const;
  public:
    qreal highlightCircleRadius = 30;

  protected:
    QRectF bounding_rect;
    void updateBoundingBox();

  private:
    bool filter_component(const KineticTriangle& t) const;
    bool filter_component(const WavefrontVertex& v) const;
    /* If -1, we want to show them all.  If >= 0, only show this component's elements. */
    int current_component = -1;
  public:
    KineticTriangulationGraphicsItem(const KineticTriangulation * const kt, const NT& infinite_edges_extra_time);

    QRectF boundingRect() const { return bounding_rect; };

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void paint_arcs(QPainter *painter, PainterOstream painterostream) const;
    void paint_highlighted(QPainter *painter, PainterOstream painterostream) const;
    void paint_vertices(QPainter *painter, PainterOstream painterostream) const;
    void paint_edges(QPainter *painter, PainterOstream painterostream) const;
    void paint_labels(QPainter *painter, PainterOstream painterostream) const;

    void setVerticesPen(const QPen& pen) { vertices_pen = pen; };
    void setVerticesReflexPen(const QPen& pen) { vertices_pen = pen; };
    void setEdgesPen(const QPen& pen) { edges_pen = pen; };
    void setConstraintsPen(const QPen& pen) { constraints_pen = pen; };
    void setLabelsPen(const QPen& pen) { labels_pen = pen; };
    void setArcsPen(const QPen& pen) { arcs_pen = pen; };
    void setVerticesMovingPen(const QPen& pen) { vertices_moving_pen = pen; };
    const QPen& verticesPen() const { return vertices_pen; }
    const QPen& verticesReflexPen() const { return vertices_reflex_pen; }
    const QPen& verticesCollinearPen() const { return vertices_collinear_pen; }
    const QPen& edgesPen() const { return edges_pen; }
    const QPen& constraintsPen() const { return constraints_pen; }
    const QPen& labelsPen() const { return labels_pen; }
    const QPen& arcsPen() const { return arcs_pen; }
    const QPen& verticesMovingPen() const { return vertices_moving_pen; }
    const QPen& highlightCirclePen() const { return highlight_circle_pen; }
    void setHightlightFaceBrush(const QBrush& brush) { highlight_face_brush = brush; };
    const QBrush& highlightFaceBrush() const { return highlight_face_brush; }
    void setVisibleLabels(bool visible) { if (visible_labels != visible) { prepareGeometryChange(); }; visible_labels = visible; }
    void setVisibleEdges(bool visible) { if (visible_edges != visible) { prepareGeometryChange(); }; visible_edges = visible; }
    void setVisibleConstraints(bool visible) { if (visible_constraints != visible) { prepareGeometryChange(); }; visible_constraints = visible; }
    void setVisibleArcs(bool visible) { if (visible_arcs != visible) { prepareGeometryChange(); }; visible_arcs = visible; }
    void setVisibleHighlightCircle(bool visible) { if (visible_highlight_circle != visible) { prepareGeometryChange(); }; visible_highlight_circle = visible; }

    const NT& drawing_time_offset() const { return drawing_time_offset_; };
    void setTime(const NT& new_time) { now = new_time; modelChanged(); };
    void setComponent(int p_current_component) { current_component = p_current_component; modelChanged(); };
    void setDrawingOffset(const NT& new_offset) { drawing_time_offset_ = new_offset; modelChanged(); };
    NT draw_time() const { return now + drawing_time_offset_; };

    void modelChanged();
    void highlighted_clear() { highlighted.clear(); }
    void highlighted_add(const KineticTriangle * const i) { highlighted.insert(i); }
    bool highlighted_has(const KineticTriangle * const i) const { return highlighted.find(i) != highlighted.end(); }

  private:
    //static Point_2 get_drawing_pos_for_infinite_vertex(const KineticTriangle& t, unsigned idx, const NT& time);
    //static std::tuple<Point_2,Point_2,Point_2> get_drawing_pos_triangle_v(const KineticTriangle& t, const NT& time);
};
