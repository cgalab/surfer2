/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2015 -- 2019 Peter Palfraader
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
#include <CGAL/Bbox_2.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Qt/Converter.h>
#include <QWidget>

#include "SkeletonGraphicsItem.h"

SkeletonGraphicsItem::
SkeletonGraphicsItem(const SkeletonDCEL * const skeleton_)
  : Base()
  , skeleton(skeleton_)
  , painterostream(0)
  , vertices_pen(QPen(::Qt::blue, 3))
  , segments_pen(QPen(::Qt::blue, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , rays_pen(QPen(QColor("#5599ff"), 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , labels_pen(QPen(Qt::black, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
{
  modelChanged();
  setZValue(2);
}

void
SkeletonGraphicsItem::
paint(QPainter *painter, const QStyleOptionGraphicsItem * /*option*/, QWidget * /*widget*/) {
  CGAL::Qt::Converter<Kernel> convert;

  painterostream = CGAL::Qt::PainterOstream<Kernel> (painter);
  int ray_length = 10;

  for (auto hit = skeleton->halfedges_begin(); hit != skeleton->halfedges_end(); ++hit) {
    if (hit > hit->opposite()) continue;

    const auto& arc = hit->curve();
    if (arc.type() == typeid(Segment_3)) {
      const Segment_3& s = boost::get<Segment_3>(arc);
      painter->setPen(segmentsPen());
      painterostream << project_plane(s);
    } else {
      assert(arc.type() == typeid(Ray_3));
      const Ray_3& r = boost::get<Ray_3>(arc);
      const Ray_2 r2 = project_plane(r);
      painter->setPen(raysPen());
      painterostream << Segment_2(r2.source(), r2.point(ray_length));
    }
  }

  painter->setPen(verticesPen());
  auto transform = painter->worldTransform();
  painter->resetTransform();
  for (auto vit = skeleton->vertices_begin(); vit != skeleton->vertices_end(); ++vit) {
    if (vit->has_null_point()) continue;
    QPointF point = transform.map(convert(project_plane(vit->point())));
    painter->drawPoint(point);
  }

  #ifndef NDEBUG
  if (visible_labels) {
    painter->setPen(labelsPen());
    QFont font(painter->font());

    font.setPointSize(8);
    painter->setFont(font);

    for (auto vit = skeleton->vertices_begin(); vit != skeleton->vertices_end(); ++vit) {
      if (vit->has_null_point()) continue;
      const QPointF p(transform.map(convert( project_plane(vit->point()) )));
      std::string t = "dv#"+std::to_string(vit->id);
      painter->drawText(p.x()+4, p.y(), QString::fromStdString(t));
    }

    for (auto hit = skeleton->halfedges_begin(); hit != skeleton->halfedges_end(); ++hit) {
      if (hit > hit->opposite()) continue;

      std::string t = "dh#(" + std::to_string(hit->id)+ "," + std::to_string(hit->opposite()->id) + ")";

      Point_2 pos;
      const auto& arc = hit->curve();
      if (arc.type() == typeid(Segment_3)) {
        const Segment_3& s = boost::get<Segment_3>(arc);
        const Segment_2 s2 = project_plane(s);
        pos = CGAL::midpoint(s2.source(), s2.target());
      } else {
        assert(arc.type() == typeid(Ray_3));
        const Ray_3& r = boost::get<Ray_3>(arc);
        const Ray_2 r2 = project_plane(r);
        pos = CGAL::midpoint(r2.source(), r2.point(ray_length));
      }
      const QPointF p(transform.map(convert( pos )));
      painter->drawText(p.x()+4, p.y(), QString::fromStdString(t));
    }
  }
  #endif
  painter->setWorldTransform(transform);
}

void
SkeletonGraphicsItem::
updateBoundingBox() {
  CGAL::Qt::Converter<Kernel> convert;
  prepareGeometryChange();

  auto vit = skeleton->vertices_begin();
  if (vit->has_null_point()) { ++vit; };
  assert(! vit->has_null_point());

  auto bb = project_plane(vit->point()).bbox();
  ++vit;
  for (; vit != skeleton->vertices_end(); ++vit) {
    if (vit->has_null_point()) continue;
    bb += project_plane(vit->point()).bbox();
  }

  bounding_rect = convert(bb);
}

void
SkeletonGraphicsItem::
modelChanged() {
  updateBoundingBox();
}
