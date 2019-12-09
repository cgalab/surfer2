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

#include "OffsetsGraphicsItem.h"

OffsetsGraphicsItem::
OffsetsGraphicsItem()
  : Base()
  , painterostream(0)
  , segments_pen(QPen(::Qt::gray, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
{
  modelChanged();
  setZValue(3);
}

void
OffsetsGraphicsItem::
paint(QPainter *painter, const QStyleOptionGraphicsItem * /*option*/, QWidget * /*widget*/) {
  painterostream = CGAL::Qt::PainterOstream<Kernel> (painter);

  painter->setPen(segmentsPen());
  for (const auto& family : offsets) {
    for (const auto& curve : family) {
      for (auto edge_it = curve.edges_begin(); edge_it != curve.edges_end(); ++edge_it) {
         painterostream << *edge_it;
      }
    }
  }
}

void
OffsetsGraphicsItem::
updateBoundingBox() {
  CGAL::Qt::Converter<Kernel> convert;
  prepareGeometryChange();

  if (offsets.size() == 0) {
    return;
  }

  assert(offsets[0].size() > 0);

  auto bb = offsets[0][0].bbox();
  for (const auto& family : offsets) {
    for (const auto& curve : family) {
      bb += curve.bbox();
    }
  }

  bounding_rect = convert(bb);
}

void
OffsetsGraphicsItem::
modelChanged() {
  prepareGeometryChange();
  updateBoundingBox();
}
