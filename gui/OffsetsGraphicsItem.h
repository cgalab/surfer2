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

#include "surfgui.h"
#include "SkeletonDCEL.h"

class OffsetsGraphicsItem :
  public CGAL::Qt::GraphicsItem
{
  private:
    using Base = CGAL::Qt::GraphicsItem;

  private:
    std::vector<SkeletonDCEL::OffsetFamily> offsets;
    QPen segments_pen;

  protected:
    QRectF bounding_rect;
    void updateBoundingBox();

  public:
    OffsetsGraphicsItem();

    QRectF boundingRect() const { return bounding_rect; };

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void set_offsets(std::vector<SkeletonDCEL::OffsetFamily>&& o) {
      offsets.swap(o);
      modelChanged();
    }

    void setSegmentsPen(const QPen& pen) { segments_pen = pen; };
    const QPen& segmentsPen() const { return segments_pen; }

    void modelChanged();
};
