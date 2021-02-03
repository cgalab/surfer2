/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2018, 2019 Peter Palfrader
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

class SkeletonGraphicsItem :
  public CGAL::Qt::GraphicsItem
{
  private:
    using Base = CGAL::Qt::GraphicsItem;

  private:
    const SkeletonDCEL * const skeleton;
    QPen vertices_pen;
    QPen input_segments_pen;
    QPen segments_pen;
    QPen rays_pen;
    QPen labels_pen;
    bool visible_labels = false;


  protected:
    QRectF bounding_rect;
    void updateBoundingBox();

  public:
    SkeletonGraphicsItem(const SkeletonDCEL * const skeleton);

    QRectF boundingRect() const { return bounding_rect; };

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void setVerticesPen(const QPen& pen) { vertices_pen = pen; };
    void setInputSegmentsPen(const QPen& pen) { input_segments_pen = pen; };
    void setSegmentsPen(const QPen& pen) { segments_pen = pen; };
    void setRaysPen(const QPen& pen) { rays_pen = pen; };
    void setLabelsPen(const QPen& pen) { labels_pen = pen; };
    const QPen& verticesPen() const { return vertices_pen; }
    const QPen& inputSegmentsPen() const { return input_segments_pen; }
    const QPen& segmentsPen() const { return segments_pen; }
    const QPen& raysPen() const { return rays_pen; }
    const QPen& labelsPen() const { return labels_pen; }
    void setVisibleLabels(bool visible) { if (visible_labels != visible) { prepareGeometryChange(); }; visible_labels = visible; }

    void modelChanged();
};
